"""
Inspire RH56E2 hand controller (Python port of the verified C++ controller).

Mirrors the conventions used by `src/cpp/hands/inspire_e2/inspire_e2_hand.cpp`:

  * get_hand_state()  -> 6 float  (joint angles, 0..1, 1.0 = fully open)
                          order: [pinky, ring, middle, index, thumbBend, thumbRot]
                          value = ANGLE_ACT / 1000

  * set_command(q)    -> q in 0..1 (1.0 = open) per joint, written to ANGLE_SET
                          as uint16 (q * 1000). Single-finger (per-joint) command
                          is supported. For a uniform open/close use set_uniform(q).

  * get_touch_state() -> 17 float per hand (0..1)
                          pads: [pinky/ring/middle/index × (top,tip,base),
                                 thumb × (top,tip,mid,base),
                                 palm]
                          value = peak pressure on pad / touch_full_scale(4095)

Hardware / register map (confirmed by TЗ unless marked VERIFY):
  HAND_ID            = 1000     (read, 1 reg)            CONFIRMED
  ANGLE_ACT          = 1546     (read, 6 regs)           CONFIRMED
  ANGLE_SET          = 1009     (write, 6 regs)          ** VERIFY ON HARDWARE **
                                                          (documented for
                                                           Inspire RH56 series;
                                                           cross-check against
                                                           inspire-api / C++
                                                           controller before
                                                           sending real motion)
  TACTILE_BASE       = 3000     (read, 17 regs, peaks)   ** VERIFY LAYOUT **
                                                          (TЗ says range
                                                           3000-4900; this
                                                           controller reads
                                                           17 consecutive peak
                                                           registers starting
                                                           at 3000 — adjust
                                                           TACTILE_BASE if the
                                                           C++ code reads a
                                                           different offset)
  TOUCH_FULL_SCALE   = 4095                               CONFIRMED (TЗ)

A missing / powered-off hand is represented by `is_connected() == False`;
calls then return zero-filled state vectors without raising (per TЗ
«Отсутствующая/выключенная рука → просто нули, без падения»).
"""

from __future__ import annotations

import logging
import threading
import time
from typing import List, Optional

from inspire_modbus import ModbusTCPClient, ModbusError

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------- #
# Register map (see module docstring for verification status)
# ---------------------------------------------------------------------- #
HAND_ID_REG        = 1000
ANGLE_ACT_REG      = 1546     # read  6 regs
ANGLE_SET_REG      = 1009     # write 6 regs — ** VERIFY ON HARDWARE **
TACTILE_BASE_REG   = 3000     # read 17 regs (peak per pad) — ** VERIFY LAYOUT **
TOUCH_FULL_SCALE   = 4095

N_JOINTS = 6
N_PADS   = 17

# Tactile pad order (matches TЗ / C++ controller convention):
#   0  pinky  top
#   1  pinky  tip
#   2  pinky  base
#   3  ring   top
#   4  ring   tip
#   5  ring   base
#   6  middle top
#   7  middle tip
#   8  middle base
#   9  index  top
#  10  index  tip
#  11  index  base
#  12  thumb  top
#  13  thumb  tip
#  14  thumb  mid
#  15  thumb  base
#  16  palm
TIP_PAD_INDICES = (1, 4, 7, 10, 13)   # 5 fingertip pads
PALM_PAD_INDEX  = 16


class InspireE2Hand:
    """
    Single-hand controller. One instance = one TCP socket = one hand.

    The class is thread-safe. A background poller can call get_touch_state()
    while a foreground caller invokes set_command(); both are serialised by
    the underlying ModbusTCPClient lock.

    Parameters
    ----------
    host, port, unit_id, timeout :
        Forwarded to ModbusTCPClient. Defaults match the left Inspire RH56E2
        on the G1 (192.168.123.210:6000, unit_id=1).
    name :
        Free-form label used in logs (e.g. "left" / "right").
    auto_connect :
        If True (default), the first call lazily opens the socket. If the
        socket is not open and auto_connect is False, calls return zeros.
    """

    def __init__(self, host: str = "192.168.123.210", port: int = 6000,
                 unit_id: int = 1, timeout: float = 0.5,
                 name: str = "left", auto_connect: bool = True):
        self.name = name
        self.host = host
        self.port = port
        self._client = ModbusTCPClient(host, port, unit_id, timeout)
        self._auto_connect = auto_connect
        self._connected = False
        self._hand_id: Optional[int] = None
        # Last known ANGLE_SET we wrote — used by probe/safety logic.
        self._last_set_command: List[float] = [1.0] * N_JOINTS
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Connection
    # ------------------------------------------------------------------ #
    def connect(self) -> bool:
        """Open the socket and verify the hand answers with a HAND_ID read."""
        try:
            if not self._client.connect():
                self._connected = False
                return False
            hid = self._client.read_holding_registers(HAND_ID_REG, count=1)
            self._hand_id = hid[0] if hid else None
            self._connected = True
            logger.info("[%s] Inspire RH56E2 connected at %s:%d (HAND_ID=%s)",
                        self.name, self.host, self.port, self._hand_id)
            return True
        except (OSError, ModbusError) as exc:
            logger.warning("[%s] connect failed: %s", self.name, exc)
            self._connected = False
            return False

    def disconnect(self) -> None:
        self._client.close()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def hand_id(self) -> Optional[int]:
        return self._hand_id

    # ------------------------------------------------------------------ #
    # State reads
    # ------------------------------------------------------------------ #
    def _safe_read(self, address: int, count: int) -> List[int]:
        """Read registers; return [] on any failure (and mark disconnected)."""
        if not self._connected:
            if self._auto_connect:
                if not self.connect():
                    return []
            else:
                return []
        try:
            return self._client.read_holding_registers(address, count)
        except (OSError, ModbusError) as exc:
            logger.warning("[%s] read reg %d count %d failed: %s",
                           self.name, address, count, exc)
            # Force a reconnect on next call.
            self._connected = False
            self._client.close()
            return []

    def get_hand_state(self) -> List[float]:
        """6 joint angles, normalised 0..1 (1.0 = fully open).

        Returns zeros if the hand is not reachable.
        """
        raw = self._safe_read(ANGLE_ACT_REG, N_JOINTS)
        if len(raw) != N_JOINTS:
            return [0.0] * N_JOINTS
        # value = ANGLE_ACT / 1000, clamp to [0, 1]
        return [min(1.0, max(0.0, v / 1000.0)) for v in raw]

    def get_touch_state(self) -> List[float]:
        """17 tactile-pad peaks, normalised 0..1 (peak / TOUCH_FULL_SCALE).

        Returns zeros if the hand is not reachable.
        """
        raw = self._safe_read(TACTILE_BASE_REG, N_PADS)
        if len(raw) != N_PADS:
            return [0.0] * N_PADS
        return [min(1.0, max(0.0, v / float(TOUCH_FULL_SCALE))) for v in raw]

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def set_command(self, q: List[float]) -> bool:
        """Write per-joint target angles (0..1, 1.0 = open).

        Returns True on successful FC16 round-trip, False otherwise.
        Length of `q` must be 6 (N_JOINTS).
        """
        if len(q) != N_JOINTS:
            raise ValueError(f"set_command expects {N_JOINTS} values, got {len(q)}")
        # Clamp + scale to uint16 in [0, 1000]
        raw = [int(round(min(1.0, max(0.0, v)) * 1000.0)) for v in q]
        return self._write_command(raw, q)

    def set_uniform(self, q: float) -> bool:
        """Convenience: same target for all 6 joints (1.0 = open, 0.0 = closed)."""
        return self.set_command([q] * N_JOINTS)

    def open_fully(self) -> bool:
        return self.set_uniform(1.0)

    def close_fully(self) -> bool:
        return self.set_uniform(0.0)

    def _write_command(self, raw_values: List[int],
                       human_values: List[float]) -> bool:
        if not self._connected:
            if self._auto_connect:
                if not self.connect():
                    return False
            else:
                return False
        try:
            self._client.write_multiple_registers(ANGLE_SET_REG, raw_values)
            self._last_set_command = list(human_values)
            return True
        except (OSError, ModbusError) as exc:
            logger.warning("[%s] write ANGLE_SET failed: %s", self.name, exc)
            self._connected = False
            self._client.close()
            return False

    # ------------------------------------------------------------------ #
    # Helpers for the grasp logic
    # ------------------------------------------------------------------ #
    @staticmethod
    def peak_fingertip_pressure(touch: List[float]) -> float:
        """Max pressure across the 5 fingertip pads (top-of-finger contacts)."""
        if len(touch) != N_PADS:
            return 0.0
        return max((touch[i] for i in TIP_PAD_INDICES), default=0.0)

    @staticmethod
    def peak_pressure(touch: List[float]) -> float:
        """Max pressure across all 17 pads."""
        return max(touch) if touch else 0.0

    @staticmethod
    def contact_pad_count(touch: List[float], threshold: float = 0.05) -> int:
        """Number of pads above `threshold` — useful for 'is the cup held?'."""
        return sum(1 for v in touch if v > threshold)


__all__ = [
    "InspireE2Hand",
    # register constants
    "HAND_ID_REG", "ANGLE_ACT_REG", "ANGLE_SET_REG", "TACTILE_BASE_REG",
    "TOUCH_FULL_SCALE", "N_JOINTS", "N_PADS",
    "TIP_PAD_INDICES", "PALM_PAD_INDEX",
]
