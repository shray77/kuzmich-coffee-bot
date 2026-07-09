"""
Zero-dependency ModbusTCP client for the Inspire RH56E2 hand.

Implemented on raw POSIX sockets because the target Jetson has no reliable
pip/internet access (see TЗ: «свой Modbus-клиент тоже стоит делать на чистых
сокетах, без pymodbus»).

Protocol:
  * Read Holding Registers  -> Modbus function code 0x03 (FC03)
  * Write Multiple Registers -> Modbus function code 0x10 (FC16)

This matches what the verified C++ controller
(`src/cpp/hands/inspire_e2/inspire_e2_hand.cpp`) does — same FC03/FC16,
same raw register addresses, no Unitree SDK in the loop.

The class is thread-safe at the request level (a single lock serialises
socket access). Reconnection is lazy: if a socket is closed/stale, the
next call will re-establish it.
"""

from __future__ import annotations

import logging
import socket
import struct
import time
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)


class ModbusError(RuntimeError):
    """Raised when the device returns a Modbus exception or an invalid frame."""

    def __init__(self, message: str, function_code: Optional[int] = None,
                 exception_code: Optional[int] = None):
        super().__init__(message)
        self.function_code = function_code
        self.exception_code = exception_code

    @property
    def is_exception(self) -> bool:
        return self.exception_code is not None


# Modbus exception codes (subset that matters for us)
EX_ILLEGAL_FUNCTION = 0x01
EX_ILLEGAL_DATA_ADDRESS = 0x02
EX_ILLEGAL_DATA_VALUE = 0x03
EX_SLAVE_DEVICE_FAILURE = 0x04

_EXC_TEXT = {
    EX_ILLEGAL_FUNCTION: "illegal function",
    EX_ILLEGAL_DATA_ADDRESS: "illegal data address",
    EX_ILLEGAL_DATA_VALUE: "illegal data value",
    EX_SLAVE_DEVICE_FAILURE: "slave device failure",
}


class ModbusTCPClient:
    """
    Minimal ModbusTCP client (FC03 / FC16 only).

    Parameters
    ----------
    host : str
        IP address of the hand. For the left Inspire RH56E2 on the G1 this is
        192.168.123.210 by convention (see TЗ).
    port : int
        ModbusTCP port. Inspire hand exposes 6000.
    unit_id : int
        Modbus slave/unit id. Inspire hands answer on unit_id = 1 by default,
        but the value is exposed so it can be overridden if a different hand_id
        was flashed.
    timeout : float
        Per-socket-operation timeout in seconds.
    """

    def __init__(self, host: str, port: int = 6000, unit_id: int = 1,
                 timeout: float = 0.5):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._tx_seq = 0  # Modbus/TCP transaction id
        # Single lock — modbus is a request/response protocol and the
        # underlying socket is not safe for concurrent use.
        import threading
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Connection management
    # ------------------------------------------------------------------ #
    def connect(self) -> bool:
        """Open a TCP connection to the device. Returns True on success."""
        with self._lock:
            return self._connect_locked()

    def _connect_locked(self) -> bool:
        if self._sock is not None:
            return True
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            self._sock = sock
            logger.debug("ModbusTCP connected to %s:%d", self.host, self.port)
            return True
        except OSError as exc:
            logger.debug("ModbusTCP connect to %s:%d failed: %s",
                         self.host, self.port, exc)
            self._sock = None
            return False

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def is_alive(self) -> bool:
        """Cheap liveness probe: try a 1-register read of HAND_ID (reg 1000).

        Returns True if a valid FC03 response came back.
        """
        try:
            self.read_holding_registers(1000, count=1)
            return True
        except (ModbusError, OSError) as exc:
            logger.debug("is_alive probe failed on %s: %s", self.host, exc)
            return False

    # ------------------------------------------------------------------ #
    # Modbus/TCP framing
    # ------------------------------------------------------------------ #
    def _build_mbap_header(self, length: int) -> bytes:
        self._tx_seq = (self._tx_seq + 1) & 0xFFFF
        # MBAP: transaction_id (2) | protocol_id (2, always 0) | length (2) | unit_id (1)
        return struct.pack(">HHHB", self._tx_seq, 0x0000, length, self.unit_id)

    def _send_recv(self, pdu: bytes) -> bytes:
        """Send a PDU, return the response PDU (without MBAP header)."""
        if not self._connect_locked():
            raise OSError(f"modbus socket to {self.host}:{self.port} not open")

        mbap = self._build_mbap_header(length=len(pdu) + 1)
        frame = mbap + pdu

        try:
            self._sock.sendall(frame)
        except OSError as exc:
            self._close_locked()
            raise OSError(f"modbus send failed: {exc}") from exc

        # Read MBAP header (7 bytes)
        try:
            header = self._recv_exact(7)
        except OSError as exc:
            self._close_locked()
            raise OSError(f"modbus recv header failed: {exc}") from exc

        if len(header) != 7:
            self._close_locked()
            raise ModbusError(f"short MBAP header ({len(header)} bytes)")

        tx_id, proto_id, length, unit_id = struct.unpack(">HHHB", header)
        if proto_id != 0:
            self._close_locked()
            raise ModbusError(f"bad protocol id {proto_id}")
        if tx_id != self._tx_seq:
            # Not fatal — Inspire hands sometimes reuse seq ids — but log it.
            logger.debug("modbus tx_id mismatch: sent %d, got %d",
                         self._tx_seq, tx_id)
        if unit_id != self.unit_id:
            self._close_locked()
            raise ModbusError(f"bad unit id {unit_id}, expected {self.unit_id}")

        # length includes unit_id (1 byte) + PDU. We already consumed unit_id.
        pdu_len = length - 1
        if pdu_len <= 0:
            self._close_locked()
            raise ModbusError(f"bad mbap length {length}")

        try:
            resp_pdu = self._recv_exact(pdu_len)
        except OSError as exc:
            self._close_locked()
            raise OSError(f"modbus recv pdu failed: {exc}") from exc

        return resp_pdu

    def _recv_exact(self, n: int) -> bytes:
        """Read exactly n bytes from the socket; returns fewer only on EOF."""
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
        return bytes(buf)

    # ------------------------------------------------------------------ #
    # Public Modbus operations
    # ------------------------------------------------------------------ #
    def read_holding_registers(self, address: int, count: int = 1) -> List[int]:
        """FC03 — Read `count` holding registers starting at `address`.

        Returns a list of uint16 values (length == count).
        """
        if not (1 <= count <= 125):
            raise ValueError(f"FC03 count out of range: {count}")
        if not (0 <= address <= 0xFFFF):
            raise ValueError(f"FC03 address out of range: {address}")

        pdu = struct.pack(">BHH", 0x03, address, count)
        with self._lock:
            resp = self._send_recv(pdu)

        if not resp:
            raise ModbusError("empty FC03 response")
        func = resp[0]
        if func & 0x80:
            exc_code = resp[1] if len(resp) > 1 else 0
            raise ModbusError(
                f"FC03 exception 0x{exc_code:02X} "
                f"({_EXC_TEXT.get(exc_code, 'unknown')}) at addr {address}",
                function_code=0x03, exception_code=exc_code,
            )
        if func != 0x03:
            raise ModbusError(f"FC03 unexpected function 0x{func:02X}")
        if len(resp) < 2:
            raise ModbusError("FC03 response too short")
        byte_count = resp[1]
        if byte_count != count * 2:
            raise ModbusError(
                f"FC03 byte count mismatch: expected {count*2}, got {byte_count}"
            )
        payload = resp[2:2 + byte_count]
        if len(payload) != byte_count:
            raise ModbusError("FC03 truncated payload")
        return [struct.unpack(">H", payload[i:i+2])[0]
                for i in range(0, byte_count, 2)]

    def write_multiple_registers(self, address: int,
                                 values: List[int]) -> None:
        """FC16 — Write multiple registers. Values are uint16."""
        count = len(values)
        if not (1 <= count <= 123):
            raise ValueError(f"FC16 count out of range: {count}")
        if not (0 <= address <= 0xFFFF):
            raise ValueError(f"FC16 address out of range: {address}")
        for v in values:
            if not (0 <= v <= 0xFFFF):
                raise ValueError(f"FC16 value out of uint16 range: {v}")

        byte_count = count * 2
        pdu = struct.pack(">BHHB", 0x10, address, count, byte_count)
        for v in values:
            pdu += struct.pack(">H", v)

        with self._lock:
            resp = self._send_recv(pdu)

        if not resp:
            raise ModbusError("empty FC16 response")
        func = resp[0]
        if func & 0x80:
            exc_code = resp[1] if len(resp) > 1 else 0
            raise ModbusError(
                f"FC16 exception 0x{exc_code:02X} "
                f"({_EXC_TEXT.get(exc_code, 'unknown')}) at addr {address}",
                function_code=0x10, exception_code=exc_code,
            )
        if func != 0x10:
            raise ModbusError(f"FC16 unexpected function 0x{func:02X}")
        if len(resp) < 5:
            raise ModbusError("FC16 response too short")
        addr_echo, count_echo = struct.unpack(">HH", resp[1:5])
        if addr_echo != address or count_echo != count:
            raise ModbusError(
                f"FC16 echo mismatch: addr {addr_echo}/{address}, "
                f"count {count_echo}/{count}"
            )

    # ------------------------------------------------------------------ #
    # Convenience: context manager
    # ------------------------------------------------------------------ #
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


__all__ = ["ModbusTCPClient", "ModbusError"]
