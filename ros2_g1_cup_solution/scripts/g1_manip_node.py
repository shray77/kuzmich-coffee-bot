"""
ROS2 node: G1ManipNode

Drop-in replacement for the old `g1_manip_node.py` in repo `kuzmich-coffee-bot`
(`ros2_g1_cup_solution/scripts/g1_manip_node.py`). Removes the dependency on
the non-existent `unitree_sdk2py.g1.hand.hand_client.HandClient` and drives
the Inspire RH56E2 hand directly over ModbusTCP.

External interface (unchanged — drop-in):
  * /g1_manip/grasp_cup   std_srvs/Trigger   — close the hand with tactile feedback
  * /g1_manip/release_cup std_srvs/Trigger   — open the hand

Grasp algorithm (per TЗ, adapted from force-in-grams to tactile):
  1. Open the hand fully (ANGLE_SET = max).
  2. Step-close in small decrements; after each step, read get_touch_state().
  3. Stop when:
       * contact_pad_count(touch, contact_threshold) >= min_contact_pads
         AND peak_fingertip_pressure(touch) >= hold_threshold
       → grasp achieved, freeze command, return success.
  4. Safety: if any single pad delta between two consecutive samples exceeds
     spike_threshold, immediately back off by `back_off_step` and return
     failure (or success-with-warning, controlled by `fail_on_spike`).
  5. If the hand reaches fully closed (q == 0) without achieving contact,
     return failure.
  6. release_cup: write ANGLE_SET = max, wait `release_settle_time`, return
     success.

A missing right hand is not an error: the node starts with whatever hand(s)
are reachable and operates left-only if .211 is offline (TЗ explicitly allows
this). The target hand for the grasp service is selected by the `target_hand`
parameter.

SAFETY / VERIFICATION (read before first run on the real robot):
  * ANGLE_SET register is set to 1009 (Inspire RH56 series documented value).
    Before enabling motion, run `inspire_hand_probe.py --ip <hand_ip>` and
    confirm that the no-op write test does not move the hand. If the probe
    raises "illegal data address", ANGLE_SET_REG in inspire_e2_hand.py is
    wrong — find the correct address in `thirdparty/inspire-api/` or the
    C++ controller `src/cpp/hands/inspire_e2/inspire_e2_hand.cpp` and edit
    the constant in place.
  * `enable_motion` parameter defaults to False. Set it to True only after
    the probe passes. With enable_motion=False the node reads state and
    answers services, but never writes ANGLE_SET — safe for first bring-up.
  * `max_close_steps` caps the closing loop so the hand cannot grind against
    a stuck object forever.
"""

from __future__ import annotations

import os
import sys
import time
from typing import List, Optional

# Allow running this file from the repo's scripts/ dir without installation.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from inspire_e2_hand import (  # noqa: E402
    InspireE2Hand,
    N_JOINTS,
    N_PADS,
    TIP_PAD_INDICES,
)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_srvs.srv import Trigger
    from std_srvs.srv import SetBool  # noqa: F401  (kept for future use)
    _RCLPY_OK = True
except ImportError:
    # Allow `python g1_manip_node.py --probe` to run without ROS2 installed.
    _RCLPY_OK = False
    Node = object  # type: ignore[misc, assignment]


# ---------------------------------------------------------------------- #
# Grasp result codes (mapped to Trigger.success + Trigger.message)
# ---------------------------------------------------------------------- #
GRASP_OK                = "OK"
GRASP_NO_CONTACT        = "NO_CONTACT"
GRASP_SPIKE_BACKOFF     = "SPIKE_BACKOFF"
GRASP_HAND_OFFLINE      = "HAND_OFFLINE"
GRASP_MOTION_DISABLED   = "MOTION_DISABLED"
GRASP_TIMEOUT           = "TIMEOUT"
RELEASE_OK              = "RELEASE_OK"
RELEASE_HAND_OFFLINE    = "RELEASE_HAND_OFFLINE"


class G1ManipNode(Node):  # type: ignore[misc]
    def __init__(self):
        if not _RCLPY_OK:
            raise RuntimeError(
                "rclpy not available — install ROS2 or run inspire_hand_probe.py "
                "instead of this node."
            )
        super().__init__("g1_manip_node")

        # ---------------------------------------------------------------- #
        # Parameters
        # ---------------------------------------------------------------- #
        self.declare_parameter("left_hand_ip",  "192.168.123.210")
        self.declare_parameter("right_hand_ip", "192.168.123.211")
        self.declare_parameter("hand_port",     6000)
        self.declare_parameter("hand_unit_id",  1)
        self.declare_parameter("target_hand",   "left")    # 'left' | 'right' | 'both'
        self.declare_parameter("enable_motion", False)     # SAFETY: must flip to True
        self.declare_parameter("socket_timeout", 0.5)

        # Grasp tuning
        self.declare_parameter("grasp_step",            0.05)   # decrement per step (0..1)
        self.declare_parameter("grasp_step_delay",      0.08)   # seconds between steps
        self.declare_parameter("hold_threshold",        0.05)   # peak fingertip pressure to consider 'holding'
        self.declare_parameter("contact_threshold",     0.03)   # per-pad pressure to count as 'contact'
        self.declare_parameter("min_contact_pads",      3)      # how many pads must be in contact
        self.declare_parameter("spike_threshold",       0.30)   # single-step per-pad delta => back off
        self.declare_parameter("back_off_step",         0.20)   # how much to reopen on spike
        self.declare_parameter("fail_on_spike",         True)   # True => return failure on spike
        self.declare_parameter("max_close_steps",       40)     # hard cap on closing loop
        self.declare_parameter("open_settle_time",      0.6)    # wait after open_fully()
        self.declare_parameter("release_settle_time",   0.6)    # wait after release_cup
        self.declare_parameter("poll_touch_pre_steps",  1)      # baseline samples before closing

        # ---------------------------------------------------------------- #
        # Wire hands
        # ---------------------------------------------------------------- #
        port       = int(self.get_parameter("hand_port").value)
        unit_id    = int(self.get_parameter("hand_unit_id").value)
        timeout    = float(self.get_parameter("socket_timeout").value)
        left_ip    = str(self.get_parameter("left_hand_ip").value)
        right_ip   = str(self.get_parameter("right_hand_ip").value)

        self._left = InspireE2Hand(host=left_ip, port=port, unit_id=unit_id,
                                   timeout=timeout, name="left")
        self._right = InspireE2Hand(host=right_ip, port=port, unit_id=unit_id,
                                    timeout=timeout, name="right")

        # Try to connect left first; right is best-effort.
        if not self._left.connect():
            self.get_logger().warn(
                f"Left hand at {left_ip}:{port} not reachable — grasp service "
                "will return HAND_OFFLINE until it comes back."
            )
        if not self._right.connect():
            self.get_logger().warn(
                f"Right hand at {right_ip}:{port} not reachable (expected per "
                "TЗ — right hand is offline until physically restored)."
            )

        # ---------------------------------------------------------------- #
        # ROS2 services (drop-in compatible with the old node)
        # ---------------------------------------------------------------- #
        self.srv_grasp  = self.create_service(
            Trigger, "/g1_manip/grasp_cup",  self._handle_grasp)
        self.srv_release = self.create_service(
            Trigger, "/g1_manip/release_cup", self._handle_release)

        self.get_logger().info(
            f"G1ManipNode up. target_hand="
            f"{self.get_parameter('target_hand').value} "
            f"enable_motion={self.get_parameter('enable_motion').value}"
        )

    # ------------------------------------------------------------------ #
    # Service handlers
    # ------------------------------------------------------------------ #
    def _target_hands(self) -> List[InspireE2Hand]:
        sel = str(self.get_parameter("target_hand").value).lower()
        if sel == "left":
            return [self._left]
        if sel == "right":
            return [self._right]
        if sel == "both":
            return [self._left, self._right]
        self.get_logger().warn(f"Unknown target_hand='{sel}', defaulting to left")
        return [self._left]

    def _handle_grasp(self, request, response):
        if not bool(self.get_parameter("enable_motion").value):
            response.success = False
            response.message = GRASP_MOTION_DISABLED
            return response

        any_contact = False
        any_failure = False
        msgs = []
        for hand in self._target_hands():
            code, msg = self._grasp_one(hand)
            msgs.append(f"[{hand.name}] {msg}")
            if code == GRASP_OK:
                any_contact = True
            elif code in (GRASP_SPIKE_BACKOFF, GRASP_NO_CONTACT,
                          GRASP_HAND_OFFLINE, GRASP_TIMEOUT):
                any_failure = True

        response.success = any_contact and not any_failure
        response.message = " | ".join(msgs) if msgs else GRASP_HAND_OFFLINE
        return response

    def _handle_release(self, request, response):
        if not bool(self.get_parameter("enable_motion").value):
            response.success = False
            response.message = GRASP_MOTION_DISABLED
            return response

        any_ok = False
        msgs = []
        for hand in self._target_hands():
            if not hand.is_connected:
                msgs.append(f"[{hand.name}] {RELEASE_HAND_OFFLINE}")
                continue
            ok = hand.open_fully()
            if ok:
                any_ok = True
                msgs.append(f"[{hand.name}] {RELEASE_OK}")
            else:
                msgs.append(f"[{hand.name}] WRITE_FAIL")
        # Give the hand time to physically open.
        time.sleep(float(self.get_parameter("release_settle_time").value))
        response.success = any_ok
        response.message = " | ".join(msgs) if msgs else RELEASE_HAND_OFFLINE
        return response

    # ------------------------------------------------------------------ #
    # Core grasp logic (per hand)
    # ------------------------------------------------------------------ #
    def _grasp_one(self, hand: InspireE2Hand):
        if not hand.is_connected:
            # Best-effort reconnect — service may have been called after the
            # hand was rebooted.
            if not hand.connect():
                return GRASP_HAND_OFFLINE, GRASP_HAND_OFFLINE

        hold_thresh    = float(self.get_parameter("hold_threshold").value)
        contact_thresh = float(self.get_parameter("contact_threshold").value)
        min_pads       = int(self.get_parameter("min_contact_pads").value)
        spike_thresh   = float(self.get_parameter("spike_threshold").value)
        back_off       = float(self.get_parameter("back_off_step").value)
        fail_on_spike  = bool(self.get_parameter("fail_on_spike").value)
        step           = float(self.get_parameter("grasp_step").value)
        step_delay     = float(self.get_parameter("grasp_step_delay").value)
        max_steps      = int(self.get_parameter("max_close_steps").value)
        open_settle    = float(self.get_parameter("open_settle_time").value)
        n_pre          = max(1, int(self.get_parameter("poll_touch_pre_steps").value))

        # 1) Open fully and let it settle.
        if not hand.open_fully():
            return GRASP_HAND_OFFLINE, "OPEN_FAIL"
        time.sleep(open_settle)

        # 2) Baseline touch (so spike detection has a reference).
        prev_touch = hand.get_touch_state()
        for _ in range(n_pre - 1):
            time.sleep(step_delay)
            prev_touch = hand.get_touch_state()

        # 3) Step-close loop.
        q = 1.0
        for i in range(max_steps):
            q = max(0.0, q - step)
            if not hand.set_uniform(q):
                return GRASP_HAND_OFFLINE, "WRITE_FAIL"

            time.sleep(step_delay)
            touch = hand.get_touch_state()

            # 3a) Spike check (per-pad delta)
            if len(touch) == N_PADS and len(prev_touch) == N_PADS:
                max_delta = max((abs(touch[k] - prev_touch[k])
                                 for k in range(N_PADS)), default=0.0)
                if max_delta > spike_thresh:
                    # Back off immediately.
                    back_q = min(1.0, q + back_off)
                    hand.set_uniform(back_q)
                    msg = (f"SPIKE step={i} q={q:.2f} "
                           f"delta={max_delta:.3f} back_off_to={back_q:.2f}")
                    self.get_logger().warn(f"[{hand.name}] {msg}")
                    if fail_on_spike:
                        return GRASP_SPIKE_BACKOFF, msg
                    # else: keep going but reset baseline
                    prev_touch = touch
                    continue

            # 3b) Hold check
            peak_tip = InspireE2Hand.peak_fingertip_pressure(touch)
            n_contact = InspireE2Hand.contact_pad_count(touch, contact_thresh)
            if peak_tip >= hold_thresh and n_contact >= min_pads:
                msg = (f"OK step={i} q={q:.2f} peak_tip={peak_tip:.3f} "
                       f"pads={n_contact}")
                self.get_logger().info(f"[{hand.name}] {msg}")
                return GRASP_OK, msg

            prev_touch = touch

            if q <= 0.0:
                break

        # 4) Ran out of steps without contact.
        return GRASP_NO_CONTACT, f"NO_CONTACT final_q={q:.2f}"


# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #
def main(argv=None):
    if not _RCLPY_OK:
        print("ERROR: rclpy (ROS2) not available. Use inspire_hand_probe.py "
              "for hardware verification without ROS2.", file=sys.stderr)
        return 1
    rclpy.init(args=argv)
    node = G1ManipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
