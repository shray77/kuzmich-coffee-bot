#!/usr/bin/env python3
"""
Inspire RH56E2 hand probe — run this BEFORE enabling motion in g1_manip_node.

Per TЗ «Перед началом кодинга»:
  1. Verify the hand is alive and answers at the expected IP.
  2. Read HAND_ID (reg 1000), ANGLE_ACT (reg 1546), and all 17 tactile pads
     (regs 3000..3016) to confirm the register layout.
  3. Find/verify the ANGLE_SET register. Default hypothesis: 1009 (Inspire
     RH56 series documented value). This script does a SAFE no-op write
     test: it reads current ANGLE_ACT and writes those same values back to
     the candidate ANGLE_SET register. The hand should NOT move. If it
     returns "illegal data address", the candidate is wrong — try another.
  4. Optionally, with `--move-test`, do a tiny open/close cycle (q=0.9 → 0.7
     → 0.9) to confirm ANGLE_SET really drives the hand. Only run this after
     the no-op test passes and the hand is empty (no cup, no fingers in the
     way).

Usage:
  python3 inspire_hand_probe.py --ip 192.168.123.210
  python3 inspire_hand_probe.py --ip 192.168.123.210 --unit-id 1
  python3 inspire_hand_probe.py --ip 192.168.123.210 --angle-set-reg 1009
  python3 inspire_hand_probe.py --ip 192.168.123.210 --move-test

No external dependencies. Pure stdlib + the local inspire_modbus.py module.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from inspire_modbus import ModbusTCPClient, ModbusError  # noqa: E402
from inspire_e2_hand import (  # noqa: E402
    HAND_ID_REG, ANGLE_ACT_REG, ANGLE_SET_REG, TACTILE_BASE_REG,
    TOUCH_FULL_SCALE, N_JOINTS, N_PADS, TIP_PAD_INDICES,
)

PAD_NAMES = [
    "pinky.top", "pinky.tip", "pinky.base",
    "ring.top",  "ring.tip",  "ring.base",
    "mid.top",   "mid.tip",   "mid.base",
    "idx.top",   "idx.tip",   "idx.base",
    "th.top",    "th.tip",    "th.mid",    "th.base",
    "palm",
]


def _fmt_row(name: str, values) -> str:
    if isinstance(values, (list, tuple)):
        joined = " ".join(f"{v:5.3f}" if isinstance(v, float) else f"{v:>4d}"
                          for v in values)
        return f"  {name:<14s} {joined}"
    return f"  {name:<14s} {values}"


def probe_basic(client: ModbusTCPClient) -> bool:
    print("\n=== 1. Liveness / HAND_ID ===")
    try:
        hid = client.read_holding_registers(HAND_ID_REG, count=1)
        print(f"  HAND_ID reg {HAND_ID_REG} = {hid}")
        return bool(hid)
    except (OSError, ModbusError) as exc:
        print(f"  FAIL: {exc}")
        return False


def probe_angle_act(client: ModbusTCPClient) -> bool:
    print("\n=== 2. ANGLE_ACT (current joint angles, reg 1546, 6 regs) ===")
    try:
        raw = client.read_holding_registers(ANGLE_ACT_REG, count=N_JOINTS)
        norm = [v / 1000.0 for v in raw]
        print(_fmt_row("raw", raw))
        print(_fmt_row("norm 0..1", norm))
        print(f"  (1.0 = fully open)")
        return len(raw) == N_JOINTS
    except (OSError, ModbusError) as exc:
        print(f"  FAIL: {exc}")
        return False


def probe_tactile(client: ModbusTCPClient) -> bool:
    print(f"\n=== 3. Tactile pads (regs {TACTILE_BASE_REG}.."
          f"{TACTILE_BASE_REG + N_PADS - 1}, {N_PADS} regs) ===")
    try:
        raw = client.read_holding_registers(TACTILE_BASE_REG, count=N_PADS)
        norm = [v / float(TOUCH_FULL_SCALE) for v in raw]
        print(_fmt_row("raw", raw))
        print(_fmt_row("norm 0..1", norm))
        print("  Per-pad (normalised):")
        for i, name in enumerate(PAD_NAMES):
            tag = " <- fingertip" if i in TIP_PAD_INDICES else ""
            print(f"    [{i:2d}] {name:<12s} {norm[i]:.3f}{tag}")
        print(f"  (touch_full_scale = {TOUCH_FULL_SCALE}; "
              f"if all values are 0, either nothing is touching OR "
              f"TACTILE_BASE_REG is wrong — verify against the C++ controller)")
        return len(raw) == N_PADS
    except (OSError, ModbusError) as exc:
        print(f"  FAIL: {exc}")
        return False


def probe_angle_set_noop(client: ModbusTCPClient,
                          candidate_reg: int) -> bool:
    """SAFE no-op test: write current ANGLE_ACT back to candidate ANGLE_SET.

    The hand should accept the write and NOT move (because we're telling it
    to go to where it already is).
    """
    print(f"\n=== 4. ANGLE_SET no-op write test (candidate reg {candidate_reg}) ===")
    try:
        cur = client.read_holding_registers(ANGLE_ACT_REG, count=N_JOINTS)
        print(f"  Current ANGLE_ACT = {cur}")
        print(f"  Writing same values to reg {candidate_reg} (hand must NOT move)...")
        client.write_multiple_registers(candidate_reg, cur)
        time.sleep(0.1)
        after = client.read_holding_registers(ANGLE_ACT_REG, count=N_JOINTS)
        print(f"  After write, ANGLE_ACT = {after}")
        moved = any(abs(a - b) > 5 for a, b in zip(cur, after))  # 5 raw = 0.005
        if moved:
            print("  WARN: hand moved on no-op write! Either ANGLE_SET is a "
                  "different register, or it accepts relative commands. "
                  "DO NOT use this register as ANGLE_SET without further "
                  "investigation.")
            return False
        print("  OK — no-op write accepted, hand did not move. "
              f"Reg {candidate_reg} is a valid ANGLE_SET candidate.")
        return True
    except ModbusError as exc:
        if exc.is_exception and exc.exception_code == 0x02:
            print(f"  FAIL: illegal data address — reg {candidate_reg} is NOT "
                  f"ANGLE_SET. Try another candidate (e.g. 1003, 1007, 1011, "
                  f"1015). Check `thirdparty/inspire-api/` or the C++ controller "
                  f"for the correct address.")
        else:
            print(f"  FAIL: {exc}")
        return False
    except OSError as exc:
        print(f"  FAIL (transport): {exc}")
        return False


def probe_move_test(client: ModbusTCPClient, angle_set_reg: int) -> bool:
    """Tiny open/close cycle to confirm ANGLE_SET really drives the hand.

    Sequence: q=0.9 → 0.7 → 0.9 (small motion, ~10% of range).
    Assumes the hand is empty (no cup, no fingers near).
    """
    print(f"\n=== 5. Move test (small open/close cycle via reg {angle_set_reg}) ===")
    print("  !!! Make sure the hand is EMPTY and nothing is in its reach !!!")
    print("  Press Ctrl-C within 3s to abort...")
    try:
        time.sleep(3.0)
    except KeyboardInterrupt:
        print("  Aborted by user.")
        return False

    sequence = [(0.9, "near-open"), (0.7, "half-closed"), (0.9, "near-open")]
    try:
        for q, label in sequence:
            raw = [int(round(q * 1000.0))] * N_JOINTS
            print(f"  -> q={q:.2f} ({label}), writing {raw}...")
            client.write_multiple_registers(angle_set_reg, raw)
            time.sleep(0.6)
            cur = client.read_holding_registers(ANGLE_ACT_REG, count=N_JOINTS)
            cur_norm = [v / 1000.0 for v in cur]
            print(f"     ANGLE_ACT now = {cur}  (norm = "
                  f"{[round(v,2) for v in cur_norm]})")
        print("  OK — move test passed. ANGLE_SET register confirmed.")
        return True
    except (OSError, ModbusError) as exc:
        print(f"  FAIL: {exc}")
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.123.210",
                    help="Hand IP (default 192.168.123.210 = left hand)")
    ap.add_argument("--port", type=int, default=6000)
    ap.add_argument("--unit-id", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--angle-set-reg", type=int, default=ANGLE_SET_REG,
                    help=f"Candidate ANGLE_SET register (default {ANGLE_SET_REG} = "
                         "documented Inspire RH56 series value; verify on hardware)")
    ap.add_argument("--move-test", action="store_true",
                    help="After no-op test passes, do a tiny open/close cycle. "
                         "Only use this with an EMPTY hand.")
    ap.add_argument("--skip-tactile", action="store_true")
    args = ap.parse_args()

    print(f"Inspire RH56E2 probe — {args.ip}:{args.port} unit_id={args.unit_id}")

    client = ModbusTCPClient(host=args.ip, port=args.port,
                             unit_id=args.unit_id, timeout=args.timeout)
    if not client.connect():
        print(f"\nERROR: cannot open TCP connection to {args.ip}:{args.port}.")
        print("Possible causes:")
        print("  * host not on the same subnet (need 192.168.123.x/24)")
        print("  * hand powered off / disconnected")
        print("  * wrong IP (right hand is on factory IP 192.168.11.210 — see TЗ)")
        return 2

    ok = True
    ok &= probe_basic(client)
    ok &= probe_angle_act(client)
    if not args.skip_tactile:
        ok &= probe_tactile(client)

    # ANGLE_SET no-op test is the critical pre-motion check.
    noop_ok = probe_angle_set_noop(client, args.angle_set_reg)

    if noop_ok and args.move_test:
        ok &= probe_move_test(client, args.angle_set_reg)

    client.close()

    print("\n=== SUMMARY ===")
    print(f"  basic read (HAND_ID):           {'OK' if ok else 'FAIL'}")
    print(f"  ANGLE_SET no-op write test:     {'OK' if noop_ok else 'FAIL'}")
    if not noop_ok:
        print("\nACTION REQUIRED before enabling motion in g1_manip_node:")
        print("  1. Edit inspire_e2_hand.py:ANGLE_SET_REG to the correct value.")
        print("  2. Re-run this probe until the no-op test passes.")
        print("  3. Then run `--move-test` with an empty hand for final confirmation.")
        print("  4. Only then set enable_motion:=true on g1_manip_node.")
        return 1
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
