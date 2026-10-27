#!/usr/bin/env python3
"""Cross-check the C++ DBC codec and E2E protection against independent implementations.

1. Encoding: random signal values -> bytes, by cantools and by `haul-can vectors`. Must match
   byte for byte (this is what catches Motorola bit-numbering mistakes).
2. Decoding: random payload bytes -> signal values, by both. Must agree on every signal.
3. Optionally a candump log recorded by `haul-sim --candump`: every frame must decode with
   cantools, and every E2E CRC must verify with the CRC-8/SAE-J1850 written out below (a
   plain bitwise implementation, independent of the C++ table-driven one).

    python tools/crosscheck_dbc.py --haul-can build/release/haul-can --n 2000 --candump run.log

Exits non-zero on the first mismatch.
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
from pathlib import Path

import cantools

ROOT = Path(__file__).resolve().parent.parent


def crc8_sae_j1850(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def e2e_crc_ok(msg: cantools.database.Message, payload: bytes) -> bool | None:
    """AUTOSAR E2E Profile 1 style: CRC over the data ID (CAN ID, low byte first) + non-CRC bytes."""
    crc_sig = next((s for s in msg.signals if s.name == f"{msg.name}_CRC"), None)
    if crc_sig is None:
        return None
    if crc_sig.byte_order == "little_endian":
        crc_bytes = {(crc_sig.start + k) // 8 for k in range(crc_sig.length)}
    else:  # 8-bit big-endian signal starting on a byte boundary
        crc_bytes = {crc_sig.start // 8}
    data_id = msg.frame_id
    buf = bytes([data_id & 0xFF, (data_id >> 8) & 0xFF]) + bytes(b for i, b in enumerate(payload) if i not in crc_bytes)
    stored = msg.decode(payload, decode_choices=False, scaling=False)[crc_sig.name]
    return crc8_sae_j1850(buf) == stored


def random_physical(sig: cantools.database.Signal, rng: random.Random) -> float:
    if sig.is_signed:
        raw = rng.randint(-(1 << (sig.length - 1)), (1 << (sig.length - 1)) - 1)
    else:
        raw = rng.randint(0, (1 << sig.length) - 1)
    return raw * sig.scale + sig.offset


class HaulCan:
    def __init__(self, exe: Path, dbc: Path):
        self.proc = subprocess.Popen(
            [str(exe), "--dbc", str(dbc), "vectors"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def ask(self, request: dict) -> dict:
        assert self.proc.stdin and self.proc.stdout
        self.proc.stdin.write(json.dumps(request) + "\n")
        self.proc.stdin.flush()
        return json.loads(self.proc.stdout.readline())

    def close(self) -> None:
        if self.proc.stdin:
            self.proc.stdin.close()
        self.proc.wait(timeout=10)


def close_enough(a: float, b: float, scale: float) -> bool:
    return abs(a - b) <= max(1e-9, abs(scale) * 1e-6)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--haul-can", type=Path, default=ROOT / "build" / "release" / "haul-can")
    ap.add_argument("--dbc", type=Path, default=ROOT / "dbc" / "haul_truck.dbc")
    ap.add_argument("--n", type=int, default=1000, help="random vectors per message and direction")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--candump", type=Path, help="also verify every frame in this candump log")
    args = ap.parse_args()

    db = cantools.database.load_file(str(args.dbc))
    rng = random.Random(args.seed)
    cpp = HaulCan(args.haul_can, args.dbc)
    checked = 0
    try:
        for msg in db.messages:
            for _ in range(args.n):
                # Encode direction.
                values = {s.name: random_physical(s, rng) for s in msg.signals}
                expected = msg.encode(values, scaling=True, strict=False).hex()
                got = cpp.ask({"message": msg.name, "signals": values})
                if got["hex"] != expected:
                    print(f"ENCODE MISMATCH {msg.name} {values}\n  cantools {expected}\n  haul-can {got['hex']}")
                    return 1
                # Decode direction, from arbitrary bytes.
                payload = bytes(rng.getrandbits(8) for _ in range(msg.length))
                want = msg.decode(payload, decode_choices=False)
                got = cpp.ask({"message": msg.name, "hex": payload.hex()})
                for sig in msg.signals:
                    if not close_enough(got["decoded"][sig.name], want[sig.name], sig.scale):
                        print(f"DECODE MISMATCH {msg.name}.{sig.name} bytes {payload.hex()}: "
                              f"cantools {want[sig.name]} haul-can {got['decoded'][sig.name]}")
                        return 1
                if got["crc_ok"] is not None and got["crc_ok"] != e2e_crc_ok(msg, payload):
                    print(f"CRC VERDICT MISMATCH {msg.name} bytes {payload.hex()}")
                    return 1
                checked += 2
        print(f"codec: {checked} vectors over {len(db.messages)} messages agree with cantools")

        if args.candump:
            frames = crc_ok = 0
            for line in args.candump.read_text().splitlines():
                if not line.strip():
                    continue
                _, _, frame = line.split()
                can_id, data = frame.split("#")
                msg = db.get_message_by_frame_id(int(can_id, 16))
                payload = bytes.fromhex(data)
                msg.decode(payload)  # raises on anything cantools can't decode
                ok = e2e_crc_ok(msg, payload)
                if ok is False:
                    print(f"E2E CRC FAILED in log: {line}")
                    return 1
                frames += 1
                crc_ok += ok is True
            print(f"candump: {frames} frames decoded by cantools, {crc_ok} E2E CRCs verified independently")
    finally:
        cpp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
