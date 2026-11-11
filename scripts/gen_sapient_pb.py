"""Compile the vendored Dstl SAPIENT schema into the descriptor set Kestrel ships.

    python scripts/gen_sapient_pb.py            # writes src/kestrel/interop/sapient_v2.desc
    python scripts/gen_sapient_pb.py --out FILE

``kestrel.interop.wire`` builds its message classes from this descriptor set at import, so
no generated Python is checked in and any protobuf runtime from 4.x on works. The schema
under ``third_party/sapient-protos`` is used unchanged. Needs ``grpcio-tools``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROTOS = ROOT / "third_party" / "sapient-protos"
DESC = ROOT / "src" / "kestrel" / "interop" / "sapient_v2.desc"
ENTRY = "sapient_msg/bsi_flex_335_v2_0/sapient_message.proto"


def generate(out: Path) -> None:
    import grpc_tools
    from grpc_tools import protoc

    include = str(Path(grpc_tools.__file__).parent / "_proto")
    out.parent.mkdir(parents=True, exist_ok=True)
    args = [
        "protoc",
        f"-I{PROTOS}",
        f"-I{include}",
        "--include_imports",
        f"--descriptor_set_out={out}",
        ENTRY,
    ]
    if protoc.main(args) != 0:
        raise SystemExit("protoc failed")


def main() -> None:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("--out", default=str(DESC))
    args = ap.parse_args()
    generate(Path(args.out))
    print(f"wrote {args.out} ({Path(args.out).stat().st_size} bytes)")


if __name__ == "__main__":
    main()
