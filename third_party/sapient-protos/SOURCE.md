# Source

Copied unchanged from Dstl's public repository https://github.com/dstl/SAPIENT-Proto-Files at
commit `244a741c13bcee628f84944aef91e4a66854bd7d` (4 June 2024): `proto_options.proto` and the
`bsi_flex_335_v1_0` and `bsi_flex_335_v2_0` folders, arranged under `sapient_msg/` to match
their import paths. Crown copyright, licensed under the Apache License 2.0 (`LICENCE.txt`).

Kestrel uses them in two ways: `scripts/gen_sapient_pb.py` compiles the v2.0 files into the
descriptor set `src/kestrel/interop/sapient_v2.desc`, from which the wire encoding and the
mandatory-field validation are built at import (`interop/wire.py`); and the test suite parses
every SAPIENT message Kestrel writes against them (`tests/test_sapient_conformance.py`).
