# Source

Copied unchanged from Dstl's public repository https://github.com/dstl/SAPIENT-Proto-Files at
commit `244a741c13bcee628f84944aef91e4a66854bd7d` (4 June 2024): `proto_options.proto` and the
`bsi_flex_335_v1_0` and `bsi_flex_335_v2_0` folders, arranged under `sapient_msg/` to match
their import paths. Crown copyright, licensed under the Apache License 2.0 (`LICENCE.txt`).

Kestrel uses them only in its test suite, to check that every SAPIENT message it writes parses
against the published schema (`tests/test_sapient_conformance.py`).
