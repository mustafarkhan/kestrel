# Interoperability: SAPIENT reports

A counter-UAS picture is built from many sensors made by different companies: radars,
cameras, acoustic arrays, RF detectors. Allied doctrine and the European studies of the
drone threat agree on one point: those sensors are only useful together if they report in
a common format to a common fusion node. For counter-UAS that format is **SAPIENT**,
developed by the UK Defence Science and Technology Laboratory (Dstl), published as BSI
Flex 335, trialled with 17 sensor modules and 7 decision modules at NATO's counter-UAS
Technical Interoperability Exercise, and in NATO standardisation since 2024.

Kestrel can write its output as SAPIENT messages:

```bash
kestrel sim "survey the north field at 25 m and report any drone" \
    --config configs/acoustic.yaml --sapient runs/latest/sapient.jsonl
```

## What is sent

| SAPIENT message (ICD v7) | When | Kestrel content |
|---|---|---|
| `Registration` | once per node | node type, ICD version `7.0`, status interval 1 s, one permanent mode, UTM zone |
| `StatusReport` | every second, and on a mode change | system state, current behaviour as the mode, observer position |
| `DetectionReport` | per confirmed track, at 2 Hz | track position and velocity with one-sigma errors, recent detection ratio as `detection_confidence`, UAV/bird classification when the classifier is on |
| `DetectionReport` (acoustic node) | per triangulated fix | horizontal fix with its error; altitude at mid-band with the band as its error |

Two nodes register: the airborne cameras (`NODE_TYPE_CAMERA`) and, if fitted, the ground
acoustic network (`NODE_TYPE_ACOUSTIC`). Keeping them separate lets the fusion node weigh
each sensor on its own record, which is the point of the standard.

## Mapping decisions

* **Coordinates.** `Location` in `UTM_M` on `WGS84_G`: easting, northing and altitude in
  metres, errors in metres. The ICD declares the UTM zone once, in the registration's
  location type, and Kestrel does the same (`55H` at the default origin). Positions go
  local ENU → geodetic → UTM, so grid convergence is handled. The UTM conversion is
  Krüger's series, tested to a millimetre against PROJ (`tests/test_sapient.py`).
* **Acoustic altitude.** A microphone network measures direction on the ground plane
  only. The ICD assumes an object without `z` is *on the ground*, which for a drone is
  the wrong assumption to hand a fusion node. Kestrel sends the middle of the operating
  altitude band with `z_error` equal to half the band.
* **Identifiers.** `node_id` is a UUID v4 per node, and `report_id` and `object_id` are
  ULIDs, as the ICD specifies. A camera track keeps its `object_id` for its lifetime. The
  acoustic network reports fixes rather than tracks, so each fix is a new object.

## What has not been done

* **Binary encoding.** The ICD's wire format is protobuf over TCP with a 4-byte
  little-endian length prefix. Dstl supplies the `.proto` files on request; they are not
  published. Kestrel writes the same messages as proto3 JSON (snake_case field names,
  enums by name). Converting JSON to binary is mechanical once those files are available.
* **Unverified member names.** Where the ICD text names a `oneof` but not its members, the
  names used are assumptions to check against the `.proto` files:
  * `LocationType.coordinates_oneof` and `datum_oneof`: written as `location_units` and
    `location_datum`;
  * the location and velocity oneofs of `DetectionReport`: written as `location` and
    `enu_velocity`.
* **Taxonomy.** Class names (`UAV`, `Bird`) are placeholders. The ICD's taxonomy appendix
  is where the agreed names live.
* **Conformance.** No run against the SAPIENT test harness or a fusion node.
* **Tasking.** Kestrel accepts no SAPIENT `Task` messages; tasking is by natural language
  through its own planner.

## Sources

* Dstl, *SAPIENT Interface Control Document*, version 7.0 (2023), Open Government Licence.
  https://assets.publishing.service.gov.uk/media/6419a2068fa8f547c68029d3/SAPIENT_Interface_Control_Document_v7_FINAL__fixed2_.pdf
* *SAPIENT becomes NATO standard for counter-drone systems* (June 2026), reporting on the
  NATO ratification process and the NATO TIE trials.
  https://letsdatascience.com/news/sapient-becomes-nato-standard-for-counter-drone-systems-199eb30d
* ECFR, *Buzzkill: how to defend Europe against drones* (2026), recommending common
  standards and certification with NATO for interoperability, and one command-and-control
  node fed by all flank detection stations.
  https://ecfr.eu/publication/buzzkill-how-to-defend-europe-against-drones/
