# Interoperability: SAPIENT

A counter-UAS picture is built from many sensors made by different companies: radars,
cameras, acoustic arrays, RF detectors. Allied doctrine and the European studies of the
drone threat agree on one point: those sensors are only useful together if they report in
a common format to a common fusion node. For counter-UAS that format is **SAPIENT**,
developed by the UK Defence Science and Technology Laboratory (Dstl), published as BSI
Flex 335, trialled with 17 sensor modules and 7 decision modules at NATO's counter-UAS
Technical Interoperability Exercise, and adopted by NATO through STANREC/AEDP-4869.

Everything in Kestrel that talks to anything else speaks it: Kestrel's own camera and
acoustic nodes, the simulated ground sensors, and the fusion node that builds the air
picture and tasks Kestrel ([air-picture.md](air-picture.md)).

```bash
kestrel sim "survey the north field at 25 m and report any drone" \
    --config configs/acoustic.yaml --sapient runs/latest/sapient.jsonl
kestrel fuse runs/latest/sapient.jsonl --out runs/latest/rap.jsonl
```

## Version and conformance

Kestrel writes **BSI Flex 335 v2.0** messages, the current version. Field names, oneof
members and enumerations come from Dstl's published protobuf schema
(https://github.com/dstl/SAPIENT-Proto-Files, Apache 2.0, vendored unchanged under
`third_party/sapient-protos`). `tests/test_sapient_conformance.py` compiles that schema and
parses **every kind of message Kestrel writes** against it (registration, status report,
detection report, task, task acknowledgement) with `google.protobuf.json_format`, which
rejects unknown fields, wrong enum names and wrong types. Passing means the JSON converts to
the binary wire format as is.

Status, detection and acknowledgement messages also parse against **v1.0** (the ICD v7
generation). Three things changed between the versions and are written the v2.0 way:
`detection_definition` became a list and `task` a single definition in a registration's mode;
a task acknowledgement's `reason` became a list; and a detection report gained an object `id`
(Kestrel puts a Remote ID serial there, and in `track_info` for v1.0 readers).

Writing this check corrected four things earlier versions had assumed from the ICD text:
the registration's `node_definition` (not `node_type`), a status report's `node_location`
(not `sensor_location`), the `look_at.location_list` wrapper in a task, and no `name` on a
task command definition.

## What is sent

| SAPIENT message | From | When | Content |
|---|---|---|---|
| `Registration` | every node | once | node type (`NODE_TYPE_CAMERA`, `_ACOUSTIC`, `_RADAR`, `_PASSIVE_RF`), standard version, status interval, one permanent mode, UTM zone; Kestrel's camera node also declares a `LOOK_AT` task definition |
| `StatusReport` | every node | every second (Kestrel), every 5 s (ground sensors), and on a mode change | system state, mode, node location |
| `DetectionReport` | every node | per track at 2 Hz (Kestrel), per scan or fix (ground sensors) | position and velocity with one-sigma errors, `detection_confidence`, UAV/bird classification, Remote ID serial (`id`), RF protocol (`track_info`) |
| `Task` | fusion node | when the picture needs a look | `LOOK_AT` with the track's position and errors, `destination_id` set to Kestrel's camera node |
| `TaskAck` | Kestrel | on every task | accepted, or rejected with a reason when the mission has been aborted |

Kestrel registers two nodes, its cameras and (if fitted) the ground acoustic network,
so a fusion node weighs each on its own record.

## On the wire

`src/kestrel/interop/wire.py` turns the JSON-layout messages into what a SAPIENT link
carries: protobuf binary built from Dstl's schema, each message preceded by its length as a
4-byte little-endian integer, over TCP. A validator reads the schema's `is_mandatory` options,
which protobuf itself does not enforce; everything Kestrel sends is checked against them in
the tests and everything the fusion service receives is checked on arrival. The standard
defines no transport security, so both endpoints run mutual TLS 1.3 against a private site
CA ([security.md](security.md)). `kestrel fusion` is the fusion-node end, `kestrel fly
--sapient` and `kestrel send` the node end.

## Tasking

A task that asks Kestrel to look at a list of locations sends the observer to investigate
their centre. When the locations carry a measured altitude (`z_error` under 15 m, as a
radar or fused track does), Kestrel closes to 30 m to classify; when they are a vertical
column (an acoustic or RF fix), it keeps a 45 m standoff. A re-sent position for the look
under way updates it, so a fusion node can follow a moving aircraft. The `follow`,
`move_to` and `patrol` commands of v2.0 are not accepted yet.

## A jammed link

With `link_loss_action: continue`, reports made while the link is down are held, with
their measurement timestamps, and sent in order when it returns. Only the latest status
report per node is kept, since a stale status is no use late. Tasks do not arrive while the
link is down.

## Mapping decisions

* **Coordinates.** `Location` in `UTM_M` on `WGS84_G`: easting, northing and altitude in
  metres, errors in metres. The zone is declared once, in the registration's location type.
  Positions go local ENU → geodetic → UTM, so grid convergence is handled. The UTM
  conversion is Krüger's series, tested to a millimetre against PROJ (`tests/test_sapient.py`).
* **No altitude.** An acoustic or RF fix knows direction on the ground plane only. The
  standard assumes an object without `z` is *on the ground*, the wrong assumption to hand a
  fusion node about a drone. Kestrel sends the middle of the altitude band with `z_error`
  equal to half the band.
* **Identifiers.** `node_id` is a UUID v4 per node, and `report_id`, `object_id` and
  `task_id` are ULIDs. A tracking sensor keeps an `object_id` for its track's lifetime; the
  acoustic network reports fixes, so each is a new object.
* **Reading.** The fusion node also reads v1.0-style registrations (`node_type`) and Remote
  ID serials in `track_info`, and accepts locations in `LAT_LNG_DEG_M` (x as longitude).

## What has not been done

* **Taxonomy.** Class names (`UAV`, `Bird`) are placeholders. v2.0 defines taxonomies at run
  time; aligning with an agreed one is open.
* **Conformance beyond the schema.** No run against the SAPIENT test harness, the Apex
  middleware, or a third-party fusion node.
* **Hierarchy.** v2.0's hierarchical fusion (`NODE_TYPE_FUSION_NODE`, `dependent_nodes`) is
  not used; Kestrel's fusion node is a single level.

## Sources

* Dstl, *SAPIENT Proto Files* (BSI Flex 335 v1.0 and v2.0). https://github.com/dstl/SAPIENT-Proto-Files
* Dstl, *SAPIENT Interface Control Document*, version 7.0 (2023), Open Government Licence.
  https://assets.publishing.service.gov.uk/media/6419a2068fa8f547c68029d3/SAPIENT_Interface_Control_Document_v7_FINAL__fixed2_.pdf
* GOV.UK, *New milestone as BSI publishes updated SAPIENT standard* (BSI Flex 335 v2.0).
  https://www.gov.uk/government/news/new-milestone-as-bsi-publishes-updated-sapient-standard
* Defence Online, *Cambridge Pixel validates SAPIENT-compliant radar processing at NATO
  counter-UAS exercise in Romania* (August 2026), on NATO adoption through STANREC/AEDP-4869.
  https://www.defenceonline.co.uk/2026/08/07/cambridge-pixel-validates-sapient-compliant-radar-processing-at-nato-counter-uas-exercise-in-romania/
* ECFR, *Buzzkill: how to defend Europe against drones* (2026).
  https://ecfr.eu/publication/buzzkill-how-to-defend-europe-against-drones/
