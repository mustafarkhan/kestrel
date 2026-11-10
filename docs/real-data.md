# Real-data evaluation

Every detection number elsewhere in this repository is measured on rendered frames. The
renderer is area-true and noisy, so misses and false alarms are emergent rather than
scripted, but it is still a model of the sky. `kestrel evaluate` runs the same detector on
recorded footage with ground-truth boxes, so the simulated numbers can be checked against
real cameras.

```bash
# a visible-band set in YOLO layout (images/ and labels/)
kestrel evaluate path/to/dataset --format yolo --out runs/eval_visible.json

# thermal footage in the Anti-UAV layout (a folder per sequence, JSON labels), white hot
kestrel evaluate path/to/anti-uav/test --format anti-uav --polarity bright --every 5

# MOT layout (frames plus gt.txt), with the detector settings of a config file
kestrel evaluate path/to/sequence --format mot --config configs/thermal.yaml
```

## What is measured

* **Recall**: the share of ground-truth boxes with a detection whose centre lies inside the
  box grown by 3 pixels.
* **Recall by target size**: by box diagonal, in the bands the CST Anti-UAV benchmark uses
  (tiny up to 10 px, small up to 30, normal up to 50, large above). Kestrel's detector is
  built for the tiny and small bands, where a drone is a few pixels across and learned
  detectors are weakest.
* **False detections per frame and precision**: detections that match no box.
* **Per sequence**: recall and false detections, to find the sequences that fail.

## Layouts read

| layout | what it looks like | used by |
|---|---|---|
| `anti-uav` | a folder per sequence holding the frames and a JSON file with `exist` (0/1 per frame) and `gt_rect` (`[x, y, w, h]` per frame) | the Anti-UAV challenges and Anti-UAV410 (thermal) |
| `yolo` | `images/` and `labels/`, one text file per image with normalised `class cx cy w h` lines | most public drone sets exported for YOLO, e.g. Det-Fly or Drone-vs-Bird conversions |
| `mot` | a folder of frames and `gt.txt` with `frame,id,x,y,w,h,...` lines, frames from 1 | MOT-style tracking sets |

The readers are tested on rendered frames written in each layout (`tests/test_evaluate.py`),
so the harness is checked end to end; its numbers on real footage are not yet published.

## What is different from flight

Recorded footage carries no camera attitude, so the horizon mask cannot be applied and the
whole frame is searched. Treelines, buildings and ground clutter that the mask removes in
flight are counted as false detections here. Recall is unaffected; false detections are an
upper bound on what the same detector raises in the air with a gimbal attitude. Size
estimates, range and the bird/drone classifier (which needs a track over time) are not
scored.

## Sets to run

Not redistributed here; each has its own licence and download.

* **Anti-UAV410** and the CVPR Anti-UAV challenge sets: thermal infrared, tracking sequences
  with tiny targets (`--format anti-uav --polarity bright`).
* **Det-Fly**: air-to-air visible frames of a drone seen from another drone, the closest
  public match to Kestrel's camera.
* **Drone-vs-Bird** (WOSDETC challenge): visible footage with birds, the false-alarm case.
* **NATO counter-UAS data challenge 2026**: multi-sensor data including EO/IR; its sensor
  reports go to `kestrel fuse` ([air-picture.md](air-picture.md)), its imagery here.

Results on real footage will be added to this page, with the data set version and the exact
command, when they have been run.
