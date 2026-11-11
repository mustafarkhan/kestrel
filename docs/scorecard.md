# The threat library and the scorecard

Every counter-drone vendor publishes its own numbers on its own situations. This part of
Kestrel is a common yardstick: a library of named threat situations anyone can replay, one
set of measures, and a published reference. A sensor maker, a fusion engine or a test range
can put a system through the same sky, as often as they like, and read the result the same
way everyone else does.

It scores the **air picture** a system produces: what is tracked, what it is called, who it
is said to be. It does not score effectors, and nothing in it attacks a system's software
or links: the library is situations to detect.

```bash
kestrel threats                                   # the library
kestrel scorecard --seeds 5                       # Kestrel's own fusion node on all of it: the reference
kestrel threats --export silent-low --seed 9000   # one situation as files, for a system under test
kestrel scorecard --picture their_picture.jsonl --truth truth.jsonl --site site.json \
    --system "System X" --reference docs/results/scorecard/scorecard.json
```

## Threat library 1.0

A threat is a function of a seed: the same seed is the same situation, another seed is
another draw of it. All share the benchmark site (a 1.6 km square with a protected depot
and substation), its sensors (a radar, four RF direction finders, a Remote ID receiver, an
acoustic grid), two friendly patrol drones broadcasting whitelisted serials, a compliant
civil drone and birds. Definitions are in `src/kestrel/rap/threats.py` and never change
within a library version; results quote the version.

| name | the situation | why it is here |
|---|---|---|
| `mixed-raid` | a radio-controlled quadcopter, an engine-driven fixed-wing and a silent low multirotor at once | each defeats a different sensor; the everyday case |
| `silent-low` | four quiet drones with no radio link, 8 to 20 m up, from all sides | fibre-optic and autonomous drones emit nothing; low flight hides them from radar |
| `one-way-attack` | three fast fixed-wing drones straight at the depot | warning time is everything |
| `saturation` | twelve drones from one sector within a minute | track capacity, duplicates, track continuity |
| `decoys` | eight radar-bright decoys on one side, two silent low drones from the other | attention drawn one way while the attack comes from another |
| `remote-id-spoof` | a hostile broadcasts a friendly serial with a false position | Remote ID is unauthenticated |
| `remote-id-impersonation` | a hostile broadcasts a grounded friend's serial from its own true position | the case broadcast identification cannot solve |
| `birds-only` | forty birds, no hostile drone | every drone alarm is a false alarm |
| `radar-outage` | the mixed raid with the radar silent for 200 s | a sensor down, jammed or cut off |

Sources for each are in [research.md](research.md), section 6.

## The measures

From `src/kestrel/rap/score.py`, every 0.5 s against ground truth inside the site; a track
matches an object within 25 m horizontally.

| measure | better | meaning |
|---|---|---|
| hostile drone-time tracked | higher | share of the time hostile drones were inside the site that a track was on them |
| entry to first track | lower | median time from a drone entering the site to its first track |
| drones never tracked | lower | drones that crossed the site with no track at all (total over the runs) |
| tracked drone-time shown as UAV / as bird | higher / lower | what the picture called the drones it tracked; *bird* is the dangerous error |
| first track to UAV | lower | for hostile drones, median time from first track to being called a drone |
| bird-time shown as UAV | lower | share of all bird-time shown as a drone: the false alarm an operator sees |
| false tracks | lower | mean number of tracks on nothing |
| duplicate tracks per object | lower | extra tracks on an object that already has one |
| track identities per drone | lower | 1 is an unbroken track |
| friend-time shown as friend | higher | share of friendly drone-time identified as friend |
| hostile shown as friend | lower | seconds a hostile drone's track carried a friendly identity (total) |
| friend shown where none is | lower | seconds a friendly identity sat where no friendly drone was (total; a hostile shown as friend counts here too) |
| GOSPA | lower | one number for localisation, misses and false tracks (cut-off 25 m, p = 2, α = 2) |

There is **no pass mark**. A threshold would be this project's opinion of what is good
enough, and that depends on the site and the mission. The scorecard prints the measures
next to a reference so two parties read the same table.

## Testing another system

1. **Export a situation.** `kestrel threats --export NAME --seed N` writes three files:
   * `sensors.jsonl`: what the simulated sensors reported, as SAPIENT (BSI Flex 335 v2.0)
     messages in JSON layout, one per line, time-ordered. `kestrel send` streams the same
     file over TCP in SAPIENT's binary framing to a live fusion node.
   * `truth.jsonl`: one line per 0.5 s, `{"t", "objects": [{"key", "kind": "uav"|"bird",
     "role": "hostile"|"friendly"|"civil"|"bird", "lat", "lon", "alt_m", "enu_m"}]}`.
     **Not shown to the system under test.**
   * `site.json`: origin, fence, protected areas, friendly serials, duration.
2. **Run the system under test** on the sensor traffic, and write its picture as JSON
   lines, one picture per line:

   ```json
   {"t": 12.5, "tracks": [{"id": 7, "lat": 60.3012, "lon": 25.0478, "alt_m": 61.0,
                           "class": "UAV", "identity": "SUSPECT"}]}
   ```

   `t` is seconds from the start of the situation. `class` is `UAV`, `Bird` or `Unknown`;
   `identity` is one of `FRIEND`, `ASSUMED_FRIEND`, `NEUTRAL`, `SUSPECT`, `HOSTILE`,
   `UNKNOWN`, `PENDING`. Positions are latitude, longitude (WGS84) and altitude above the
   ellipsoid, or `enu_m` metres east, north, up from the site origin. `kestrel fuse` and
   `kestrel fusion --out` already write this format.
3. **Score it.** `kestrel scorecard --picture ... --truth ... --site ...`. At each truth
   step the newest picture not later than the step is used; a picture more than 2 s old
   counts as no picture, so a system that stalls is scored as blind while it stalls.

A system that brings its **own sensors** rather than a fusion engine is tested the same way
on a range: the truth file comes from the target drones' GNSS logs in the same format, and
the scoring is unchanged.

## The reference

[results/scorecard/scorecard.md](results/scorecard/scorecard.md) is Kestrel's own fusion
node on the library, five situations per threat (seeds 9000 to 9004), with every run in
`scorecard.json`. Read it as a reference, not as a target, and note what it is: **simulated
sensors with typical published parameters and a simulated sky**. A real sensor's numbers
will differ, which is the point of running it.

What it shows about Kestrel itself, weaknesses first:

* **`remote-id-impersonation`: the hostile is shown as a friend for most of its
  flight** (380 to 540 s of a 600 s situation). A radar confirms the broadcast position because the position is true. Kestrel's
  corroboration rule and clone rule do not help when the real owner of the serial is on
  the ground. Only authenticated friendly-force identification closes this; it is stated in
  the README's limitations and measured here so it cannot be forgotten.
* **`silent-low`: about a third of hostile drone-time is untracked** and the first track
  comes about 46 s after entry, against under a second for drones that radiate or fly high.
  This is the gap an airborne node is sent into.
* **`remote-id-spoof`: friends are shown as friend only 57% of the time**, because a serial
  heard from two places is never shown as friend, the genuine copy included. That is the
  price of never showing the spoofer as one.
* **`radar-outage`** costs about seven points of hostile coverage; the RF, acoustic and
  Remote ID nodes hold what radiates or is loud, and nothing holds the silent low drone.
* `one-way-attack` and `saturation` are handled: hostile coverage of 100%, duplicates at
  0.1%, about 1.5 track identities per drone under saturation.
* `decoys`: 77% hostile coverage with no drone left wholly untracked. The measure does not
  separate the decoys from the two silent low drones behind them, so it cannot yet say which
  of them the missing time belongs to; a per-drone breakdown is the next addition.

## Limits

* The sensors are models. A threat that defeats a model sensor may not defeat a real one,
  and the reverse.
* Scoring is horizontal (25 m), because acoustic and RF fixes carry no altitude. A system
  with good altitude gets no credit for it yet.
* The library is nine situations, not the threat. Version 1.0 has no electronic attack on
  the sensors themselves, no night, no weather, and no terrain.
* Seeds 9000 upward are published and are used to develop Kestrel from now on; a system
  tuned to them will look better than it is. Use other seeds for acceptance.
