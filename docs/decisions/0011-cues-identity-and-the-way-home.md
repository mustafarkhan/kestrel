# 0011: Acoustic cues, aircraft identity and the way home

Three faults found by the benchmark matrix, all in how the mission executive treats
contacts rather than in sensing or control.

## 1. Giving way to aircraft the camera already had

**Symptom.** With the ground acoustic network added, the worst-case separation improved
(closest approach 10.6 m against 1.8 m for the camera alone, on the same 24 encounters)
but only 17 of 24 missions completed: 7 returned home early because they had spent their
two-minute traffic budget giving way, with 12–28 yields per run.

**Cause.** An acoustic fix has no altitude, so it is handed to the executive and the
safety monitor as a vertical column at the observer's height. That is the right
conservative reading for an aircraft the camera has *not* seen. But the network keeps
hearing the aircraft the camera is already tracking, and its column sat over the survey
area even when the visual track showed the intruder 10–15 m higher.

**Fix.** A fix that a fresh visual track explains (within the combined horizontal
uncertainty, chi-square gate at p = 0.99) is dropped. The track carries altitude and
velocity, which sound cannot. Cues now matter for what they are for: aircraft the camera
does not have.

## 2. The same aircraft followed again and again

**Symptom.** One run hit the 480 s time limit without landing. The executive had followed
the intruder for 45 s three times, as tracks 1, 4 and 9.

**Cause.** "If you see a drone, follow it" was implemented per track. A track is lost
when the aircraft leaves the field of view or crosses a cloud edge, and comes back with a
new id.

**Fix.** The executive remembers where each handled aircraft was last seen. A new track
appearing within 30 s of that, inside the distance the aircraft could have flown since
(15 m plus 8 m/s times the gap), is taken to be the same aircraft. Two aircraft close
together can be confused; the cost is not following the second one. Separation does not
depend on identity.

## 3. Stuck on the way home

**Symptom.** Another run returned home for traffic and was still 34 m from home at the time
limit. On the way back it yielded 30 times, a few seconds each, to an intruder orbiting
near the return path.

**Cause.** The traffic budget counted time spent giving way and triggered the return home.
But giving way continued on the return path, with no limit of its own.

**Fix.** Giving way on the return path is capped at 30 s in total. After that the
aircraft flies home and the safety monitor alone keeps separation: it pushes away from,
and escapes, any contact inside the buffer whatever the executive wants. Once the overall
traffic budget is spent, no further yields start at all.

## Result

On the published set (seeds 4000–4023), camera plus acoustic network: no run inside
12 m (closest 12.1 m), 20 of 24 missions completed and 4 returned home early for
traffic, all landed at home. With the camera alone on the same encounters: 1 run inside
12 m (7.7 m), 22 completed, 2 returned early. Sound buys separation and costs some
mission time, because an unconfirmed cue has no altitude.
