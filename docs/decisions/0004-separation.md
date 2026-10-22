# 0004. Separation: what the buffer around an aircraft must contain

The minimum separation is 12 m. Each term below was added because a simulated
encounter broke that limit without it.

| Problem found | Closest approach before | Fix | After |
|---|---|---|---|
| Contact interrupt commanded 13.8 m/s *towards* the target (limit 8): a far setpoint became speed through the position loop | 5.0 m | reference filter on the standoff goal; cascaded, speed-saturated position loop (0006) | 12.7 m |
| Filter claimed 1.1 m uncertainty while 5-6 m off: size-based range errors are correlated, not independent | | inflated range noise; uncertainty floored at 15% of range in the monitor | |
| Both aircraft closing at about 8 m/s: a static buffer leaves under a second | 8.4 m | stopping distance at the current closing speed plus reaction time | 13.2 m |
| Target crossed behind the observer during return-home: no track, nothing to avoid | 1.8 m | contact watch: within 70 m of the last known contact, point camera and heading at it; keep lost tracks 5 s with a reachable-set keep-out | 13.7 m |

Single runs were then clean, but the first 24-run Monte Carlo benchmark had 9
runs with some time inside 12 m (worst 5.1 m). Diagnosing them:

| Problem found | Fix |
|---|---|
| inside the buffer, a position nudge cannot outrun an intruder flying at the observer | monitor commands an escape velocity: intruder's speed towards us + 1 m/s + a term growing with depth |
| intruder above: line-of-sight escape was mostly downward, capped at 2.5 m/s vertical | escape horizontally (about 3x the speed authority) |
| track lost with the intruder nearly overhead, observer kept hovering | on losing a close contact, back away from its last position before scanning |
| constant-velocity filter lagged manoeuvring targets by about 5 m at 10 m range | process noise 2.5 -> 4 m/s^2 |
| test intruders reversed velocity instantly at waypoints (infinite acceleration) | waypoint intruders now round corners within 3 m/s^2, like real multirotors |

After these fixes the same benchmark gives 7 of 24 runs briefly inside 12 m
(at most 3.2 s), worst 7.8 m. That residual is published in the README, not tuned
away.

**Consequences.** Separation is guaranteed only against aircraft the sensor has
seen. The Monte Carlo benchmark reports any residual violations rather than
hiding them.
