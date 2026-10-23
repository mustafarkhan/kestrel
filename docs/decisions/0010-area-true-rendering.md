# 0010: Sub-pixel parts were drawn too large

## Problem

The renderer draws an aircraft as a set of discs (body, rotors; for birds, body and wing
sections) on a 4× supersampled grid. A disc much smaller than the grid spacing can fall
between samples and vanish, so every disc had a minimum radius of 0.35 px. That kept small
parts visible, but at full opacity: a rotor that should cover 0.1 px² was drawn covering
0.38 px². At long range, where every part is sub-pixel, the drone became several times
darker in total than its real silhouette, and so detectable further out than it should be.
A second factor then multiplied in a guessed "sub-pixel contrast loss".

It surfaced while adding birds. A distant bird's wing-beat barely changed its brightness,
because the clamped wing discs overlapped whatever their spread, which pointed at the
clamp.

## Fix

A disc below the minimum radius is still drawn at that radius, but with opacity
`(r_true / 0.35)²`, so the area the pixels integrate is the true area. The extra
contrast factor is removed; the optics blur already spreads a small object's darkness
over more pixels, which is the physical reason small objects lose contrast.

## Effect on detection

Probability of detecting a 0.35 m drone in a single frame (40 random skies per range,
same detector and thresholds):

| range | before | after |
|---|---|---|
| 20–45 m | 100% | 100% |
| 60 m | 78% | 53% |
| 80 m | 28% | 0% |
| 100 m | 7% | 0% |

The detector itself did not change. What changed is how far out the simulated target
was honestly visible: about 60 m rather than about 80. Every benchmark in the README was
re-run on the corrected renderer; numbers from before this change are not comparable.
