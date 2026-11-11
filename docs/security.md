# Security: what is protected, how, and what is not

Kestrel sits on two links and reads two kinds of untrusted input. This page says what an
attacker on each can and cannot do, so an integrator's security review starts from facts.
It is a description of the software, not an accreditation.

## The SAPIENT link (sensors ↔ fusion node ↔ airborne node)

The standard defines no transport security, so Kestrel adds it underneath.

| threat | defence | where |
|---|---|---|
| an outsider injects tracks, impersonates a sensor, or tasks the aircraft | mutual TLS 1.3 against a private site CA (`deploy/pki.sh`): both ends present a certificate that CA signed. A non-local listener or link without TLS is refused unless `--insecure` is given | `interop/wire.py::tls_context` |
| an insider node speaks for another node | a `node_id` is bound to the connection that registered it; reports under another node's id, and a second registration of a live id, are refused | `SapientServer` |
| a crafted or broken stream crashes or exhausts the service | a frame longer than 1 MiB is refused before it is allocated; an undecodable payload closes only that connection; connections are capped and queues bounded | `FrameReader`, `SapientServer` |
| non-conformant messages corrupt the picture | every received message is checked for the schema's mandatory fields; reports whose position, errors, velocity or confidence are not finite and plausible, positions outside the site, and times more than 2 s off the fusion clock are dropped and counted by reason; new tracks stop at a limit | `wire.violations`, `rap/fusion.py`, `rap/service.py` |
| recorded reports are replayed later | the same 2 s clock window; TLS prevents replay of a captured session | `rap/service.py` |
| a compromised or confused fusion node sends the aircraft somewhere | a task must name a location inside the fence, finite and readable; anything else is rejected with a `TaskAck` reason. Every resulting setpoint still passes the on-board safety monitor (fence, altitude, speed, separation, energy), which no message can switch off. Traffic can only push the aircraft away from a position | `live.py`, `safety/monitor.py` |
| a silent sensor is taken for an empty sky | per-node health in every picture (`OK`, `STALE`, `DISCONNECTED`) | `rap/service.py` |

Not covered: no certificate revocation (retiring a device means a new CA and reissuing);
certificates identify a device, not a role, so any certified node may send any message
kind its registration allows; private keys are files on disk, protected only by the host.

## The MAVLink link (companion computer ↔ autopilot ↔ radios)

Kestrel does not sign MAVLink. Anyone who can transmit on the telemetry radio's channel
can command the autopilot whatever Kestrel does. Configure **MAVLink 2 signing** on the
autopilot and every radio, and prefer a wired serial link between the companion computer
and the flight controller. Kestrel's own protections on this link are behavioural: it
stops sending the moment the vehicle leaves GUIDED (a pilot always wins), sends nothing
on a stale position, and relies on the autopilot's own fence and failsafes as the backstop,
which `kestrel preflight` reports on.

## Untrusted input

| input | handling |
|---|---|
| the instruction, and the language model's output | the model's output is parsed against a JSON Schema and then validated like any plan: sites must exist, the path must stay in the fence and altitude band, energy must fit. The model never produces setpoints. `stop`, `abort` and `land` bypass it |
| site and config files | parsed with a safe YAML loader into typed, validated models; a site file for somewhere else cannot be flown, because the vehicle's own GNSS position must fall inside its fence before arming |
| camera frames | pixels only; a camera that stops delivering is a failed sensor, reported and acted on, not an empty sky |
| Remote ID broadcasts | **not authenticated, and treated so**: a whitelisted serial is *assumed friend* until an independent sensor agrees with the broadcast position, and a serial heard from two places is never *friend*. A hostile broadcasting our serial from its own true position while ours is on the ground is still shown as friend: see [readiness.md](readiness.md), item E |
| the operator console | `declare` is the only way a track becomes *hostile*; every use is recorded in the picture. The console is authenticated only by access to the terminal it runs in |

## Navigation

Kestrel takes its position from the autopilot. GNSS jamming or spoofing degrades the fence
and every reported position; the autopilot's EKF failsafe is the only protection today.

## Supply chain

Dependencies are few and pinned by minimum version; a weekly CI job audits them for known
vulnerabilities; the container runs as a non-root user; Dstl's schema is vendored unchanged
with its source commit recorded. No generated protobuf code is checked in.

## Reporting a vulnerability

Open a private security advisory on the repository rather than a public issue.
