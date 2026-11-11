# Security policy

## Reporting a vulnerability

Please report it privately through GitHub: **Security → Advisories → Report a
vulnerability** on this repository. Do not open a public issue or pull request for it.
Include what is affected, how to reproduce it, and what an attacker gains.

You will get an acknowledgement, and a fix or a stated reason for none, before anything
is made public. Credit is given unless you ask otherwise.

## What counts

Anything that lets someone who should not be able to: inject, alter or suppress tracks
in the air picture; impersonate a sensor or the fusion node; task or redirect the
aircraft; weaken or bypass the on-board safety monitor; make a track *hostile* or
*friend* without the conditions the documentation states; crash or stall a service with
crafted input.

## What is already known

The threat model, the defences and the gaps that are known and documented (Remote ID is
not authenticated, the MAVLink link is not signed by Kestrel, no certificate revocation,
reliance on GNSS) are in [docs/security.md](docs/security.md). Reports that go beyond those are especially
welcome.

## Supported versions

Only the latest release receives fixes.
