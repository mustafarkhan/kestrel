"""Command-line interface.

kestrel plan  "<instruction>"            parse + validate, print the plan
kestrel sim   "<instruction>" [--gif]    closed-loop simulation with report
kestrel bench --runs 20                  Monte Carlo benchmark
kestrel fly   "<instruction>" --connect tcp:127.0.0.1:5760   ArduPilot over MAVLink
kestrel preflight "<instruction>" --connect ...   the pre-flight checks only
kestrel fusion --site site.yaml --listen 0.0.0.0:5020   the fusion node as a live service
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import ssl
import sys
import threading
import time
from pathlib import Path
from typing import Any

from kestrel.config import Config, Scenario, load_config, load_scenario
from kestrel.mission.llm import ChatBackend, OllamaBackend

DEFAULT_SCENARIO = Path(__file__).resolve().parents[2] / "scenarios" / "airfield.yaml"


def _load(args: argparse.Namespace) -> tuple[Config, Scenario]:
    cfg = load_config(args.config)
    if not args.scenario and not DEFAULT_SCENARIO.exists():
        return cfg, Scenario()  # installed without the repository: the built-in empty site
    scen_path = Path(args.scenario) if args.scenario else DEFAULT_SCENARIO
    if not scen_path.exists():
        sys.exit(f"scenario not found: {scen_path} (pass --scenario)")
    return cfg, load_scenario(scen_path)


def _backend(args: argparse.Namespace) -> ChatBackend | None:
    return OllamaBackend(model=args.model, host=args.ollama) if args.llm else None


def cmd_plan(args: argparse.Namespace) -> int:
    from kestrel.mission.parser import parse_emergency
    from kestrel.runtime import plan_and_validate

    cfg, sc = _load(args)
    emergency = parse_emergency(args.instruction)
    if emergency is not None:
        print(f"emergency command -> {emergency.value} (bypasses planning)")
        return 0
    plan, report = plan_and_validate(args.instruction, cfg, sc, _backend(args))
    print(f"planner: {plan.source}")
    for n in plan.notes:
        print(f"  note: {n}")
    if report is None:
        print("could not build a plan")
        return 2
    print(report.summary())
    if args.json:
        print(report.mission.model_dump_json(indent=2))
    return 0 if report.ok else 1


def _parse_commands(items: list[str]) -> list[tuple[float, str]]:
    out = []
    for item in items:
        t, _, text = item.partition(":")
        out.append((float(t), text))
    return out


def cmd_sim(args: argparse.Namespace) -> int:
    from kestrel.metrics import compute_metrics
    from kestrel.report import write_report
    from kestrel.runtime import run

    cfg, sc = _load(args)
    if args.seed is not None:
        sc = sc.model_copy(update={"seed": args.seed})
    record = 3 if args.gif else 0
    sapient = None
    if args.sapient or args.picture:
        from kestrel.interop.sapient import SapientExporter, SapientOptions

        sapient = SapientExporter(
            SapientOptions(
                origin_lla=sc.world.origin_lla,
                altitude_band_m=(0.0, cfg.safety.max_altitude),
                seed=sc.seed,
            )
        )
    air_picture = None
    if args.picture:
        from kestrel.rap.loop import AirPicture

        if sc.sensors is None:
            print("the scenario has no ground sensors (see scenarios/airfield_picture.yaml)")
            return 2
        assert sapient is not None
        air_picture = AirPicture(sc, sapient, altitude_band_m=(0.0, cfg.safety.max_altitude))
    print(f"simulating '{sc.name}' (seed {sc.seed}) ...", flush=True)
    res = run(
        args.instruction,
        cfg,
        sc,
        _backend(args),
        _parse_commands(args.command),
        record_every=record,
        sapient=sapient,
        air_picture=air_picture,
    )
    if air_picture is not None:
        print("recognised air picture:", json.dumps(air_picture.summary()))
    if sapient is not None and args.sapient:
        sapient.write_jsonl(args.sapient)
        print(f"{len(sapient.messages)} SAPIENT messages written to {args.sapient}")
    out = Path(args.out)
    m = compute_metrics(res)
    write_report(res, m, out, gif=args.gif)
    if res.report is not None:
        print(res.report.summary())
    for e in m.get("events", []):
        print(f"  {e}")
    keys = ["completed", "landed", "duration_s", "energy_wh", "min_separation_m", "track_error_median_m",
            "target_in_fov_fraction", "false_tracks"]  # fmt: skip
    print(json.dumps({k: m.get(k) for k in keys}, indent=2))
    print(f"report written to {out}/")
    return 0 if m.get("accepted") else 1


def cmd_bench(args: argparse.Namespace) -> int:
    from kestrel.bench import run_benchmark

    cfg, sc = _load(args)
    summary = run_benchmark(
        cfg,
        sc,
        args.runs,
        Path(args.out),
        jobs=args.jobs,
        seed0=args.seed0,
        avoidance=not args.no_avoidance,
        intruders=args.intruders,
        first=args.first,
        mission=args.mission,
        jam_s=args.jam,
        observers=args.observers,
    )
    print(json.dumps(summary, indent=2))
    return 0


def cmd_picture(args: argparse.Namespace) -> int:
    from kestrel.rap.bench import CONFIGS, run_loop_benchmark, run_picture_benchmark

    if args.loop:
        cfg = load_config(args.config)
        base = load_scenario(args.scenario or "scenarios/airfield_picture.yaml")
        modes = {"both": (False, True), "on": (True,), "off": (False,)}[args.tasking]
        seed0 = 8000 if args.seed0 is None else args.seed0
        out = Path(args.out or "runs/loop")
        summary = run_loop_benchmark(cfg, base, args.runs, seed0, out, args.jobs, modes, args.first)
        print(json.dumps(summary, indent=2))
        return 0
    names = args.only.split(",") if args.only else list(CONFIGS)
    seed0 = 7000 if args.seed0 is None else args.seed0
    summary = run_picture_benchmark(
        args.runs, seed0, Path(args.out or "runs/picture"), names, workers=args.jobs
    )
    print(json.dumps(summary, indent=2))
    return 0


def cmd_fuse(args: argparse.Namespace) -> int:
    from kestrel.rap.replay import fuse_file

    whitelist = [s for s in args.whitelist.split(",") if s] if args.whitelist else []
    n = fuse_file(args.input, args.out, whitelist, args.period)
    print(f"{n} detection reports fused; picture written to {args.out}")
    return 0


def _endpoint(text: str, default_host: str = "127.0.0.1") -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    return host or default_host, int(port)


def _tls(args: argparse.Namespace, server: bool) -> ssl.SSLContext | None:
    given = [args.tls_ca, args.tls_cert, args.tls_key]
    if not any(given):
        return None
    if not all(given):
        sys.exit("TLS needs all of --tls-ca, --tls-cert and --tls-key")
    from kestrel.interop.wire import tls_context

    return tls_context(args.tls_ca, args.tls_cert, args.tls_key, server)


def _stop_on_signal() -> threading.Event:
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    return stop


def cmd_fusion(args: argparse.Namespace) -> int:
    from kestrel.rap.service import FusionService, ServiceOptions, summary_line

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    site = load_scenario(args.site)
    host, port = _endpoint(args.listen)
    tls = _tls(args, server=True)
    if tls is None and host not in ("127.0.0.1", "localhost", "::1"):
        if not args.insecure:
            sys.exit(
                "refusing to listen beyond this machine without TLS: anyone on the network could "
                "inject tracks or read the picture. Pass --tls-ca/--tls-cert/--tls-key, or --insecure "
                "on an isolated network."
            )
        print("WARNING: listening without TLS; the network itself must be trusted", file=sys.stderr)
    sensors = site.sensors
    whitelist = [x for x in (args.whitelist or "").split(",") if x] or (sensors.whitelist if sensors else [])
    opts = ServiceOptions(
        host=host,
        port=port,
        period_s=args.period,
        tasking=not args.no_tasking,
        tls=tls,
        out=Path(args.out) if args.out else None,
    )
    svc = FusionService(site.world, whitelist, sensors.protected if sensors else {}, opts)
    print(f"fusion node {svc.fusion.node_id} listening on {host}:{svc.server.port} (site '{site.name}')")
    cot = None
    if args.cot:
        from kestrel.interop.cot import CotSender

        cot = CotSender(*_endpoint(args.cot))
        print(f"sending the picture as Cursor on Target to {args.cot}")

    def console() -> None:  # the operator's identification decisions, one per line
        for line in sys.stdin:
            if line.strip():
                print(svc.operator_command(line), flush=True)

    if not args.no_console and sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=console, name="operator", daemon=True).start()

    def on_picture(pic: dict[str, Any]) -> None:
        if cot is not None:
            cot.send(pic)
        if not args.quiet:
            print(summary_line(pic), flush=True)

    svc.run(_stop_on_signal(), on_picture)
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    from kestrel.rap.service import stream_log

    host, port = _endpoint(args.connect)
    n = stream_log(args.input, host, port, args.speed, _tls(args, server=False), _stop_on_signal())
    print(f"{n} messages sent to {host}:{port}")
    return 0


def cmd_evaluate(args: argparse.Namespace) -> int:
    from kestrel.perception.evaluate import READERS, evaluate

    seqs = READERS[args.format](Path(args.path))
    if not seqs:
        print(f"no {args.format} sequences found under {args.path}")
        return 2
    params = load_config(args.config).detector if args.config else None
    res = evaluate(seqs, params, args.polarity, every=args.every)
    summary = res.summary()
    Path(args.out).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "per_sequence"}, indent=2))
    print(f"report written to {args.out}")
    return 0


def _flight_setup(args: argparse.Namespace) -> tuple[Any, ...]:
    """Everything a flight needs, opened in the order that fails earliest and cheapest."""
    from kestrel.mission.llm import MissionPlanner

    cfg, sc = _load(args)
    plan = MissionPlanner(list(sc.world.sites), _backend(args)).plan(args.instruction)
    if plan.mission is None:
        sys.exit("\n".join(plan.notes) or "could not build a plan")
    frames = None
    if args.connect == "bench":
        from kestrel.vehicle.bench_vehicle import BenchVehicle

        vehicle: Any = BenchVehicle(cfg, sc)
        if args.camera == "bench":
            frames = vehicle.camera_source()
        align = False
    else:
        from kestrel.vehicle.mavlink_vehicle import MavlinkVehicle

        vehicle = MavlinkVehicle(args.connect, cfg.vehicle.battery_wh)
        vehicle.wait_ready()
        align = not args.no_align
    if args.camera and args.camera != "bench":
        from kestrel.perception.source import OpenCvSource

        frames = OpenCvSource(args.camera, cfg.camera.width, cfg.camera.height)
    client = None
    if args.sapient:
        from kestrel.interop.wire import SapientClient

        host, port = _endpoint(args.sapient)
        tls = _tls(args, server=False)
        if tls is None and host not in ("127.0.0.1", "localhost", "::1") and not args.insecure:
            sys.exit(
                "refusing a fusion link beyond this machine without TLS: tasks and traffic would be "
                "accepted from anyone on the network. Pass --tls-ca/--tls-cert/--tls-key, or --insecure "
                "on an isolated network."
            )
        client = SapientClient(host, port, tls=tls).start()
    log_dir = Path(args.log_dir or time.strftime("runs/flight-%Y%m%dT%H%M%SZ", time.gmtime()))
    return cfg, sc, plan.mission, vehicle, frames, client, align, log_dir


def _preflight(args: argparse.Namespace, setup: tuple[Any, ...]) -> bool:
    from kestrel.preflight import cleared, preflight

    cfg, sc, mission, vehicle, frames, client, align, log_dir = setup
    checks = preflight(mission, cfg, sc.world, vehicle, frames, client, align, log_dir)
    for c in checks:
        print(c.line())
    ok = cleared(checks)
    print("preflight:", "CLEARED" if ok else "NOT CLEARED: a failed check must be fixed before flight")
    return ok


def _close(setup: tuple[Any, ...]) -> None:
    _, _, _, vehicle, frames, client, _, _ = setup
    for thing in (frames, client, vehicle):
        if thing is not None and hasattr(thing, "close"):
            thing.close()


def cmd_preflight(args: argparse.Namespace) -> int:
    setup = _flight_setup(args)
    try:
        return 0 if _preflight(args, setup) else 1
    finally:
        _close(setup)


def cmd_fly(args: argparse.Namespace) -> int:
    from kestrel.live import LiveOptions, MissionRejected, run_live

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    setup = _flight_setup(args)
    cfg, sc, mission, vehicle, frames, client, align, log_dir = setup
    try:
        if not _preflight(args, setup):
            return 1
        if not args.yes:
            print(f"\nmission: {mission.describe()}")
            if input("type FLY to arm and take off: ").strip() != "FLY":
                print("not flown")
                return 1
        opts = LiveOptions(
            frames=frames, sapient=client, log_dir=log_dir, align_site=align, stop=_stop_on_signal()
        )
        res = run_live(mission, cfg, sc.world, vehicle, options=opts)
    except (MissionRejected, ValueError) as exc:
        print(exc)
        return 1
    finally:
        _close(setup)
    for e in res.events:
        print(f"{e.t:7.1f}s {e.kind}: {e.detail}")
    print(f"flight log: {log_dir}/")
    return 0 if res.completed else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="kestrel", description="Airborne sensor node for counter-drone defence")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--config", help="config YAML (defaults built in)")
        p.add_argument("--scenario", help="scenario YAML (default: scenarios/airfield.yaml)")
        p.add_argument("--llm", action="store_true", help="plan with a local Ollama model")
        p.add_argument("--model", default="qwen2.5:3b")
        p.add_argument("--ollama", default="http://127.0.0.1:11434")

    p = sub.add_parser("plan", help="parse and validate an instruction")
    p.add_argument("instruction")
    p.add_argument("--json", action="store_true", help="also print the mission JSON")
    common(p)
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("sim", help="run a closed-loop simulation")
    p.add_argument("instruction")
    p.add_argument("--out", default="runs/latest")
    p.add_argument("--gif", action="store_true", help="also render demo.gif")
    p.add_argument("--seed", type=int)
    p.add_argument(
        "--sapient", metavar="PATH", help="also write SAPIENT (BSI Flex 335 v2.0) messages as JSON lines"
    )
    p.add_argument(
        "--picture",
        action="store_true",
        help="run the scenario's ground sensors and a fusion node that tasks Kestrel (needs `sensors:`)",
    )
    p.add_argument(
        "--command",
        action="append",
        default=[],
        metavar="T:TEXT",
        help="operator command at time T, e.g. 60:abort",
    )
    common(p)
    p.set_defaults(func=cmd_sim)

    p = sub.add_parser("bench", help="Monte Carlo benchmark")
    p.add_argument("--runs", type=int, default=20)
    p.add_argument(
        "--seed0", type=int, default=1000, help="first encounter seed (1000 tuning set, 2000 held-out set)"
    )
    p.add_argument("--jobs", type=int, default=2)
    p.add_argument("--no-avoidance", action="store_true", help="unmitigated baseline for the risk ratio")
    p.add_argument("--intruders", type=int, default=1, help="intruders per encounter")
    p.add_argument("--first", type=int, default=0, help="index of the first encounter (to split a set)")
    p.add_argument("--mission", choices=["survey", "standby"], default="survey", help="benchmark mission")
    p.add_argument("--jam", type=float, default=0.0, metavar="S", help="jam the ground link for S seconds")
    p.add_argument("--observers", type=int, default=1, help="observers flying together")
    p.add_argument("--out", default="runs/bench")
    common(p)
    p.set_defaults(func=cmd_bench)

    p = sub.add_parser("picture", help="air-picture benchmark: each ground sensor alone and all fused")
    p.add_argument("--runs", type=int, default=24)
    p.add_argument("--seed0", type=int, help="first seed (default 7000; 8000 with --loop)")
    p.add_argument("--jobs", type=int, default=2)
    p.add_argument("--only", help="comma-separated configurations (default: all)")
    p.add_argument("--out", help="output folder (default runs/picture; runs/loop with --loop)")
    p.add_argument("--loop", action="store_true", help="fly Kestrel with the picture: listening, and tasked")
    p.add_argument("--tasking", choices=("both", "on", "off"), default="both", help="with --loop")
    p.add_argument("--first", type=int, default=0, help="with --loop: first encounter, to run in parts")
    p.add_argument("--config", default="configs/birds.yaml", help="with --loop")
    p.add_argument("--scenario", help="with --loop (default scenarios/airfield_picture.yaml)")
    p.set_defaults(func=cmd_picture)

    p = sub.add_parser("fuse", help="fuse a recorded SAPIENT log (JSON lines) into an air picture")
    p.add_argument("input")
    p.add_argument("--out", default="rap.jsonl")
    p.add_argument("--whitelist", help="comma-separated Remote ID serials of friendly drones")
    p.add_argument("--period", type=float, default=1.0, help="seconds between pictures")
    p.set_defaults(func=cmd_fuse)

    p = sub.add_parser("evaluate", help="run the detector on recorded footage with ground-truth boxes")
    p.add_argument("path", help="dataset folder")
    p.add_argument("--format", choices=["anti-uav", "yolo", "mot"], default="anti-uav")
    p.add_argument(
        "--polarity", choices=["dark", "bright"], default="dark", help="bright: thermal (white hot) footage"
    )
    p.add_argument("--every", type=int, default=1, help="use every N-th frame")
    p.add_argument("--config", help="config YAML whose detector settings to use")
    p.add_argument("--out", default="evaluation.json")
    p.set_defaults(func=cmd_evaluate)

    def tls_args(p: argparse.ArgumentParser) -> None:
        p.add_argument("--tls-ca", help="CA certificate both ends must be signed by (mutual TLS 1.3)")
        p.add_argument("--tls-cert", help="this end's certificate")
        p.add_argument("--tls-key", help="this end's private key")

    p = sub.add_parser("fusion", help="run the fusion node as a live SAPIENT service")
    p.add_argument("--site", required=True, help="site YAML: origin, fence, protected areas, whitelist")
    p.add_argument("--listen", default="127.0.0.1:5020", help="host:port to accept SAPIENT nodes on")
    p.add_argument("--out", help="append the air picture here, one JSON line per period")
    p.add_argument("--period", type=float, default=1.0)
    p.add_argument("--whitelist", help="comma-separated friendly Remote ID serials (default: the site's)")
    p.add_argument("--no-tasking", action="store_true", help="listen and fuse only; send no LOOK_AT tasks")
    p.add_argument("--insecure", action="store_true", help="allow a non-local listener without TLS")
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--cot", help="also send tracks as Cursor on Target over UDP, e.g. 239.2.3.1:6969 (ATAK)")
    p.add_argument(
        "--no-console", action="store_true", help="do not read operator commands from the terminal"
    )
    tls_args(p)
    p.set_defaults(func=cmd_fusion)

    p = sub.add_parser("send", help="replay a recorded SAPIENT log to a fusion node in real time")
    p.add_argument("input")
    p.add_argument("--connect", required=True, help="host:port of the fusion node")
    p.add_argument("--speed", type=float, default=1.0)
    tls_args(p)
    p.set_defaults(func=cmd_send)

    def flight_args(p: argparse.ArgumentParser) -> None:
        p.add_argument("instruction")
        p.add_argument(
            "--connect",
            required=True,
            help="autopilot, e.g. tcp:127.0.0.1:5760, udp:0.0.0.0:14550, /dev/ttyAMA0; or 'bench' "
            "for the simulated vehicle behind the real loop",
        )
        p.add_argument(
            "--camera",
            help="camera for OpenCV: device index, GStreamer pipeline, URL or file; 'bench' renders",
        )
        p.add_argument("--sapient", help="host:port of the fusion node to report to and take tasks from")
        p.add_argument("--insecure", action="store_true", help="allow a non-local fusion link without TLS")
        p.add_argument("--log-dir", help="flight log directory (default runs/flight-<UTC time>)")
        p.add_argument(
            "--no-align",
            action="store_true",
            help="do not place the vehicle on the site from its GNSS position",
        )
        tls_args(p)
        common(p)

    p = sub.add_parser("preflight", help="run the pre-flight checks for a mission and stop")
    flight_args(p)
    p.set_defaults(func=cmd_preflight)

    p = sub.add_parser("fly", help="fly an ArduPilot vehicle over MAVLink (SITL or real), or the bench")
    flight_args(p)
    p.add_argument("--yes", action="store_true", help="arm without asking (automation and tests only)")
    p.set_defaults(func=cmd_fly)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
