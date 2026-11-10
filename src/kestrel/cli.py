"""Command-line interface.

kestrel plan  "<instruction>"            parse + validate, print the plan
kestrel sim   "<instruction>" [--gif]    closed-loop simulation with report
kestrel bench --runs 20                  Monte Carlo benchmark
kestrel fly   "<instruction>" --connect tcp:127.0.0.1:5760   ArduPilot over MAVLink
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from kestrel.config import Config, Scenario, load_config, load_scenario
from kestrel.mission.llm import ChatBackend, OllamaBackend

DEFAULT_SCENARIO = Path(__file__).resolve().parents[2] / "scenarios" / "airfield.yaml"


def _load(args: argparse.Namespace) -> tuple[Config, Scenario]:
    cfg = load_config(args.config)
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


def cmd_fly(args: argparse.Namespace) -> int:
    from kestrel.live import MissionRejected, run_live
    from kestrel.mission.llm import MissionPlanner
    from kestrel.vehicle.mavlink_vehicle import MavlinkVehicle

    cfg, sc = _load(args)
    plan = MissionPlanner(list(sc.world.sites), _backend(args)).plan(args.instruction)
    if plan.mission is None:
        print("\n".join(plan.notes))
        return 2
    vehicle = MavlinkVehicle(args.connect, cfg.vehicle.battery_wh)
    try:
        vehicle.wait_ready()
        res = run_live(plan.mission, cfg, sc.world, vehicle)
    except MissionRejected as exc:
        print(exc)
        return 1
    finally:
        vehicle.close()
    for e in res.events:
        print(f"{e.t:7.1f}s {e.kind}: {e.detail}")
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
    p.add_argument("--sapient", metavar="PATH", help="also write SAPIENT (ICD v7) reports as JSON lines")
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

    p = sub.add_parser("fly", help="fly an ArduPilot vehicle over MAVLink (SITL or real)")
    p.add_argument("instruction")
    p.add_argument("--connect", required=True, help="e.g. tcp:127.0.0.1:5760 or udp:0.0.0.0:14550")
    common(p)
    p.set_defaults(func=cmd_fly)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
