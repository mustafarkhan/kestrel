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
    print(f"simulating '{sc.name}' (seed {sc.seed}) ...", flush=True)
    res = run(args.instruction, cfg, sc, _backend(args), _parse_commands(args.command), record_every=record)
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
    summary = run_benchmark(cfg, sc, args.runs, Path(args.out), jobs=args.jobs)
    print(json.dumps(summary, indent=2))
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
    ap = argparse.ArgumentParser(prog="kestrel", description="Language-tasked aerial observer")
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
    p.add_argument("--jobs", type=int, default=2)
    p.add_argument("--out", default="runs/bench")
    common(p)
    p.set_defaults(func=cmd_bench)

    p = sub.add_parser("fly", help="fly an ArduPilot vehicle over MAVLink (SITL or real)")
    p.add_argument("instruction")
    p.add_argument("--connect", required=True, help="e.g. tcp:127.0.0.1:5760 or udp:0.0.0.0:14550")
    common(p)
    p.set_defaults(func=cmd_fly)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
