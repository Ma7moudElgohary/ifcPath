from __future__ import annotations

import json
import sys
from pathlib import Path

from ifcpath.evacuation import EvacuationSimulator, spawn_agents
from ifcpath.exporter import load_inav


def main() -> None:
    if len(sys.argv) not in {2, 3}:
        raise SystemExit("usage: assert_evacuation_qualification.py MODEL.inav [OUTPUT.json]")

    model = load_inav(sys.argv[1])
    agents = spawn_agents(
        model,
        24,
        min_speed_mps=0.9,
        max_speed_mps=1.4,
        seed=2026,
    )
    simulator = EvacuationSimulator(model, agents)
    while not simulator.finished and simulator.elapsed_s < 300.0:
        simulator.advance(0.25)

    stats = simulator.stats
    payload = {
        "elapsed_s": stats.elapsed_s,
        "total_agents": stats.total_agents,
        "evacuated_agents": stats.evacuated_agents,
        "active_agents": stats.active_agents,
        "waiting_agents": stats.waiting_agents,
        "trapped_agents": stats.trapped_agents,
        "average_evacuation_time_s": stats.average_evacuation_time_s,
        "clearance_time_s": stats.clearance_time_s,
        "max_queue": stats.max_queue,
        "exit_usage": stats.exit_usage,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    if len(sys.argv) == 3:
        Path(sys.argv[2]).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if stats.total_agents != 24:
        raise SystemExit(f"expected 24 agents, got {stats.total_agents}")
    if stats.evacuated_agents < 20:
        raise SystemExit(
            f"real IFC crowd qualification requires >=20/24 evacuated agents; got {stats.evacuated_agents}"
        )
    if stats.active_agents:
        raise SystemExit(f"simulation did not settle within 300 s; active={stats.active_agents}")
    if not stats.exit_usage:
        raise SystemExit("no real IFC exit served any occupant")


if __name__ == "__main__":
    main()
