from __future__ import annotations

import argparse
import json

from ifcpath.evacuation import EvacuationAgentSpec
from ifcpath.exporter import load_inav
from ifcpath.hybrid_evacuation import HybridEvacuationConfig, HybridEvacuationSimulator
from ifcpath.simulation_playback import record_playback, save_playback


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run IFCPath hybrid evacuation and export renderer-independent playback JSON"
    )
    parser.add_argument("inav")
    parser.add_argument("agents_json", help="JSON array or {agents:[...]} with id/start_m/speed_mps")
    parser.add_argument("output")
    parser.add_argument("--backend", choices=("kinematic", "jupedsim"), default="kinematic")
    parser.add_argument("--frame-interval", type=float, default=0.20)
    parser.add_argument("--max-time", type=float, default=3600.0)
    args = parser.parse_args()

    raw = json.loads(open(args.agents_json, encoding="utf-8").read())
    values = raw.get("agents", []) if isinstance(raw, dict) else raw
    agents = [
        EvacuationAgentSpec(
            id=str(item["id"]),
            start_m=tuple(float(value) for value in item["start_m"]),
            speed_mps=float(item.get("speed_mps", 1.2)),
        )
        for item in values
    ]
    if not agents:
        raise SystemExit("agents JSON contains no agents")

    model = load_inav(args.inav)
    simulator = HybridEvacuationSimulator(
        model,
        agents,
        hybrid_config=HybridEvacuationConfig(local_backend=args.backend),
    )
    playback = record_playback(
        simulator,
        frame_interval_s=args.frame_interval,
        max_time_s=args.max_time,
    )
    target = save_playback(playback, args.output)
    print(
        f"Wrote {target} frames={len(playback['frames'])} "
        f"backend={playback['backend']} elapsed={playback['summary']['elapsed_s']:.2f}s"
    )


if __name__ == "__main__":
    main()
