from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

from ifcpath.evacuation import EvacuationAgentSpec
from ifcpath.simulation_playback import PLAYBACK_SCHEMA, record_playback


@dataclass
class _Stats:
    elapsed_s: float
    total_agents: int = 1
    evacuated_agents: int = 0


class _FakeSimulator:
    def __init__(self):
        self.elapsed_s = 0.0
        self.finished = False
        self.hybrid_config = SimpleNamespace(local_backend="kinematic")
        self.elevator_snapshots = {}
        self.agents = [
            SimpleNamespace(
                spec=EvacuationAgentSpec("a", (0.0, 0.0, 0.0), 1.0),
                position=(0.0, 0.0, 0.0),
                forward_xy=(1.0, 0.0),
                status="moving",
                active_level_id="L1",
                active_space_id="A",
                waiting_gate_id=None,
                active_elevator_resource_id=None,
            )
        ]

    @property
    def stats(self):
        return _Stats(
            elapsed_s=self.elapsed_s,
            evacuated_agents=1 if self.finished else 0,
        )

    def advance(self, delta_seconds: float):
        self.elapsed_s += delta_seconds
        state = self.agents[0]
        state.position = (self.elapsed_s, 0.0, 0.0)
        if self.elapsed_s >= 0.4 - 1e-9:
            self.finished = True
            state.status = "evacuated"


def test_record_playback_preserves_agent_motion_and_status():
    playback = record_playback(_FakeSimulator(), frame_interval_s=0.2, max_time_s=2.0)

    assert playback["schema"] == PLAYBACK_SCHEMA
    assert playback["backend"] == "kinematic"
    assert [frame["time_s"] for frame in playback["frames"]] == [0.0, 0.2, 0.4]
    assert playback["frames"][1]["agents"][0]["position_m"] == (0.2, 0.0, 0.0)
    assert playback["frames"][-1]["agents"][0]["status"] == "evacuated"
    assert playback["summary"]["evacuated_agents"] == 1
