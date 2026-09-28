from __future__ import annotations

from dataclasses import dataclass

_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class ElevatorRuntimeConfig:
    """Operational assumptions for one elevator car.

    These values are intentionally runtime configuration rather than BIM facts.
    A project can replace them with manufacturer/BMS data when available.
    """

    speed_mps: float = 1.5
    door_dwell_s: float = 2.0
    capacity_persons: int = 8


@dataclass(frozen=True, slots=True)
class ElevatorRequest:
    agent_id: str
    from_level_id: str
    to_level_id: str
    sequence: int


@dataclass(frozen=True, slots=True)
class ElevatorSnapshot:
    elapsed_s: float
    phase: str
    car_z_m: float
    current_level_id: str | None
    target_level_id: str | None
    queued_agent_ids: tuple[str, ...]
    onboard_agent_ids: tuple[str, ...]
    completed_agent_ids: tuple[str, ...]


class ElevatorDispatcher:
    """Deterministic single-car elevator scheduler for semantic transfers.

    IFCPath routing decides *that* an agent should use an elevator. This class
    models the temporal resource that routing alone cannot express: a shared car
    with finite capacity, door dwell, empty-car repositioning and vertical travel.

    Requests are served FIFO. Agents waiting at the same origin and travelling to
    the same destination are batched together up to ``capacity_persons``. The
    dispatcher deliberately does not invent traffic-control heuristics such as
    collective/selective control; those can be added behind this contract later.
    """

    def __init__(
        self,
        level_elevations_m: dict[str, float],
        *,
        initial_level_id: str | None = None,
        config: ElevatorRuntimeConfig | None = None,
    ) -> None:
        if not level_elevations_m:
            raise ValueError("elevator requires at least one served level")
        self.level_elevations_m = {
            str(level_id): float(z) for level_id, z in level_elevations_m.items()
        }
        self.config = config or ElevatorRuntimeConfig()
        if self.config.speed_mps <= 0.0:
            raise ValueError("elevator speed_mps must be > 0")
        if self.config.door_dwell_s < 0.0:
            raise ValueError("elevator door_dwell_s must be >= 0")
        if self.config.capacity_persons <= 0:
            raise ValueError("elevator capacity_persons must be > 0")

        if initial_level_id is None:
            initial_level_id = min(
                self.level_elevations_m,
                key=lambda level_id: (self.level_elevations_m[level_id], level_id),
            )
        if initial_level_id not in self.level_elevations_m:
            raise ValueError(f"unknown initial elevator level: {initial_level_id}")

        self.elapsed_s = 0.0
        self.phase = "idle"
        self.current_level_id: str | None = initial_level_id
        self.target_level_id: str | None = None
        self.car_z_m = self.level_elevations_m[initial_level_id]

        self._queue: list[ElevatorRequest] = []
        self._onboard: list[ElevatorRequest] = []
        self._completed: list[str] = []
        self._next_sequence = 0
        self._phase_duration_s = 0.0
        self._phase_remaining_s = 0.0
        self._phase_start_z_m = self.car_z_m
        self._phase_target_z_m = self.car_z_m
        self._active_origin_level_id: str | None = None
        self._active_destination_level_id: str | None = None

    @property
    def queued_agent_ids(self) -> tuple[str, ...]:
        return tuple(request.agent_id for request in self._queue)

    @property
    def onboard_agent_ids(self) -> tuple[str, ...]:
        return tuple(request.agent_id for request in self._onboard)

    @property
    def completed_agent_ids(self) -> tuple[str, ...]:
        return tuple(self._completed)

    @property
    def idle(self) -> bool:
        return self.phase == "idle" and not self._queue and not self._onboard

    def enqueue(self, agent_id: str, from_level_id: str, to_level_id: str) -> None:
        agent_id = str(agent_id)
        from_level_id = str(from_level_id)
        to_level_id = str(to_level_id)
        if not agent_id:
            raise ValueError("elevator agent_id must not be empty")
        if from_level_id not in self.level_elevations_m:
            raise ValueError(f"unknown elevator origin level: {from_level_id}")
        if to_level_id not in self.level_elevations_m:
            raise ValueError(f"unknown elevator destination level: {to_level_id}")
        if from_level_id == to_level_id:
            raise ValueError("elevator origin and destination levels must differ")
        if self.contains_agent(agent_id) or agent_id in self._completed:
            raise ValueError(f"duplicate elevator agent id: {agent_id}")

        self._queue.append(
            ElevatorRequest(
                agent_id=agent_id,
                from_level_id=from_level_id,
                to_level_id=to_level_id,
                sequence=self._next_sequence,
            )
        )
        self._next_sequence += 1
        if self.phase == "idle":
            self._start_next_cycle()

    def contains_agent(self, agent_id: str) -> bool:
        return any(request.agent_id == agent_id for request in self._queue) or any(
            request.agent_id == agent_id for request in self._onboard
        )

    def cancel(self, agent_id: str) -> bool:
        """Cancel a request that has not boarded yet.

        A boarded passenger cannot be removed from an active car trip; callers
        should wait for completion and replan from the destination level.
        """
        for index, request in enumerate(self._queue):
            if request.agent_id == agent_id:
                del self._queue[index]
                return True
        return False

    def pop_completed(self) -> tuple[str, ...]:
        result = tuple(self._completed)
        self._completed.clear()
        return result

    def snapshot(self) -> ElevatorSnapshot:
        return ElevatorSnapshot(
            elapsed_s=self.elapsed_s,
            phase=self.phase,
            car_z_m=self.car_z_m,
            current_level_id=self.current_level_id,
            target_level_id=self.target_level_id,
            queued_agent_ids=self.queued_agent_ids,
            onboard_agent_ids=self.onboard_agent_ids,
            completed_agent_ids=self.completed_agent_ids,
        )

    def advance(self, delta_seconds: float) -> ElevatorSnapshot:
        remaining = max(0.0, float(delta_seconds))
        self._settle_zero_duration_phases()
        while remaining > _EPSILON:
            if self.phase == "idle":
                self._start_next_cycle()
                self._settle_zero_duration_phases()
                if self.phase == "idle":
                    self.elapsed_s += remaining
                    remaining = 0.0
                    break

            step = min(remaining, self._phase_remaining_s)
            if step <= _EPSILON:
                self._settle_zero_duration_phases()
                if self.phase != "idle" and self._phase_remaining_s <= _EPSILON:
                    raise RuntimeError("elevator zero-duration phase did not settle")
                continue

            self._advance_motion(step)
            self._phase_remaining_s = max(0.0, self._phase_remaining_s - step)
            self.elapsed_s += step
            remaining -= step
            if self._phase_remaining_s <= _EPSILON:
                self._finish_phase()
                self._settle_zero_duration_phases()

        # A finite phase may end exactly at the caller's time boundary and enter
        # a zero-duration successor (for example travel -> zero-dwell alighting).
        # Settle that successor before exposing the snapshot/completion events.
        self._settle_zero_duration_phases()
        return self.snapshot()

    def _settle_zero_duration_phases(self) -> None:
        # Each pass either reaches a positive-duration phase/idle state or
        # completes one phase. The guard catches accidental zero-time cycles.
        guard = 0
        max_passes = max(16, 4 * (len(self._queue) + len(self._onboard) + 1))
        while self.phase != "idle" and self._phase_remaining_s <= _EPSILON:
            guard += 1
            if guard > max_passes:
                raise RuntimeError("elevator zero-duration phase cycle did not converge")
            self._finish_phase()

    def _start_next_cycle(self) -> None:
        if not self._queue:
            self.phase = "idle"
            self.target_level_id = None
            self._active_origin_level_id = None
            self._active_destination_level_id = None
            return

        first = self._queue[0]
        self._active_origin_level_id = first.from_level_id
        self._active_destination_level_id = first.to_level_id
        if self.current_level_id != first.from_level_id:
            self._begin_motion("reposition", first.from_level_id)
            return
        self._begin_boarding()

    def _begin_boarding(self) -> None:
        origin = self._active_origin_level_id
        destination = self._active_destination_level_id
        if origin is None or destination is None:
            raise RuntimeError("elevator boarding without an active trip")

        selected: list[ElevatorRequest] = []
        remaining: list[ElevatorRequest] = []
        for request in self._queue:
            if (
                len(selected) < self.config.capacity_persons
                and request.from_level_id == origin
                and request.to_level_id == destination
            ):
                selected.append(request)
            else:
                remaining.append(request)
        if not selected:
            raise RuntimeError("elevator active trip has no boardable requests")

        self._queue = remaining
        self._onboard = selected
        self.phase = "boarding"
        self.target_level_id = origin
        self._set_phase_timer(self.config.door_dwell_s)

    def _begin_motion(self, phase: str, target_level_id: str) -> None:
        target_z = self.level_elevations_m[target_level_id]
        distance = abs(target_z - self.car_z_m)
        duration = distance / self.config.speed_mps
        self.phase = phase
        self.target_level_id = target_level_id
        self._phase_start_z_m = self.car_z_m
        self._phase_target_z_m = target_z
        self._set_phase_timer(duration)

    def _set_phase_timer(self, duration_s: float) -> None:
        self._phase_duration_s = max(0.0, float(duration_s))
        self._phase_remaining_s = self._phase_duration_s

    def _advance_motion(self, step_s: float) -> None:
        if self.phase not in {"reposition", "travel"}:
            return
        duration = self._phase_duration_s
        if duration <= _EPSILON:
            self.car_z_m = self._phase_target_z_m
            return
        before_remaining = self._phase_remaining_s
        after_remaining = max(0.0, before_remaining - step_s)
        progress = 1.0 - after_remaining / duration
        progress = min(1.0, max(0.0, progress))
        self.car_z_m = self._phase_start_z_m + (
            self._phase_target_z_m - self._phase_start_z_m
        ) * progress

    def _finish_phase(self) -> None:
        if self.phase == "reposition":
            origin = self._active_origin_level_id
            if origin is None:
                raise RuntimeError("elevator reposition without origin")
            self.car_z_m = self.level_elevations_m[origin]
            self.current_level_id = origin
            self._begin_boarding()
            return

        if self.phase == "boarding":
            destination = self._active_destination_level_id
            if destination is None:
                raise RuntimeError("elevator boarding without destination")
            self.current_level_id = None
            self._begin_motion("travel", destination)
            return

        if self.phase == "travel":
            destination = self._active_destination_level_id
            if destination is None:
                raise RuntimeError("elevator travel without destination")
            self.car_z_m = self.level_elevations_m[destination]
            self.current_level_id = destination
            self.phase = "alighting"
            self.target_level_id = destination
            self._set_phase_timer(self.config.door_dwell_s)
            return

        if self.phase == "alighting":
            self._completed.extend(request.agent_id for request in self._onboard)
            self._onboard.clear()
            self.phase = "idle"
            self.target_level_id = None
            self._active_origin_level_id = None
            self._active_destination_level_id = None
            self._phase_duration_s = 0.0
            self._phase_remaining_s = 0.0
            self._start_next_cycle()
            return

        if self.phase == "idle":
            self._start_next_cycle()
            return

        raise RuntimeError(f"unknown elevator phase: {self.phase}")
