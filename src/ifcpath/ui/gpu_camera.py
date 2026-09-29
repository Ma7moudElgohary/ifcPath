from __future__ import annotations

import math
from dataclasses import dataclass

from ..model import Vec3


@dataclass(slots=True)
class GpuCameraState:
    target: Vec3 = (0.0, 0.0, 0.0)
    yaw_deg: float = 35.0
    pitch_deg: float = 38.0
    distance: float = 20.0
    fov_y_deg: float = 45.0
    near: float = 0.05
    far: float = 10000.0

    def orbit(self, dx_px: float, dy_px: float, *, sensitivity: float = 0.22) -> None:
        self.yaw_deg = (self.yaw_deg + dx_px * sensitivity) % 360.0
        self.pitch_deg = min(88.0, max(-88.0, self.pitch_deg - dy_px * sensitivity))

    def dolly(self, wheel_steps: float, *, sensitivity: float = 0.14) -> None:
        factor = math.exp(-wheel_steps * sensitivity)
        self.distance = min(1_000_000.0, max(0.02, self.distance * factor))

    def pan(self, dx_px: float, dy_px: float, viewport_height_px: float) -> None:
        height = max(1.0, float(viewport_height_px))
        world_per_px = 2.0 * self.distance * math.tan(math.radians(self.fov_y_deg) * 0.5) / height
        right, up, _forward = self.axes()
        scale_x = -dx_px * world_per_px
        scale_y = dy_px * world_per_px
        self.target = (
            self.target[0] + right[0] * scale_x + up[0] * scale_y,
            self.target[1] + right[1] * scale_x + up[1] * scale_y,
            self.target[2] + right[2] * scale_x + up[2] * scale_y,
        )

    def fit_bounds(self, bounds_min: Vec3, bounds_max: Vec3, *, margin: float = 1.35) -> None:
        self.target = tuple((bounds_min[i] + bounds_max[i]) * 0.5 for i in range(3))  # type: ignore[assignment]
        extent = [max(0.001, bounds_max[i] - bounds_min[i]) for i in range(3)]
        radius = 0.5 * math.sqrt(sum(value * value for value in extent))
        half_fov = max(math.radians(5.0), math.radians(self.fov_y_deg) * 0.5)
        self.distance = max(0.1, radius * margin / math.sin(half_fov))
        self.near = max(0.01, self.distance - radius * 3.0)
        self.far = max(self.near + 100.0, self.distance + radius * 8.0)

    def position(self) -> Vec3:
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        cp = math.cos(pitch)
        direction = (
            cp * math.cos(yaw),
            cp * math.sin(yaw),
            math.sin(pitch),
        )
        return (
            self.target[0] - direction[0] * self.distance,
            self.target[1] - direction[1] * self.distance,
            self.target[2] - direction[2] * self.distance,
        )

    def axes(self) -> tuple[Vec3, Vec3, Vec3]:
        position = self.position()
        forward = _normalize(_sub(self.target, position))
        world_up: Vec3 = (0.0, 0.0, 1.0)
        right = _normalize(_cross(forward, world_up))
        if _length(right) <= 1e-9:
            right = (1.0, 0.0, 0.0)
        up = _normalize(_cross(right, forward))
        return right, up, forward

    def view_projection(self, aspect: float) -> tuple[float, ...]:
        view = _look_at(self.position(), self.target, (0.0, 0.0, 1.0))
        projection = _perspective_webgpu(
            math.radians(self.fov_y_deg),
            max(1e-6, float(aspect)),
            max(1e-5, self.near),
            max(self.near + 1e-3, self.far),
        )
        return _mat_mul(projection, view)

    def screen_ray(
        self,
        x_px: float,
        y_px: float,
        width_px: float,
        height_px: float,
    ) -> tuple[Vec3, Vec3]:
        width = max(1.0, float(width_px))
        height = max(1.0, float(height_px))
        ndc_x = 2.0 * float(x_px) / width - 1.0
        ndc_y = 1.0 - 2.0 * float(y_px) / height
        aspect = width / height
        tan_half = math.tan(math.radians(self.fov_y_deg) * 0.5)
        right, up, forward = self.axes()
        direction = _normalize(
            (
                forward[0] + right[0] * ndc_x * aspect * tan_half + up[0] * ndc_y * tan_half,
                forward[1] + right[1] * ndc_x * aspect * tan_half + up[1] * ndc_y * tan_half,
                forward[2] + right[2] * ndc_x * aspect * tan_half + up[2] * ndc_y * tan_half,
            )
        )
        return self.position(), direction


def ray_triangle_distance(origin: Vec3, direction: Vec3, triangle: tuple[Vec3, Vec3, Vec3]) -> float | None:
    """Möller–Trumbore ray/triangle intersection distance."""

    a, b, c = triangle
    edge1 = _sub(b, a)
    edge2 = _sub(c, a)
    pvec = _cross(direction, edge2)
    determinant = _dot(edge1, pvec)
    if abs(determinant) < 1e-9:
        return None
    inv_det = 1.0 / determinant
    tvec = _sub(origin, a)
    u = _dot(tvec, pvec) * inv_det
    if u < 0.0 or u > 1.0:
        return None
    qvec = _cross(tvec, edge1)
    v = _dot(direction, qvec) * inv_det
    if v < 0.0 or u + v > 1.0:
        return None
    distance = _dot(edge2, qvec) * inv_det
    return distance if distance >= 0.0 else None


def _look_at(eye: Vec3, target: Vec3, up_hint: Vec3) -> tuple[float, ...]:
    forward = _normalize(_sub(target, eye))
    right = _normalize(_cross(forward, up_hint))
    if _length(right) <= 1e-9:
        right = (1.0, 0.0, 0.0)
    up = _cross(right, forward)
    # Column-major matrix for WGSL mat4x4<f32>.
    return (
        right[0], up[0], -forward[0], 0.0,
        right[1], up[1], -forward[1], 0.0,
        right[2], up[2], -forward[2], 0.0,
        -_dot(right, eye), -_dot(up, eye), _dot(forward, eye), 1.0,
    )


def _perspective_webgpu(fov_y: float, aspect: float, near: float, far: float) -> tuple[float, ...]:
    f = 1.0 / math.tan(fov_y * 0.5)
    nf = 1.0 / (near - far)
    return (
        f / aspect, 0.0, 0.0, 0.0,
        0.0, f, 0.0, 0.0,
        0.0, 0.0, far * nf, -1.0,
        0.0, 0.0, far * near * nf, 0.0,
    )


def _mat_mul(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    out = [0.0] * 16
    for col in range(4):
        for row in range(4):
            out[col * 4 + row] = sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4))
    return tuple(out)


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(v: Vec3) -> float:
    return math.sqrt(_dot(v, v))


def _normalize(v: Vec3) -> Vec3:
    length = _length(v)
    if length <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / length, v[1] / length, v[2] / length)
