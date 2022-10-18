from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import atan2
from typing import Sequence, Optional, Mapping
import numpy as np
from geometry import SE2value

from .bspline import (
    dynamic_Bspline,
    dynamic_Bspline_tangent,
    dynamic_Bspline_radius,
    dynamic_Bspline_normal,
)


class TrackAbc(ABC):
    _name: str

    @abstractmethod
    def progress_from_pose(self, p: SE2value) -> float:
        ...

    @abstractmethod
    def centerline_from_progress(self, p: float) -> SE2value:
        ...

    @abstractmethod
    def left_boundary_from_progress(self, p: float) -> SE2value:
        ...

    @abstractmethod
    def right_boundary_from_progress(self, p: float) -> SE2value:
        ...

    @abstractmethod
    def get_name(self) -> str:
        ...


@dataclass
class BSplineTrack:
    x: Sequence[float]
    y: Sequence[float]
    radius: Sequence[float]

    def __post_init__(self):
        assert len(set(map(len, [self.x, self.y, self.radius]))) == 1

    def as_np_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.radius]).T

    def get_control_point(self, idx: int):
        idx %= len(self.x)
        return np.array([self.x[idx], self.y[idx], self.radius[idx]])

    def dynamic_Bspline(self, p: float):
        return dynamic_Bspline(p, np.transpose(np.array([self.x, self.y])))

    def dynamic_Bspline_radius(self, p: float):
        return dynamic_Bspline_radius(p, self.radius)

    def dynamic_Bspline_tangent(self, p: float):
        return dynamic_Bspline_tangent(p, np.transpose(np.array([self.x, self.y])))

    def dynamic_Bspline_normal(self, p: float):
        return dynamic_Bspline_normal(p, [self.x, self.y])


@dataclass
class Track(TrackAbc):
    name: str
    spline: BSplineTrack
    background: Optional[np.ndarray]
    "Background image, usually an occupancy grid"
    scale_factor: Optional[float]
    "Scale factor as meters/pixels of the background"

    @classmethod
    def from_dict(cls, d: Mapping):
        spline = BSplineTrack(x=d["x"], y=d["y"], radius=[w / 2 for w in d["w"]])
        return Track(
            name=d["name"], spline=spline, background=d["background"], scale_factor=d["scale_factor"]
        )

    def progress_from_pose(self, pose: np.ndarray) -> float:
        # parametrize curve
        n_t = 500
        t = np.linspace(0, len(self.spline.x), n_t, endpoint=True)
        xxyy = []
        for t_val in t:
            xxyy.append(dynamic_Bspline(t_val, self.spline.as_np_array()[:, 0:2]))
        xxyy = np.asarray(xxyy)

        # find minimum
        eucl_dist = np.linalg.norm(pose[0:2] - xxyy, ord=2, axis=1)
        min_dist = np.amin(eucl_dist)
        min_idx = np.where(eucl_dist == min_dist)
        progress = t[min_idx]
        dx, dy = dynamic_Bspline_tangent(progress, self.spline.as_np_array()[:, 0:2])
        progress_ang = float(atan2(dy, dx))
        assert abs(progress_ang - pose[-1]) < np.pi / 2
        r = float(dynamic_Bspline_radius([progress], self.spline.as_np_array()[:, -1: np.newaxis]))
        assert min_dist < r
        return progress

    def centerline_from_progress(self, p: float) -> SE2value:
        # todo fixme this does not return a SE2value
        return self.spline.dynamic_Bspline(p)

    def left_boundary_from_progress(self, p: float) -> SE2value:
        # todo fixme this does not return a SE2value
        r = self.spline.dynamic_Bspline_radius(p)
        normal = self.spline.dynamic_Bspline_normal(p)
        xy = self.spline.dynamic_Bspline(p)
        return xy + normal * r

    def right_boundary_from_progress(self, p: float) -> SE2value:
        # todo fixme this does not return a SE2value
        r = self.spline.dynamic_Bspline_radius(p)
        normal = self.spline.dynamic_Bspline_normal(p)
        xy = self.spline.dynamic_Bspline(p)
        return xy - normal * r

    def get_name(self) -> str:
        return self.name

    def get_n_points(self, n: int) -> Sequence[SE2value]:
        points = self.spline.as_np_array()
        tck = np.array(np.linspace(0, points.shape[0], num=n, endpoint=True))
        centerline = []
        tangents = []
        for i in tck:
            centerline.append(self.centerline_from_progress(i))
            xx, yy = self.spline.dynamic_Bspline_tangent(i)
            tangents.append(np.arctan(yy / xx))
        return np.array(centerline).reshape((1000, 2)), np.array(tangents).reshape(-1)
