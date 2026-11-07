"""Nominal sampled structural surfaces. No scene perturbation or simulator access."""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import trimesh


def is_structure(name: str, include_floor: bool = True) -> bool:
    return any((token in name for token in ("wall", "divider", "barrier", "tube_oval"))) or (
        include_floor and name.startswith("floor_20_")
    )


@dataclass
class StructuralMap:
    """Distance to sampled *surfaces*, never convex hulls of hollow structures.

    Samples lie on the original triangles. Voxel reduction retains actual surface
    points, with spacing-scale approximation error; it must be accounted for in
    the observation likelihood. This is not a collision signed-distance field.
    """

    points: np.ndarray
    source_names: tuple[str, ...]
    spacing: float
    point_normals: np.ndarray | None = None
    point_surface_ids: np.ndarray | None = None
    point_object_ids: np.ndarray | None = None
    object_reference_points: np.ndarray | None = None
    surface_names: tuple[str, ...] = ()

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float)
        if (
            self.points.ndim != 2
            or self.points.shape[1] != 3
            or (not len(self.points))
            or (not np.isfinite(self.points).all())
        ):
            raise ValueError("Map points must be a nonempty finite (N,3) array")
        count = len(self.points)
        if self.point_normals is None:
            self.point_normals = np.zeros((count, 3))
        if self.point_surface_ids is None:
            self.point_surface_ids = np.full(count, -1, dtype=np.int64)
        if self.point_object_ids is None:
            self.point_object_ids = np.full(count, -1, dtype=np.int64)
        if self.object_reference_points is None:
            self.object_reference_points = np.zeros((len(self.source_names), 3))
        self.point_normals = np.asarray(self.point_normals, dtype=float)
        self.point_surface_ids = np.asarray(self.point_surface_ids, dtype=np.int64)
        self.point_object_ids = np.asarray(self.point_object_ids, dtype=np.int64)
        self.object_reference_points = np.asarray(self.object_reference_points, dtype=float)
        if (
            self.point_normals.shape != (count, 3)
            or self.point_surface_ids.shape != (count,)
            or self.point_object_ids.shape != (count,)
        ):
            raise ValueError("Map normals and labels must match point count")
        self._tree = cKDTree(self.points)

    @classmethod
    def from_assets(cls, root: str | Path, spacing: float = 0.035, include_floor: bool = True):
        if spacing <= 0:
            raise ValueError("spacing must be positive")
        root = Path(root).resolve()
        directory = root / "assets" / "meshes"
        if not directory.exists():
            directory = root / "meshes" if (root / "meshes").exists() else root
        clouds, names, normals, surface_ids, object_ids, anchors, surface_names = (
            [],
            [],
            [],
            [],
            [],
            [],
            [],
        )
        for path in sorted(directory.glob("*.obj")):
            if not is_structure(path.stem, include_floor):
                continue
            mesh = trimesh.load(path, process=False, force="mesh")
            object_id = len(names)
            names.append(path.stem)
            anchors.append(mesh.bounds.mean(axis=0))
            planes = {}
            for triangle, normal in zip(mesh.triangles, mesh.face_normals):
                if np.linalg.norm(normal) < 0.5:
                    continue
                plane_offset = float(np.dot(normal, triangle[0]))
                key = tuple(np.round(normal, 3)) + (round(plane_offset, 3),)
                if key not in planes:
                    planes[key] = len(surface_names)
                    surface_names.append(f"{path.stem}:plane_{len(planes) - 1}")
                surface_id = planes[key]
                n = max(
                    1,
                    int(
                        np.ceil(
                            np.max(np.linalg.norm(triangle - np.roll(triangle, 1, axis=0), axis=1))
                            / spacing
                        )
                    ),
                )
                i, j = np.triu_indices(n + 1)
                a, b = (i / n, (j - i) / n)
                cloud = (
                    triangle[0]
                    + a[:, None] * (triangle[1] - triangle[0])
                    + b[:, None] * (triangle[2] - triangle[0])
                )
                clouds.append(cloud)
                normals.append(np.broadcast_to(normal, (len(cloud), 3)))
                surface_ids.append(np.full(len(cloud), surface_id, dtype=np.int64))
                object_ids.append(np.full(len(cloud), object_id, dtype=np.int64))
        if not clouds:
            raise ValueError(f"No nominal structural meshes found in {directory}")
        points = np.vstack(clouds)
        _, keep = np.unique(np.floor(points / spacing).astype(np.int64), axis=0, return_index=True)
        return cls(
            points[keep],
            tuple(names),
            spacing,
            np.vstack(normals)[keep],
            np.concatenate(surface_ids)[keep],
            np.concatenate(object_ids)[keep],
            np.asarray(anchors),
            tuple(surface_names),
        )

    def distance(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        result = np.full(len(points), np.inf)
        valid = np.isfinite(points).all(axis=1)
        if np.any(valid):
            result[valid] = self._tree.query(points[valid], workers=1)[0]
        return result

    def query(self, points: np.ndarray) -> dict[str, np.ndarray]:
        """Nearest bounded surface sample plus actual triangle metadata.

        Coplanar labels do NOT extend surfaces into infinite planes. Distance is
        Euclidean sample distance (same as distance()), so apertures stay open.
        At edges/ties a single nearest face is returned; normals are not blended
        across corners. source_names[object_id] identifies shared wall geometry.
        Legacy cloud-only maps return unknown labels and zero normals.
        """
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        count = len(points)
        result = {
            "distance": np.full(count, np.inf),
            "normal": np.zeros((count, 3)),
            "surface_id": np.full(count, -1, dtype=np.int64),
            "object_id": np.full(count, -1, dtype=np.int64),
            "reference_point": np.full((count, 3), np.nan),
            "object_reference_point": np.full((count, 3), np.nan),
            "is_floor": np.zeros(count, dtype=bool),
        }
        valid = np.flatnonzero(np.isfinite(points).all(axis=1))
        if not len(valid):
            return result
        distance, nearest = self._tree.query(points[valid], workers=1)
        objects = self.point_object_ids[nearest]
        result["distance"][valid] = distance
        result["normal"][valid] = self.point_normals[nearest]
        result["surface_id"][valid] = self.point_surface_ids[nearest]
        result["object_id"][valid] = objects
        result["reference_point"][valid] = self.points[nearest]
        known = objects >= 0
        if known.any():
            result["object_reference_point"][valid[known]] = self.object_reference_points[
                objects[known]
            ]
            result["is_floor"][valid[known]] = [
                self.source_names[obj].startswith("floor_20_") for obj in objects[known]
            ]
        return result
