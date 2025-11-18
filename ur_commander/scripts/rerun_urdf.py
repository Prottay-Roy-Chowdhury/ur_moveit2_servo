from __future__ import annotations

import io
import os
from typing import TYPE_CHECKING, cast
from urllib.parse import urlparse

import numpy as np
import rerun as rr
from ament_index_python.packages import get_package_share_directory
from sympy import false  # pip install rerun-sdk
from yourdfpy import URDF

if TYPE_CHECKING:
    import trimesh
    from std_msgs.msg import String


def ament_locate_package(fname: str) -> str:
    """Helper to locate urdf resources via ament."""
    if not fname.startswith("package://"):
        return fname
    parsed = urlparse(fname)
    return os.path.join(get_package_share_directory(parsed.netloc), parsed.path[1:])


def load_urdf_from_msg(msg: String) -> URDF:
    """Load a URDF file using `yourdfpy` and find resources via ament."""
    f = io.StringIO(msg.data)
    return URDF.load(f, filename_handler=ament_locate_package)


def log_scene(
    scene: "trimesh.Scene", node: str, path: str | None = None, static: bool = False
) -> None:
    """
    Recursively log a trimesh.Scene to Rerun.
    Logs both transforms and meshes for every node in the URDF scene.
    """

    path = path + "/" + node if path else node

    # Get parent and children in the scene graph
    parent = scene.graph.transforms.parents.get(node)
    children = scene.graph.transforms.children.get(node)

    # Get transform and geometry associated with this node
    node_data = scene.graph.get(frame_to=node, frame_from=parent)
    if node_data:
        world_from_mesh, geom_name = node_data

        # Log transform for this node (always, even if root)
        rr.log(
            path,
            rr.Transform3D(
                translation=world_from_mesh[0:3, 3],  # last column
                mat3x3=world_from_mesh[0:3, 0:3],  # top-left 3x3 rotation
            ),
            static=False,
        )

        # Log mesh if it exists
        mesh = scene.geometry.get(geom_name)
        if mesh:
            # Determine albedo color from vertex colors if available
            albedo_factor = None
            try:
                colors = np.mean(mesh.visual.vertex_colors, axis=0)
                if len(colors) == 4:
                    albedo_factor = colors / 255.0
            except Exception:
                pass

            if albedo_factor is None:
                try:
                    colors = mesh.visual.to_color().vertex_colors
                    if len(colors) == 4:
                        albedo_factor = colors / 255.0
                except Exception:
                    pass
            # log the mesh with rotation angles

            rr.log(
                path,
                rr.Mesh3D(
                    vertex_positions=mesh.vertices,
                    triangle_indices=mesh.faces,
                    vertex_normals=mesh.vertex_normals,
                    albedo_factor=albedo_factor,
                ),
                static=True,
            )

    # Recurse on children
    if children:
        for child in children:
            log_scene(scene, child, path, static)
