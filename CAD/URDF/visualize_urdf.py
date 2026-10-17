"""Visualize rotary_double_pendulum.urdf and animate its joints.

Usage: python visualize_urdf.py [--static] [--mesh-scale S]
"""
import argparse
import time
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy

URDF_PATH = Path(__file__).with_name("rotary_double_pendulum.urdf")

# Sinusoidal test motion per joint: (amplitude [rad], frequency [Hz])
MOTION = {
    "main_axis_joint": (1.0, 0.2),
    "pendulum1_joint": (0.8, 0.35),
    "pendulum2_joint": (1.2, 0.5),
}


# Camera axes (screen right, screen up, direction toward the camera) in world coordinates.
# x is the vertical axis of the robot: top looks down -x (yz plane), front looks along -y (xz plane).
VIEWS = {
    "top": ((0, 1, 0), (0, 0, 1), (1, 0, 0)),
    "front": ((0, 0, 1), (1, 0, 0), (0, 1, 0)),
    "side": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
}


def load(mesh_scale):
    urdf = yourdfpy.URDF.load(
        str(URDF_PATH),
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=False,
        load_collision_meshes=False,
    )
    if mesh_scale != 1.0:
        for geom in urdf.scene.geometry.values():
            geom.apply_scale(mesh_scale)
    return urdf


def report(urdf):
    print("Links:", list(urdf.link_map))
    print("Actuated joints:", urdf.actuated_joint_names)
    for name, geom in urdf.scene.geometry.items():
        ext = geom.bounds[1] - geom.bounds[0]
        print(f"  {name}: extents [m] = {np.round(ext, 4)}")
    for jn, joint in urdf.joint_map.items():
        print(f"  {jn}: {joint.parent} -> {joint.child}, origin xyz = {joint.origin[:3, 3]}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--view", choices=VIEWS, default="front")
    parser.add_argument("--static", action="store_true", help="show zero pose only")
    parser.add_argument("--mesh-scale", type=float, default=1.0,
                        help="e.g. 0.001 if the STL files are in millimetres")
    args = parser.parse_args()

    urdf = load(args.mesh_scale)
    report(urdf)

    scene = urdf.scene
    scene.add_geometry(trimesh.creation.axis(origin_size=0.005, axis_length=0.1),
                       node_name="world_axes")
    center = scene.centroid
    right, up, back = VIEWS[args.view]
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = right, up, back
    pose[:3, 3] = center + 2.0 * scene.scale * np.array(back, dtype=float)
    scene.camera_transform = pose
    start = time.time()

    def animate(_scene):
        t = time.time() - start
        cfg = [MOTION[n][0] * np.sin(2 * np.pi * MOTION[n][1] * t)
               for n in urdf.actuated_joint_names]
        urdf.update_cfg(np.array(cfg))

    if args.static:
        scene.show()
    else:
        scene.show(callback=animate, callback_period=1 / 30)


if __name__ == "__main__":
    main()
