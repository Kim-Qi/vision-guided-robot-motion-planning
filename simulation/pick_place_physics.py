"""Scene authoring and stability criteria; importing this module starts no simulator."""
import json
from pathlib import Path

import numpy as np

from scene_frames import camera_transform


def load_config(path):
    config = json.loads(Path(path).read_text())
    positive = ("cube_size_m", "cube_mass_kg", "spawn_clearance_m",
                "pregrasp_clearance_m", "settle_linear_speed_m_s",
                "settle_angular_speed_rad_s", "settle_position_drift_m",
                "settle_angle_drift_rad", "support_height_tolerance_m",
                "settle_window_s", "min_capture_time_s", "settle_timeout_s",
                "post_capture_observe_s")
    for key in positive:
        value = float(config[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"Invalid positive scene parameter: {key}")
    for key, length in (("cube_xy_world_m", 2), ("support_center_world_m", 3),
                        ("support_dimensions_m", 3)):
        value = np.asarray(config[key], dtype=float)
        if value.shape != (length,) or not np.isfinite(value).all():
            raise ValueError(f"Invalid scene vector: {key}")
    if min(config["support_dimensions_m"]) <= 0:
        raise ValueError("Support dimensions must be positive")
    if not (0 <= config["dynamic_friction"] <= config["static_friction"]
            and np.isfinite(config["static_friction"])
            and 0 <= config["restitution"] <= 1
            and np.isfinite(config["gravity_m_s2"]) and config["gravity_m_s2"] < 0):
        raise ValueError("Invalid friction/restitution/gravity")
    if config["settle_timeout_s"] <= config["min_capture_time_s"] + config["settle_window_s"]:
        raise ValueError("Settling timeout is too short")
    center = np.asarray(config["support_center_world_m"][:2])
    margin = np.asarray(config["support_dimensions_m"][:2]) / 2 - config["cube_size_m"] / 2
    if np.any(np.abs(np.asarray(config["cube_xy_world_m"]) - center) >= margin):
        raise ValueError("Cube spawn is outside support footprint")
    return config


def support_top(config):
    return config["support_center_world_m"][2] + config["support_dimensions_m"][2] / 2


def initial_position(config):
    return [*config["cube_xy_world_m"], support_top(config) + config["cube_size_m"] / 2
            + config["spawn_clearance_m"]]


def configure_physics(stage, cube, floor, config):
    # UsdPhysics authors real physics, separately from MoveIt PlanningScene.
    from pxr import UsdPhysics, UsdShade
    body = UsdPhysics.RigidBodyAPI.Apply(cube)
    body.CreateRigidBodyEnabledAttr().Set(True)
    body.CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI.Apply(cube).CreateMassAttr().Set(config["cube_mass_kg"])
    material = UsdShade.Material.Define(stage, "/World/PickPlacePhysicsMaterial")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr().Set(config["static_friction"])
    physics.CreateDynamicFrictionAttr().Set(config["dynamic_friction"])
    physics.CreateRestitutionAttr().Set(config["restitution"])
    for prim in (cube, floor):
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")
    # No RigidBodyAPI on floor: a static collider, never a kinematic follower.


class StabilityMonitor:
    def __init__(self, config):
        self.config = config
        self.samples = []
        self.window = []

    def update(self, time_s, position, quaternion, linear, angular):
        config = self.config
        pose = camera_transform(position, quaternion)  # generic wxyz rigid pose
        position = pose[:3, 3]
        linear = np.asarray(linear, dtype=float)
        angular = np.asarray(angular, dtype=float)
        if (linear.shape != (3,) or angular.shape != (3,)
                or not np.isfinite(linear).all() or not np.isfinite(angular).all()):
            raise ValueError("Non-finite rigid-body velocity")
        bottom = position[2] - config["cube_size_m"] / 2 * np.abs(pose[2, :3]).sum()
        xy_extent = config["cube_size_m"] / 2 * np.abs(pose[:2, :3]).sum(axis=1)
        on_support = bool(np.all(np.abs(position[:2] - config["support_center_world_m"][:2])
                                + xy_extent <= np.array(config["support_dimensions_m"][:2]) / 2))
        sample = {"time_s": time_s, "position_world_m": position.tolist(),
                  "rotation_wxyz": np.asarray(quaternion).tolist(),
                  "linear_velocity_m_s": linear.tolist(), "angular_velocity_rad_s": angular.tolist(),
                  "bottom_world_z_m": float(bottom)}
        self.samples.append(sample)
        good = (on_support and abs(bottom-support_top(config)) <= config["support_height_tolerance_m"]
                and np.linalg.norm(linear) <= config["settle_linear_speed_m_s"]
                and np.linalg.norm(angular) <= config["settle_angular_speed_rad_s"])
        if not good:
            self.window = []
            return False
        self.window.append(sample)
        cutoff = time_s - config["settle_window_s"]
        while len(self.window) > 1 and self.window[1]["time_s"] <= cutoff:
            self.window.pop(0)
        positions = np.array([s["position_world_m"] for s in self.window])
        quaternions = np.array([s["rotation_wxyz"] for s in self.window])
        angle = 2*np.arccos(np.clip(np.abs(quaternions @ quaternions[0]), 0, 1))
        return bool(time_s >= config["min_capture_time_s"]
                    and time_s-self.window[0]["time_s"] >= config["settle_window_s"]-1e-9
                    and np.max(np.linalg.norm(positions-positions[0], axis=1)) <= config["settle_position_drift_m"]
                    and np.max(angle) <= config["settle_angle_drift_rad"])
