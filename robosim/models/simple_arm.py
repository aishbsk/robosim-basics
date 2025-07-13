import os
import numpy as np
from robosim.core.robot_base import RobotBase


class SimpleArm(RobotBase):
    def __init__(self, cfg):
        self.cfg = cfg
        self.urdf_path = os.path.abspath(cfg.urdf_path)
        self.start_pos = cfg.get("start_pos", [0, 0, 0.1])
        self.robot_id = None
        self.joint_indices = []

    def reset(self):
        if self.robot_id is None:
            raise RuntimeError("Robot must be loaded into sim before calling reset.")

        for joint_index in self.joint_indices:
            print(f"[reset] Robot ID: {self.robot_id}, Joint index: {joint_index}")
            self.backend.reset_joint_state(self.robot_id, joint_index, 0.0, 0.0)

    def apply_control(self, control):
        for idx, joint_index in enumerate(self.joint_indices):
            target = control[idx] if idx < len(control) else 0.0
            self.backend.set_joint_motor_control(
                self.robot_id, joint_index, control_mode="position", target=target
            )

    def get_joint_state(self):
        joint_states = [
            self.backend.get_joint_state(self.robot_id, idx)
            for idx in self.joint_indices
        ]
        positions, velocities = zip(*joint_states)
        return {"positions": np.array(positions), "velocities": np.array(velocities)}

    def get_end_effector_pose(self):
        if not self.joint_indices:
            return None
        ee_link = self.joint_indices[-1]
        pos, ori = self.backend.get_link_pose(self.robot_id, ee_link)
        return {"position": pos, "orientation": ori}

    def load_into_sim(self, backend):
        self.backend = backend
        self.robot_id = backend.load_robot(self.urdf_path, base_position=self.start_pos)
        self.joint_indices = backend.get_movable_joints(self.robot_id)

    def get_base_pose(self):
        # Get the pose of the robot's base link (usually link 0)
        pos, ori = self.backend.get_link_pose(self.robot_id, 0)
        return {"position": pos, "orientation": ori}

    def reset_joint_state(self, joint_positions):
        # joint_positions: dict mapping joint names to positions
        for name, pos in joint_positions.items():
            if hasattr(self, "joint_names") and name in self.joint_names:
                idx = self.joint_names.index(name)
                joint_index = self.joint_indices[idx]
                self.backend.reset_joint_state(self.robot_id, joint_index, pos, 0.0)
