"""Abstract base class for robots"""

import abc
from typing import Dict, List, Tuple, Optional


class RobotBase(abc.ABC):
    def __init__(
        self,
        robot_name: str,
        num_joints: int,
        joint_names: List[str],
        joint_limits: Dict[str, Tuple[float, float]],
        end_effector_link: str,
    ):
        """Initialize robot base class.
        Args:
        robot_name: Unique string identifer for the robot
        num_joints: Number of actuated degrees of freedom
        joint_names: List of ordered joint names
        joint_limits: Dictionary mappings of the joint names to (min, max) joint angle limits
        end_effector_link: Name or ID of the end effector link
        """
        if num_joints <= 0:
            raise ValueError("Number of joints provided must be positive")
        if len(joint_names) != num_joints:
            raise ValueError("Length of joint names does not match number of joints")
        if any(name not in joint_limits for name in joint_names):
            raise ValueError("Joint limits must be declared for all joints")

        self.robot_name = robot_name
        self.num_joints = num_joints
        self.joint_names = joint_names
        self.joint_limits = joint_limits
        self.end_effector_link = end_effector_link

    @abc.abstractmethod
    def get_joint_state(self):
        """Returns the current position, velocity and effort of all joints.

        Returns: Dict[str, Dict[str, float]]: Maps joint names to their
                    position, velocity, and effort.
        """
        raise NotImplementedError("Subclass must implement get_joint_state method")

    @abc.abstractmethod
    def get_base_pose(self):
        """Returns the current position, and orientation of the root link"""
        raise NotImplementedError("Subclass must implement get_base_pose method")

    @abc.abstractmethod
    def get_end_effector_pose(self):
        """Returns the 3D pose of the robot's tip (position and orientation)"""
        raise NotImplementedError(
            "Subclass must implement get_end_effector_pose method"
        )

    def get_link_state(self, link_name: str):
        """Returns the pose or velocity of any arb link"""
        raise NotImplementedError("Subclass must implement get_link_state method")

    @abc.abstractmethod
    def reset_joint_state(self, joint_positions: Dict[str, float]):
        """Resets the joint state to the specified positions.

        Args:
        joint_positions: Dictionary mapping joint names to desired positions
        """
        raise NotImplementedError("Subclass must implement reset_joint_state method")
