# Implements PyBullet backend for RoboSim

import pybullet as p
import pybullet_data
import time
import os
import logging
from typing import Optional, Tuple, Dict, List


class PyBulletBackend:
    """A backend interface for PyBullet simulation."""

    def __init__(self, gui: bool = True, timestep: float = 1.0 / 240.0):
        """
        Initialize the backend with GUI or headless mode.

        Args:
            gui: If True, starts the simulator with GUI. Otherwise, runs headless.
            timestep: Simulation timestep in seconds.
        """
        self.gui = gui
        self.timestep = timestep
        self.client_id: Optional[int] = None
        self.robot_id: Optional[int] = None
        self.movable_joints: List[int] = []
        self._logger = logging.getLogger(self.__class__.__name__)
        self.connect()

    def __enter__(self):
        if not self.connect():
            raise RuntimeError("Failed to connect to PyBullet.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self) -> bool:
        """
        Connect to the PyBullet simulator.

        Returns:
            True if connection is successful, False otherwise.
        """
        try:
            self.client_id = p.connect(p.GUI if self.gui else p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
            return True
        except Exception as e:
            self._logger.error(f"Failed to connect to PyBullet: {e}")
            return False

    def load_robot(
        self,
        urdf_path: str,
        base_position: Tuple[float, float, float] = (0.0, 0.0, 0.1),
        use_fixed_base: bool = True,
    ) -> int:
        """
        Load a URDF robot model into the simulation.

        Args:
            urdf_path: Path to the URDF file.
            base_position: Initial position of the robot in the world.
            use_fixed_base: If True, robot base is immobile.

        Returns:
            Unique ID of the loaded robot.
        """
        full_urdf_path = (
            pybullet_data.getDataPath() + "/" + urdf_path
            if not os.path.isabs(urdf_path)
            else urdf_path
        )
        if not os.path.isfile(full_urdf_path):
            self._logger.error(f"URDF file does not exist: {urdf_path}")
            raise FileNotFoundError(f"URDF file does not exist: {urdf_path}")
        if self.client_id is None:
            raise RuntimeError("Not connected to PyBullet.")
        self.robot_id = p.loadURDF(
            full_urdf_path,
            base_position,
            useFixedBase=use_fixed_base,
            physicsClientId=self.client_id,
        )
        # Identify movable joints (revolute or prismatic)
        self.movable_joints = [
            i
            for i in range(
                p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
            )
            if p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)[2]
            in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]
        ]
        if self.robot_id is None:
            raise RuntimeError("Failed to load robot: robot_id is None.")
        return self.robot_id

    def step(self):
        """
        Advance the simulation by one time step.
        """
        if self.client_id is not None:
            p.stepSimulation(physicsClientId=self.client_id)
            time.sleep(self.timestep)
        else:
            self._logger.warning("step() called but client_id is None.")

    def get_observation(self) -> Dict[str, List[float]]:
        """
        Get current joint state of the robot.

        Returns:
            Dictionary with joint positions and velocities.
        """
        if self.robot_id is None:
            raise RuntimeError("Robot not loaded.")
        if self.client_id is None:
            raise RuntimeError("Not connected to PyBullet.")
        joint_states = p.getJointStates(
            self.robot_id, self.movable_joints, physicsClientId=self.client_id
        )
        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        return {
            "joint_positions": positions,
            "joint_velocities": velocities,
        }

    def set_joint_positions(self, positions: List[float]):
        """
        Set the joint positions of the robot.

        Args:
            positions: List of joint positions to set.
        """
        if self.robot_id is None or self.client_id is None:
            raise RuntimeError("Robot not loaded or not connected.")
        if len(positions) != len(self.movable_joints):
            raise ValueError(
                f"Length of positions ({len(positions)}) does not match number of movable joints ({len(self.movable_joints)})."
            )
        for i, pos in zip(self.movable_joints, positions):
            p.resetJointState(self.robot_id, i, pos, physicsClientId=self.client_id)

    def reset(self):
        """
        Reset the simulation and robot state.
        """
        if self.client_id is not None:
            p.resetSimulation(physicsClientId=self.client_id)
            p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
            self.robot_id = None
        else:
            self._logger.warning("reset() called but client_id is None.")

    def disconnect(self):
        """
        Cleanly disconnect from the simulator.
        """
        if self.client_id is not None:
            p.disconnect(physicsClientId=self.client_id)
            self.client_id = None
            self.robot_id = None
        else:
            self._logger.info("disconnect() called but client_id is already None.")

    def get_movable_joints(self, robot_id):
        num_joints = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        movable_joints = []
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i, physicsClientId=self.client_id)
            joint_type = joint_info[2]
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                movable_joints.append(i)
        return movable_joints

    def reset_joint_state(self, robot_id, joint_index, position, velocity):
        """
        Reset the state of a single joint (position and velocity).

        Args:
            robot_id: The unique ID of the robot.
            joint_index: Index of the joint to reset.
            position: Desired joint position.
            velocity: Desired joint velocity.
        """
        p.resetJointState(
            robot_id, joint_index, position, velocity, physicsClientId=self.client_id
        )

    def set_joint_motor_control(
        self, robot_id, joint_index, control_mode="position", target=0.0
    ):
        """
        Apply motor control to a joint.

        Args:
            robot_id: ID of the robot.
            joint_index: Index of the joint to control.
            control_mode: One of ["position", "velocity", "torque"].
            target: Target value (position/velocity/torque depending on mode).
        """
        mode_map = {
            "position": p.POSITION_CONTROL,
            "velocity": p.VELOCITY_CONTROL,
            "torque": p.TORQUE_CONTROL,
        }

        if control_mode not in mode_map:
            raise ValueError(f"Unsupported control mode: {control_mode}")

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=mode_map[control_mode],
            targetPosition=target if control_mode == "position" else 0.0,
            targetVelocity=target if control_mode == "velocity" else 0.0,
            force=500.0,  # you can parameterize this later
            physicsClientId=self.client_id,
        )

    def render(self):
        # In GUI mode, rendering happens automatically; in DIRECT mode, this could be a no-op or support camera rendering
        pass

    def close(self):
        """
        Cleanly close the simulation.
        """
        if self.client_id is not None:
            p.disconnect(physicsClientId=self.client_id)
            self.client_id = None
            self.robot_id = None
        else:
            self._logger.info("close() called but client_id is already None.")
