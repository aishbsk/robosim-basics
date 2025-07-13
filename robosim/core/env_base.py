"""RL environment loop abstraction, composing a simulation backend and a robot controller."""

from typing import Any, Dict, Tuple
import abc
import logging


class EnvBase(abc.ABC):
    def __init__(self, sim_backend_cls, robot_base_cls, config):
        self.cfg = config
        self.sim = sim_backend_cls(config.sim)
        self.robot = robot_base_cls(config.robot)
        self.sim.load_robot(self.robot.urdf_path)
        self.robot.load_into_sim(self.sim)

        self.episode_length = config.env.episode_length
        self.timestep = 0

    def reset(self):
        self.sim.reset()
        self.robot.load_into_sim(self.sim)
        self.robot.reset()
        self.timestep = 0
        obs = self.sim.get_observation()
        return obs

    def step(self, action: Any) -> Tuple[Dict, float, bool, Dict]:
        if not self._validate_action(action):
            raise ValueError(f"Invalid action provided: {action}")

        self.robot.apply_control(action)
        self.sim.step()

        try:
            obs = self.sim.get_observation()
        except Exception as e:
            logging.error(f"Simulation error: {e}")
            return {}, 0.0, True, {"error": str(e)}

        reward = self._compute_reward(obs)
        self.timestep += 1
        done = self.timestep >= self.episode_length

        early_termination = self._check_termination(obs)
        done = done or early_termination

        info = {
            "timestep": self.timestep,
            "early_termination": early_termination,
        }

        return obs, reward, done, info

    @abc.abstractmethod
    def _compute_reward(self, obs: Dict) -> float:
        raise NotImplementedError

    def _validate_action(self, action: Any) -> bool:
        return True

    def _check_termination(self, obs: Dict) -> bool:
        return False

    def render(self):
        self.sim.render()

    def close(self):
        self.sim.close()
