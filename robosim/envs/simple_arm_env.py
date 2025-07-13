from robosim.core.env_base import EnvBase
from robosim.sim.pybullet_backend import PyBulletBackend
from robosim.models.simple_arm import SimpleArm


def make_env(cfg):
    sim_cls_map = {
        "pybullet": PyBulletBackend,
    }

    robot_cls_map = {
        "simple_arm": SimpleArm,
    }

    sim_cls = sim_cls_map[cfg.env.sim]
    robot_cls = robot_cls_map[cfg.robot.name]

    class SimpleArmEnv(EnvBase):
        def _compute_reward(self, obs):  # simple placeholder reward
            return 0.0

    return SimpleArmEnv(sim_cls, robot_cls, cfg)
