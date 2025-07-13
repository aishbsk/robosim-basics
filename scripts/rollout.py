from robosim.envs.simple_arm_env import make_env
from omegaconf import OmegaConf
import time

cfg = OmegaConf.load("config/base.yaml")
env = make_env(cfg)

obs = env.reset()
for t in range(cfg.env.episode_length):
    action = [0.1] * len(obs["joint_positions"])
    obs, reward, done, info = env.step(action)
    env.render()
    time.sleep(cfg.sim.timestep)
    print(f"[{t:03d}] q: {obs['joint_positions']} reward: {reward}")
    if done:
        break

env.close()
