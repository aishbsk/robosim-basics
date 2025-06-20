import os
import math
from robosim.sim.pybullet_backend import PyBulletBackend

URDF_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__), "..", "robosim", "models", "simple_arm.urdf"
    )
)


def main():
    with PyBulletBackend(gui=True) as sim:
        sim.load_robot(urdf_path=URDF_PATH)
        for t in range(100):
            angle = math.sin(t * 0.05)
            sim.set_joint_positions([angle, -angle])
            sim.step()
            obs = sim.get_observation()
            print(f"[{t:03d}] q: {obs['joint_positions']}")


if __name__ == "__main__":
    main()
