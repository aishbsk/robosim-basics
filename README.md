# robosim-core

robosim-core is a lightweight simulation framework to help support fast reproducible and iteratable experimentations in robotic policy development. It abstracts PyBullet and Isaac Gym and includes configurable control loops and robot models. It also supports benchmarks for sim fidelity and performance.


### Pybullet Usage
`python scripts/rollout.py env=arm env.sim=pybullet robot.name=simple_arm`