from robosim.core.env_base import EnvBase


class DummySim:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset_called = False
        self.step_called = False
        self.render_called = False
        self.robot_loaded = False
        self.obs = {"state": 0}

    def load_robot(self, robot):
        self.robot_loaded = True

    def reset(self):
        self.reset_called = True

    def step(self):
        self.step_called = True

    def get_observation(self):
        return self.obs

    def render(self):
        self.render_called = True


class DummyRobot:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset_called = False
        self.apply_control_called = False
        self.last_action = None

    def reset(self):
        self.reset_called = True

    def apply_control(self, action):
        self.apply_control_called = True
        self.last_action = action


class DummyConfig:
    def __init__(self):
        class Sim:
            episode_length = 5

        class Robot:
            pass

        class Env:
            episode_length = 5

        self.sim = Sim()
        self.robot = Robot()
        self.env = Env()


class DummyEnv(EnvBase):
    def _compute_reward(self, obs):
        return 1.0

    def _validate_action(self, action):
        return True


def test_envbase_reset_and_step():
    env = DummyEnv(DummySim, DummyRobot, DummyConfig())
    obs = env.reset()
    assert env.sim.reset_called
    assert env.robot.reset_called
    assert env.timestep == 0
    assert obs == {"state": 0}

    obs, reward, done, info = env.step(action=None)
    assert env.sim.step_called
    assert reward == 1.0
    assert not done
    assert info["timestep"] == 1

    # Step until done
    for _ in range(4):
        obs, reward, done, info = env.step(action=None)
    assert done


def test_envbase_render():
    env = DummyEnv(DummySim, DummyRobot, DummyConfig())
    env.render()
    assert env.sim.render_called
