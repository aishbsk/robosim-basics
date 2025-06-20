import os
import pytest
from robosim.sim.pybullet_backend import PyBulletBackend
import pybullet_data


@pytest.fixture(scope="module")
def urdf_path():
    # Use a standard URDF from pybullet_data
    return os.path.join(pybullet_data.getDataPath(), "r2d2.urdf")


@pytest.fixture
def backend():
    backend = PyBulletBackend(gui=False)
    backend.connect()
    yield backend
    backend.disconnect()


def test_connect_and_disconnect():
    backend = PyBulletBackend(gui=False)
    assert backend.connect() is True
    assert backend.client_id is not None
    backend.disconnect()
    assert backend.client_id is None


def test_load_robot_and_observation(backend, urdf_path):
    robot_id = backend.load_robot(urdf_path)
    assert robot_id is not None
    obs = backend.get_observation()
    assert "joint_positions" in obs
    assert "joint_velocities" in obs
    assert isinstance(obs["joint_positions"], list)
    assert isinstance(obs["joint_velocities"], list)


def test_set_joint_positions(backend, urdf_path):
    backend.load_robot(urdf_path)
    obs = backend.get_observation()
    num_joints = len(obs["joint_positions"])
    new_positions = [0.1] * num_joints
    backend.set_joint_positions(new_positions)
    obs2 = backend.get_observation()
    assert all(abs(p - 0.1) < 1e-6 for p in obs2["joint_positions"])


def test_step_and_reset(backend, urdf_path):
    backend.load_robot(urdf_path)
    obs_before = backend.get_observation()
    backend.step()
    obs_after = backend.get_observation()
    # After one step, positions should not change for fixed base and no control
    assert obs_before["joint_positions"] == obs_after["joint_positions"]
    backend.reset()
    assert backend.robot_id is None


def test_load_robot_invalid_path(backend):
    with pytest.raises(FileNotFoundError):
        backend.load_robot("/invalid/path/to/robot.urdf")


def test_get_observation_without_robot(backend):
    backend.reset()
    with pytest.raises(RuntimeError):
        backend.get_observation()


def test_set_joint_positions_invalid_length(backend, urdf_path):
    backend.load_robot(urdf_path)
    with pytest.raises(ValueError):
        backend.set_joint_positions([0.1, 0.2])
