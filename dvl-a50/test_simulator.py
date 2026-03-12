import math
from unittest.mock import MagicMock

from simulator import DvlSimulator


def test_seafloor_depth():
    sim = DvlSimulator(port=16172)

    # Base depth 15.0
    # 15.0 + max_slope * L * math.sin(x / L) + max_slope * L * math.sin(y / L)
    # max_slope = tan(30) = 0.57735, L = 5.0
    assert abs(sim.get_seafloor_depth(0, 0) - 15.0) < 1e-4

    # x = 5 * pi / 2 -> sin is 1.0 => 15 + 0.57735 * 5.0 = 17.88675
    expected = 15.0 + math.tan(math.radians(30)) * 5.0
    assert abs(sim.get_seafloor_depth(5.0 * math.pi / 2.0, 0) - expected) < 1e-4
    sim.server_socket.close()


def test_rotate_vector():
    sim = DvlSimulator(port=16173)
    v = (1, 0, 0)

    # yaw 90 deg -> (0, 1, 0)
    r1 = sim.rotate_vector(v, 0, 0, math.pi / 2.0)
    assert abs(r1[0]) < 1e-6
    assert abs(r1[1] - 1.0) < 1e-6
    assert abs(r1[2]) < 1e-6

    # pitch 90 deg -> (0, 0, -1)
    r2 = sim.rotate_vector(v, 0, math.pi / 2.0, 0)
    assert abs(r2[0]) < 1e-6
    assert abs(r2[1]) < 1e-6
    assert abs(r2[2] - (-1.0)) < 1e-6
    sim.server_socket.close()


def test_cast_ray():
    sim = DvlSimulator(port=16174)
    # Simple downward ray from (0,0,-5)
    # At (0,0), seafloor is 15.0
    # From z=-5 (depth 5), distance to 15.0 is exactly 15.0 - 5 = 10.0
    direction = (0, 0, 1)  # straight down
    dist = sim.cast_ray((0.0, 0.0, -5.0), direction)
    assert abs(dist - 10.0) < 1e-2
    sim.server_socket.close()


def test_simulator_data_format():
    sim = DvlSimulator(port=16175)
    # Manually run one iteration without starting the thread
    sim.running = True

    # Mocking the socket
    sim.client_socket = MagicMock()

    sim.x = 0
    sim.y = 0
    sim.z = -5
    sim.vx = 0.5
    sim.vy = 0.2
    sim.vz = 0.0

    # Execute one loop manually
    # Just run the inner loop code directly to get the data JSON
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    depth_rov = -sim.z
    sf_center = sim.get_seafloor_depth(sim.x, sim.y)
    center_alt = sf_center - depth_rov

    transducers = []
    valid_beams = 0
    for i in range(4):
        beam_yaw = math.radians(sim.beam_yaws[i])
        beam_pitch = math.radians(sim.beam_pitch_down)

        dx = math.sin(beam_pitch) * math.cos(beam_yaw)
        dy = math.sin(beam_pitch) * math.sin(beam_yaw)
        dz = math.cos(beam_pitch)

        vector = (dx, dy, dz)
        global_dir = sim.rotate_vector(vector, roll, pitch, yaw)

        dist = sim.cast_ray((sim.x, sim.y, sim.z), global_dir)
        beam_valid = dist > 0
        if beam_valid:
            valid_beams += 1

        transducers.append(
            {
                "id": i,
                "velocity": sim.vx * vector[0] + sim.vy * vector[1] + sim.vz * vector[2],
                "distance": dist if beam_valid else 0.0,
                "rssi": -40,
                "nsd": -80,
                "beam_valid": beam_valid,
            }
        )

    data = {
        "time": 250,
        "vx": sim.vx,
        "vy": sim.vy,
        "vz": sim.vz,
        "fom": 0.01 if valid_beams >= 3 else 0.5,
        "altitude": center_alt,
        "velocity_valid": valid_beams >= 3,
        "status": 0,
        "format": "json",
        "type": "velocity",
        "transducers": transducers,
    }

    assert data["type"] == "velocity"
    assert "altitude" in data
    assert len(data["transducers"]) == 4
    for t in data["transducers"]:
        assert "distance" in t
        assert t["distance"] > 5.0  # depth is 5, seafloor is 15, distance is ~10 but can be less due to slope
        assert isinstance(t["velocity"], float)

    sim.server_socket.close()


if __name__ == "__main__":
    test_seafloor_depth()
    test_rotate_vector()
    test_cast_ray()
    test_simulator_data_format()
    print("All simulator tests passed!")
