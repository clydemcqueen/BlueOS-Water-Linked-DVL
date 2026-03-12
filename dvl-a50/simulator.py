import json
import math
import socket
import threading
import time

# pylint: disable=too-many-instance-attributes


class SimulatedMavlinkHelper:
    def __init__(self):
        self.start_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alt = 0.0
        self.heading = 0.0

    def get(self, path, _vehicle=None, _component=None):
        if path == "/HEARTBEAT":
            return True
        if path == "/ATTITUDE/message":
            return json.dumps({"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw})
        if path == "/VFR_HUD/message/alt":
            return str(self.alt)
        if path == "/VFR_HUD/heading":
            return str(math.degrees(self.yaw))
        if path == "/GPS_GLOBAL_ORIGIN/message":
            return json.dumps(
                {"time_usec": int(time.time() * 1e6), "latitude": int(47.6 * 1e7), "longitude": int(-122.3 * 1e7)}
            )
        return None

    def get_float(self, path, _vehicle=None, _component=None):
        res = self.get(path, _vehicle, _component)
        if res is None:
            return float("nan")
        if isinstance(res, str):
            try:
                return float(res)
            except ValueError:
                return float("nan")
        return float(res)

    def request_message(self, _msg_id):
        return True

    def ensure_message_frequency(self, _message_name, _msg_id, _frequency):
        return True

    def set_param(self, _param_name, _param_type, _param_value):
        return True

    def send_vision(self, position_deltas, rotation_deltas=(0, 0, 0), confidence=100, dt=125000):
        pass

    def send_vision_speed_estimate(self, speed_estimates):
        pass

    def send_vision_position_estimate(
        self, timestamp, position_estimates, attitude_estimates=(0, 0, 0), reset_counter=0
    ):
        pass

    def send_rangefinder(self, distance, sensor_id=0, orientation="MAV_SENSOR_ROTATION_PITCH_270"):
        pass

    def set_gps_origin(self, lat, lon):
        pass

    def send_statustext(self, text, severity="MAV_SEVERITY_EMERGENCY"):
        pass


class DvlSimulator:
    def __init__(self, port=16171):
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("127.0.0.1", self.port))
        self.server_socket.listen(1)
        self.client_socket = None
        self.running = False

        self.x = 0.0
        self.y = 0.0
        self.z = -5.0

        self.vx = 0.3
        self.vy = 0.1
        self.vz = 0.0

        self.base_roll = 0.0
        self.base_pitch = 0.0
        self.base_yaw = 0.0

        self.mav_helper = SimulatedMavlinkHelper()
        self.thread = threading.Thread(target=self._run, daemon=True)

        # Beam angles based on dvl.py
        # index 0: 135 deg yaw (rear-right)
        # index 1: 225 deg yaw (rear-left)
        # index 2: 315 (-45) deg yaw (front-left)
        # index 3: 45 deg yaw (front-right)
        self.beam_yaws = [135, 225, 315, 45]
        self.beam_pitch_down = 22.5

    def get_seafloor_depth(self, x, y):
        # Base depth is 15 meters
        # Slopes added with sine/cosine waves, varying from -30 to +30 degrees in both x and y
        max_slope = math.tan(math.radians(30))
        L = 5.0
        return 15.0 + max_slope * L * math.sin(x / L) + max_slope * L * math.sin(y / L)

    def cast_ray(self, start_p, direction):
        # start_p = (x, y, z)
        # direction = (dx, dy, dz) - assuming dz is positive downwards
        # we step along the ray until we hit the seafloor
        t = 0.0
        step = 0.1
        for _ in range(500):  # max 50m
            px = start_p[0] + direction[0] * t
            py = start_p[1] + direction[1] * t

            seafloor_z = self.get_seafloor_depth(px, py)

            # depth of the ray at this point
            depth_rov = -start_p[2]
            depth_ray = depth_rov + direction[2] * t

            if depth_ray >= seafloor_z:
                # intersection found!
                # refine with one step of linear interpolation
                t_prev = max(0, t - step)
                depth_prev = depth_rov + direction[2] * t_prev
                px_prev = start_p[0] + direction[0] * t_prev
                py_prev = start_p[1] + direction[1] * t_prev
                sf_prev = self.get_seafloor_depth(px_prev, py_prev)

                # linear interp:
                # value = depth_ray(t) - seafloor(t)
                val_curr = depth_ray - seafloor_z
                val_prev = depth_prev - sf_prev

                frac = val_prev / (val_prev - val_curr + 1e-6)
                t_exact = t_prev + step * frac
                return t_exact

            t += step

        return -1.0  # no intersection

    def rotate_vector(self, v, roll, pitch, yaw):
        # Simple implementation using standard rotation matrices in NED-like frame
        x, y, z = v
        # Roll
        y1 = y * math.cos(roll) - z * math.sin(roll)
        z1 = y * math.sin(roll) + z * math.cos(roll)
        # Pitch
        x2 = x * math.cos(pitch) + z1 * math.sin(pitch)
        z2 = -x * math.sin(pitch) + z1 * math.cos(pitch)
        # Yaw
        x3 = x2 * math.cos(yaw) - y1 * math.sin(yaw)
        y3 = x2 * math.sin(yaw) + y1 * math.cos(yaw)
        return x3, y3, z2

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        self.server_socket.close()
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except Exception:
                pass
        if self.thread.is_alive():
            self.thread.join()

    def _run(self):
        dt = 0.25  # 4 Hz
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                self.client_socket, _ = self.server_socket.accept()
            except socket.timeout:
                continue
            except Exception:
                break

            while self.running:
                # To move in a circle of diameter 10m (radius 5m):
                radius = 5.0
                speed = math.sqrt(self.vx**2 + self.vy**2)
                omega = speed / radius

                # Small oscillations for attitude, constant turn rate for yaw
                roll = self.base_roll + math.sin(time.time() * 0.5) * 0.05
                pitch = self.base_pitch + math.sin(time.time() * 0.3) * 0.05
                yaw = self.base_yaw + (time.time() * omega) % (2 * math.pi)

                # Update global simulation state
                vx_global = self.vx * math.cos(yaw) - self.vy * math.sin(yaw)
                vy_global = self.vx * math.sin(yaw) + self.vy * math.cos(yaw)

                self.x += vx_global * dt
                self.y += vy_global * dt
                self.z += self.vz * dt

                self.mav_helper.roll = roll
                self.mav_helper.pitch = pitch
                self.mav_helper.yaw = yaw
                self.mav_helper.alt = -self.z

                depth_rov = -self.z
                sf_center = self.get_seafloor_depth(self.x, self.y)
                center_alt = sf_center - depth_rov

                transducers = []
                valid_beams = 0
                for i in range(4):
                    beam_yaw = math.radians(self.beam_yaws[i])
                    beam_pitch = math.radians(self.beam_pitch_down)

                    # Beam vector in sensor frame
                    # Z is down, X is forward, Y is right
                    # Pitch down from vertical = 22.5 deg
                    dx = math.sin(beam_pitch) * math.cos(beam_yaw)
                    dy = math.sin(beam_pitch) * math.sin(beam_yaw)
                    dz = math.cos(beam_pitch)

                    vector = (dx, dy, dz)

                    # Rotate by ROV attitude
                    global_dir = self.rotate_vector(vector, roll, pitch, yaw)

                    # Cast ray
                    dist = self.cast_ray((self.x, self.y, self.z), global_dir)

                    beam_valid = dist > 0
                    if beam_valid:
                        valid_beams += 1

                    transducers.append(
                        {
                            "id": i,
                            "velocity": self.vx * vector[0] + self.vy * vector[1] + self.vz * vector[2],
                            "distance": dist if beam_valid else 0.0,
                            "rssi": -40,
                            "nsd": -80,
                            "beam_valid": beam_valid,
                        }
                    )

                data = {
                    "time": int(dt * 1000),
                    "vx": self.vx,
                    "vy": self.vy,
                    "vz": self.vz,
                    "fom": 0.01 if valid_beams >= 3 else 0.5,
                    "altitude": center_alt,
                    "velocity_valid": valid_beams >= 3,
                    "status": 0,
                    "format": "json",
                    "type": "velocity",
                    "transducers": transducers,
                }

                try:
                    cmd = json.dumps(data) + "\n"
                    self.client_socket.sendall(cmd.encode())
                    time.sleep(dt)
                except Exception:
                    break
