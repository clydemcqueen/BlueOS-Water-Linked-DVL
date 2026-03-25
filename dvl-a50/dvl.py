"""
Code for integration of Water Linked DVL A50/A125 with BlueOS and ArduSub
"""

import csv
import datetime
import json
import math
import os
import pathlib
import socket
import threading
import time
from enum import Enum
from select import select
from typing import Any, Dict, List

import numpy as np
from loguru import logger

from blueoshelper import request
from dvlfinder import find_the_dvl
from mavlink2resthelper import GPS_GLOBAL_ORIGIN_ID, Mavlink2RestHelper
from terrain_ekf import EKFParams, TerrainEKF

HOSTNAME = "waterlinked-dvl.local"
DVL_DOWN = 1
DVL_FORWARD = 2
LATLON_TO_CM = 1.1131884502145034e5


class DVLModelA50:
    def get_beam_vectors_body(self):
        pitch_rad = math.radians(22.5)
        # 0: rear-right (135), 1: rear-left (225), 2: front-left (315), 3: front-right (45)
        yaws = [135, 225, 315, 45]
        vectors = []
        for yaw in yaws:
            yaw_rad = math.radians(yaw)
            vx = math.sin(pitch_rad) * math.cos(yaw_rad)
            vy = math.sin(pitch_rad) * math.sin(yaw_rad)
            vz = math.cos(pitch_rad)
            vectors.append(np.array([vx, vy, vz]))
        return vectors


class MessageType(str, Enum):
    POSITION_DELTA = "POSITION_DELTA"
    POSITION_ESTIMATE = "POSITION_ESTIMATE"
    SPEED_ESTIMATE = "SPEED_ESTIMATE"

    @staticmethod
    def contains(value):
        return value in set(item.value for item in MessageType)


class EKFLogger:
    """
    Write detailed logs to a CSV file for analysis.
    """

    @staticmethod
    def get_log_path() -> str:
        """
        Generate a log path in the standard BlueOS format: /data/logs/YYYY-MM-DD/YYYY-MM-DD_HH-MM-SS_ekf.csv
        """
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")
        local_test = os.environ.get("DVL_TEST_MODE", "false").lower() == "true"
        dir_str = os.path.join("/tmp", "dvl_test", date_str) if local_test else os.path.join("/data", "logs", date_str)
        pathlib.Path(dir_str).mkdir(parents=True, exist_ok=True)
        return os.path.join(dir_str, f"{date_str}_{time_str}_ekf.csv")

    def __init__(self) -> None:
        self.ekf_log_file = None
        self.ekf_csv_writer = None
        try:
            log_path = self.get_log_path()
            # pylint: disable=consider-using-with
            self.ekf_log_file = open(log_path, mode="a", newline="", encoding="utf-8")
            self.ekf_csv_writer = csv.writer(self.ekf_log_file)
            self.ekf_csv_writer.writerow(
                [
                    "timestamp",
                    "beam_ar_range",  # aft-right
                    "beam_al_range",  # aft-left
                    "beam_fl_range",  # forward-left
                    "beam_fr_range",  # forward-right
                    "vn",
                    "ve",
                    "vd",
                    "rov_d",
                    "roll",
                    "pitch",
                    "yaw",
                    "ekf_terrain_d",
                    "ekf_slope_n",
                    "ekf_slope_e",
                    "proj_terrain_d",
                    "proj_alt",
                    "proj_rangefinder",
                    "beam_ar_status",
                    "beam_al_status",
                    "beam_fl_status",
                    "beam_fr_status",
                ]
            )
            logger.info(f"Started EKF logging to {log_path}")
        except Exception as e:
            logger.warning(f"Could not open EKF log file {log_path}: {e}")

    def log(self, data: List[Any]) -> None:
        """
        Write a row of data to the EKF log file.
        """
        if self.ekf_csv_writer and self.ekf_log_file:
            try:
                self.ekf_csv_writer.writerow(data)
                self.ekf_log_file.flush()
            except Exception as e:
                logger.warning(f"Could not write to EKF log: {e}")

    def stop(self) -> None:
        """
        Close the EKF csv file if it's open.
        """
        if self.ekf_log_file:
            try:
                self.ekf_log_file.close()
            except Exception as e:
                logger.warning(f"Error closing EKF log file: {e}")
            self.ekf_log_file = None
            self.ekf_csv_writer = None

    def __del__(self) -> None:
        self.stop()


# pylint: disable=too-many-instance-attributes
# pylint: disable=unspecified-encoding
# pylint: disable=too-many-branches
# pylint: disable=too-many-statements
class DvlDriver(threading.Thread):
    """
    Responsible for the DVL interactions themselves.
    This handles fetching the DVL data and forwarding it to Ardusub
    """

    status = "Starting"
    version = ""
    mav = Mavlink2RestHelper()
    socket = None
    port = 16171  # Water Linked mentioned they won't allow changing or disabling this
    orientation = DVL_DOWN
    enabled = True
    rangefinder = True
    hostname = os.environ.get("DVL_HOSTNAME", HOSTNAME)
    timeout = 3  # tcp timeout in seconds
    origin = [0, 0]
    saved_settings = [
        "enabled",
        "orientation",
        "hostname",
        "origin",
        "rangefinder",
        "should_send",
        "send_ekf_output",
        "ekf_sensor_delay",
        "ekf_terrain_variance",
        "ekf_slope_variance",
        "ekf_terrain_process_noise",
        "ekf_slope_process_noise",
        "ekf_gate_threshold",
    ]

    send_ekf_output = False
    ekf_enabled = True
    ekf_sensor_delay = 0.2
    ekf_terrain_variance = 100.0
    ekf_slope_variance = 1.0
    ekf_terrain_process_noise = 0.01
    ekf_slope_process_noise = 0.1
    ekf_gate_threshold = 9.0

    ekf = None
    ekf_logger = None

    ekf_state_terrain_d = 0.0
    ekf_state_slope_n = 0.0
    ekf_state_slope_e = 0.0
    ekf_projected_terrain_d = 0.0
    ekf_projected_alt = 0.0
    ekf_projected_rangefinder = 0.0

    settings_path = os.path.join(os.path.expanduser("~"), ".config", "dvl", "settings.json")

    should_send = MessageType.POSITION_DELTA
    reset_counter = 0
    timestamp = 0
    last_temperature_check_time = 0
    temperature_check_interval_s = 30
    temperature_too_hot = 45

    # Status tracking for individual beam distances
    last_beam_distances = [0, 0, 0, 0]
    last_beam_valid = [False, False, False, False]

    def __init__(self, orientation=DVL_DOWN) -> None:
        threading.Thread.__init__(self)
        self.daemon = True
        self.orientation = orientation
        # used for calculating attitude delta
        self.last_attitude = (0, 0, 0)
        self.current_attitude = (0, 0, 0)

    def report_status(self, msg: str) -> None:
        self.status = msg
        logger.debug(msg)

    def reset_ekf(self):
        if self.ekf is None:
            return

        logger.info("Resetting TerrainEKF...")

        self.ekf = None
        self.ekf_logger = None

    def load_settings(self) -> None:
        """
        Load settings from .config/dvl/settings.json
        """
        try:
            with open(self.settings_path) as settings:
                data = json.load(settings)
                for setting_name in self.saved_settings:
                    if setting_name in data:
                        setattr(self, setting_name, data[setting_name])
                    else:
                        default = getattr(self, setting_name)
                        logger.warning(f"key not found: {setting_name} - keeping {default=} instead:")
                logger.debug("Loaded settings: ", data)
        except FileNotFoundError:
            logger.warning("Settings file not found, using default.")
        except ValueError:
            logger.warning("File corrupted, using default settings.")

        env_hostname = os.environ.get("DVL_HOSTNAME")
        if env_hostname:
            self.hostname = env_hostname
        self.reset_ekf()

    @property
    def current_settings(self):
        return {setting_name: getattr(self, setting_name) for setting_name in self.saved_settings}

    def save_settings(self) -> None:
        """
        Load settings from .config/dvl/settings.json
        """

        def ensure_dir(file_path) -> None:
            """
            Helper to guarantee that the file path exists
            """
            directory = os.path.dirname(file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)

        ensure_dir(self.settings_path)
        with open(self.settings_path, "w") as settings:
            settings.write(json.dumps(self.current_settings))

    def get_status(self) -> dict:
        """
        Returns a dict with the current status
        """
        return {
            "status": self.status,
            **self.current_settings,
            "ekf_enabled": self.send_ekf_output,
            "beam_distances": self.last_beam_distances,
            "beam_valid": self.last_beam_valid,
            "ekf_state_terrain_d": self.ekf_state_terrain_d,
            "ekf_projected_terrain_d": self.ekf_projected_terrain_d,
            "ekf_state_slope_n": self.ekf_state_slope_n,
            "ekf_state_slope_e": self.ekf_state_slope_e,
            "ekf_projected_alt": self.ekf_projected_alt,
            "ekf_projected_rangefinder": self.ekf_projected_rangefinder,
        }

    @property
    def host(self) -> str:
        """Make sure there is no port in the hostname allows local testing by where http can be running on other ports than 80"""
        try:
            host = self.hostname.split(":")[0]
        except IndexError:
            host = self.hostname
        return host

    def look_for_dvl(self):
        """
        Waits for the dvl to show up at the designated hostname
        """
        self.wait_for_cable_guy()
        ip = self.hostname
        self.report_status(f"Trying to talk to dvl at http://{ip}/api/v1/about")

        # In test mode, skip the DVL discovery and try to connect directly
        if os.environ.get("DVL_TEST_MODE", "false").lower() == "true":
            logger.info(f"Test mode: Attempting direct connection to {ip}")
            return

        while "DVL not found":
            if request(f"http://{ip}/api/v1/about"):
                self.report_status(f"DVL found at {ip}, using it.")
                return
            self.report_status(f"Could not talk to dvl at {ip}, looking for it in the local network...")
            try:
                found_dvl = find_the_dvl(report_status=self.report_status)
                if found_dvl is not None:
                    self.report_status(f"Dvl found at address {found_dvl}, using it instead.")
                    self.hostname = found_dvl
                    self.save_settings()
                    return
            except Exception as e:
                self.report_status(f"Unable to find dvl: {e}")
            time.sleep(1)

    def wait_for_cable_guy(self):
        # Skip cable-guy check if running in test mode
        if os.environ.get("DVL_TEST_MODE", "false").lower() == "true":
            logger.info("Running in test mode, skipping cable-guy check")
            return
        while not request("http://host.docker.internal/cable-guy/v1.0/ethernet"):
            self.report_status("waiting for cable-guy to come online...")
            time.sleep(1)

    def wait_for_vehicle(self):
        """
        Waits for a valid heartbeat to Mavlink2Rest
        """
        # Skip vehicle check if running in test mode
        if os.environ.get("DVL_TEST_MODE", "false").lower() == "true":
            logger.info("Running in test mode, skipping vehicle heartbeat check")
            return
        self.report_status("Waiting for vehicle...")
        while not self.mav.get("/HEARTBEAT"):
            time.sleep(1)

    def set_orientation(self, orientation: int) -> bool:
        """
        Sets the DVL orientation, either DVL_FORWARD of DVL_DOWN
        """
        if orientation in [DVL_FORWARD, DVL_DOWN]:
            self.orientation = orientation
            self.save_settings()
            return True
        return False

    def set_should_send(self, should_send):
        if not MessageType.contains(should_send):
            raise ValueError(f"bad messagetype: {should_send}")
        self.should_send = should_send
        self.save_settings()

    @staticmethod
    def longitude_scale(lat: float):
        """
        from https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L325
        """
        scale = math.cos(math.radians(lat))
        return max(scale, 0.01)

    def lat_lng_to_NE_XY_cm(self, lat: float, lon: float) -> List[float]:
        """
        From https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L206
        """
        x = (lat - self.origin[0]) * LATLON_TO_CM
        y = self.longitude_scale((lat + self.origin[0]) / 2) * LATLON_TO_CM * (lon - self.origin[1])
        return [x, y]

    def has_origin_set(self) -> bool:
        try:
            old_time = self.mav.get_float("/GPS_GLOBAL_ORIGIN/message/time_usec")
            if math.isnan(old_time):
                logger.warning("Unable to read current time for GPS_GLOBAL_ORIGIN, using 0")
                old_time = 0
        except Exception as e:
            logger.warning(f"Unable to read current time for GPS_GLOBAL_ORIGIN, using 0: {e}")
            old_time = 0

        for attempt in range(5):
            logger.debug(f"Trying to read origin, try # {attempt}")
            self.mav.request_message(GPS_GLOBAL_ORIGIN_ID)
            time.sleep(0.5)  # make this a timeout?
            try:
                new_origin_data = json.loads(self.mav.get("/GPS_GLOBAL_ORIGIN/message"))
                if new_origin_data["time_usec"] != old_time:
                    self.origin = [new_origin_data["latitude"] * 1e-7, new_origin_data["longitude"] * 1e-7]
                    return True
                continue  # try again
            except Exception as e:
                logger.warning(e)
                return False
        return False

    def set_current_position(self, lat: float, lon: float):
        """
        Sets the EKF origin to lat, lon
        """
        # If origin has never been set, set it
        if not self.has_origin_set():
            logger.info("Origin was never set, trying to set it.")
            self.set_gps_origin(lat, lon)
        else:
            logger.info("Origin has already been set, sending POSITION_ESTIMATE instead")
            # if we already have an origin set, send a new position instead
            x, y = self.lat_lng_to_NE_XY_cm(lat, lon)
            depth = float(self.mav.get("/VFR_HUD/message/alt"))

            attitude = json.loads(self.mav.get("/ATTITUDE/message"))
            # code expects degrees, but the ATTITUDE message gives radians
            attitudes = [math.degrees(attitude[axis]) for axis in ("roll", "pitch", "yaw")]
            positions = [x, y, -depth]
            self.reset_counter += 1
            self.mav.send_vision_position_estimate(
                self.timestamp, positions, attitudes, reset_counter=self.reset_counter
            )

    def set_gps_origin(self, lat: float, lon: float) -> None:
        """
        Sets the EKF origin to lat, lon
        """
        self.mav.set_gps_origin(lat, lon)
        self.origin = [float(lat), float(lon)]
        self.save_settings()

    def set_enabled(self, enable: bool) -> bool:
        """
        Enables/disables the driver
        """
        self.enabled = enable
        self.save_settings()
        return True

    def set_use_as_rangefinder(self, enable: bool) -> bool:
        """
        Enables/disables DISTANCE_SENSOR messages
        """
        self.rangefinder = enable
        self.save_settings()
        if enable:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10)  # MAVLINK
        return True

    def set_ekf_enabled(self, enable: bool) -> bool:
        self.send_ekf_output = enable
        self.save_settings()
        return True

    def set_ekf_params(self, params: EKFParams) -> bool:
        self.ekf_sensor_delay = params.delay
        self.ekf_terrain_variance = params.t_var
        self.ekf_slope_variance = params.s_var
        self.ekf_terrain_process_noise = params.t_noise
        self.ekf_slope_process_noise = params.s_noise
        self.ekf_gate_threshold = params.gate
        self.save_settings()
        self.reset_ekf()
        return True

    def load_params(self, selector: str) -> bool:
        """
        Load EK3_SRC1 parameters to match the use case:
        "dvl"       The DVL will be used for horizontal position and velocity
        "dvl_gps"   The GPS will be used for horizontal position, and the DVL will be used for horizontal velocity
        """
        if selector == "dvl":
            self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 3)  # Disable
            self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)  # BARO
            return True
        if selector == "dvl_gps":
            self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 0)  # Enable
            self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 3)  # GPS
            self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
            self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)  # BARO
            return True
        return False

    def setup_mavlink(self) -> None:
        """
        Sets up mavlink streamrates so we have the needed messages at the
        appropriate rates
        """
        self.report_status("Setting up MAVLink streams...")
        self.mav.ensure_message_frequency("ATTITUDE", 30, 5)
        self.mav.ensure_message_frequency("VFR_HUD", 74, 5)

    def setup_params(self) -> None:
        """
        Sets up the required params for DVL integration -- but leave the EK3_SRC1 params alone
        """
        self.mav.set_param("AHRS_EKF_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        # TODO: Check if really required. It doesn't look like the ekf2 stops at all
        self.mav.set_param("EK2_ENABLE", "MAV_PARAM_TYPE_UINT8", 0)

        self.mav.set_param("EK3_ENABLE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("VISO_TYPE", "MAV_PARAM_TYPE_UINT8", 1)
        if self.rangefinder:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10)  # MAVLINK

    def setup_connections(self, timeout=300) -> None:
        """
        Sets up the socket to talk to the DVL
        """
        while timeout > 0:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                self.socket.setblocking(0)
                return True
            except socket.error:
                time.sleep(0.1)
            timeout -= 1
        self.report_status(f"Setup connection to {self.host}:{self.port} timed out")
        return False

    def reconnect(self):
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except Exception as e:
                self.report_status(f"Unable to reconnect: {e}, looking for dvl again...")
                self.look_for_dvl()
        success = self.setup_connections()
        if success:
            self.last_recv_time = time.time()  # Don't disconnect directly after connect
            return True

        return False

    def _get_r_body_to_earth(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        R = np.array(
            [
                [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                [-sp, sr * cp, cr * cp],
            ]
        )
        return R

    def handle_velocity(self, data: Dict[str, Any]) -> None:
        # extract velocity data from the DVL JSON
        vx, vy, vz, alt, valid, fom = (
            data["vx"],
            data["vy"],
            data["vz"],
            data["altitude"],
            data["velocity_valid"],
            data["fom"],
        )
        dt = data["time"] / 1000
        dx = dt * vx
        dy = dt * vy
        dz = dt * vz

        # fom is the standard deviation. scaling it to a confidence from 0-100%
        # 0 is a very good measurement, 0.4 is considered a inaccurate measurement
        _fom_max = 0.4
        confidence = 100 * (1 - min(_fom_max, fom) / _fom_max) if valid else 0
        # confidence = 100 if valid else 0

        if not valid:
            logger.info("Invalid  dvl reading, ignoring it.")
            return

        # Process individual beam distances
        beam_distances = []
        beam_valid = []
        if "transducers" in data:
            for transducer in data["transducers"]:
                beam_distances.append(transducer["distance"])
                beam_valid.append(transducer["beam_valid"])

        is_test_mode = os.environ.get("DVL_TEST_MODE", "false").lower() == "true"

        if "transducers" in data or is_test_mode:
            self.last_beam_distances = beam_distances
            self.last_beam_valid = beam_valid

        # We explicitly mock MAVLink messages (depth and attitude) in DVL_TEST_MODE
        # since the test environment might not emit them.
        is_test_mode = os.environ.get("DVL_TEST_MODE", "false").lower() == "true"

        rov_d = 0.0
        r_roll, r_pitch, r_yaw = 0.0, 0.0, 0.0

        try:
            if is_test_mode:
                vfr_hud = self.mav.get("/VFR_HUD/message")
                if vfr_hud:
                    rov_d = -float(json.loads(vfr_hud)["alt"])
                else:
                    rov_d = 1.0  # 1m depth assumption for test mode

                attitude = self.mav.get("/ATTITUDE/message")
                if attitude:
                    attitude_data = json.loads(attitude)
                    r_roll, r_pitch, r_yaw = attitude_data["roll"], attitude_data["pitch"], attitude_data["yaw"]
            else:
                rov_d = -float(self.mav.get("/VFR_HUD/message/alt"))
                attitude_data = json.loads(self.mav.get("/ATTITUDE/message"))
                r_roll, r_pitch, r_yaw = attitude_data["roll"], attitude_data["pitch"], attitude_data["yaw"]
        except Exception:
            pass  # Accept 0s if we fail to fetch MAVLink on this cycle

        if self.ekf is None and alt > 0:
            logger.info(f"Initializing TerrainEKF with alt={alt}, rov_d={rov_d}")
            self.ekf = TerrainEKF(
                dvl_model=DVLModelA50(),
                initial_terrain_d=alt + rov_d,
                params=EKFParams(
                    delay=self.ekf_sensor_delay,
                    t_var=self.ekf_terrain_variance,
                    s_var=self.ekf_slope_variance,
                    t_noise=self.ekf_terrain_process_noise,
                    s_noise=self.ekf_slope_process_noise,
                    gate=self.ekf_gate_threshold,
                ),
            )
            self.ekf_logger = EKFLogger()

        if self.ekf is not None:
            # We want vn, ve in earth frame for predict step
            R_body_to_earth = self._get_r_body_to_earth(r_roll, r_pitch, r_yaw)
            v_body = np.array([vx, vy, vz])
            v_earth = R_body_to_earth @ v_body
            vn, ve, vd = v_earth[0], v_earth[1], v_earth[2]

            # Predict
            self.ekf.predict(vn, ve, dt)

            # Update if we have valid beams
            if len(beam_distances) == 4:
                # Replace invalid beams with 0 for EKF (TerrainEKF handles 0 as reject)
                ekf_beams = [d if v else 0.0 for d, v in zip(beam_distances, beam_valid)]
                # Convert list to array or pass directly depending on if TerrainEKF expects array.
                self.ekf.update(ekf_beams, rov_d, R_body_to_earth, beam_variance=0.01)

            # Project forward from t_capture to t_now
            state_proj, _ = self.ekf.project((vn, ve), self.ekf_sensor_delay)
            self.ekf_state_terrain_d = self.ekf.x[0, 0]
            self.ekf_state_slope_n = self.ekf.x[1, 0]
            self.ekf_state_slope_e = self.ekf.x[2, 0]

            # Altitude of terrain at t_now (positive down)
            self.ekf_projected_terrain_d = state_proj[0]

            # Vertical range from ROV to terrain
            self.ekf_projected_alt = state_proj[0] - rov_d

            # Calculate the rangefinder distance along the downward body Z axis
            v_earth_d = R_body_to_earth[:, 2]
            normal_earth = np.array([self.ekf_state_slope_n, self.ekf_state_slope_e, 1.0])
            dot_prod = np.dot(v_earth_d, normal_earth)
            if dot_prod > 0.01:
                self.ekf_projected_rangefinder = max(0.0, self.ekf_projected_alt / dot_prod)
            else:
                self.ekf_projected_rangefinder = 0.0

            # Log Data
            self.ekf_logger.log(
                [
                    time.time(),
                    beam_distances[0],  # aft-right
                    beam_distances[1],  # aft-left
                    beam_distances[2],  # forward-left
                    beam_distances[3],  # forward-right
                    vn,
                    ve,
                    vd,
                    rov_d,
                    r_roll,
                    r_pitch,
                    r_yaw,
                    self.ekf_state_terrain_d,
                    self.ekf_state_slope_n,
                    self.ekf_state_slope_e,
                    self.ekf_projected_terrain_d,
                    self.ekf_projected_alt,
                    self.ekf_projected_rangefinder,
                    self.ekf.beam_status[0],
                    self.ekf.beam_status[1],
                    self.ekf.beam_status[2],
                    self.ekf.beam_status[3],
                ]
            )

            # Send main rangefinder message
            if self.rangefinder:
                if self.send_ekf_output:
                    if self.ekf_projected_rangefinder > 0.05:
                        self.mav.send_rangefinder(self.ekf_projected_rangefinder)
                elif alt > 0.05:
                    self.mav.send_rangefinder(alt)
        else:
            # Send main rangefinder message
            if self.rangefinder and alt > 0.05:
                self.mav.send_rangefinder(alt)

        position_delta = [0, 0, 0]
        attitude_delta = [0, 0, 0]
        if self.should_send == MessageType.POSITION_DELTA:
            dRoll, dPitch, dYaw = [
                current_angle - last_angle
                for (current_angle, last_angle) in zip(self.current_attitude, self.last_attitude)
            ]
            if self.orientation == DVL_DOWN:
                position_delta = [dx, dy, dz]
                attitude_delta = [dRoll, dPitch, dYaw]
            elif self.orientation == DVL_FORWARD:
                position_delta = [dz, dy, -dx]
                attitude_delta = [dYaw, dPitch, -dRoll]
            self.mav.send_vision(position_delta, attitude_delta, dt=data["time"] * 1e3, confidence=confidence)
        elif self.should_send == MessageType.SPEED_ESTIMATE:
            velocity = [vx, vy, vz] if self.orientation == DVL_DOWN else [vz, vy, -vx]  # DVL_FORWARD
            self.mav.send_vision_speed_estimate(velocity)

        self.last_attitude = self.current_attitude

    def handle_position_local(self, data):
        self.current_attitude = data["roll"], data["pitch"], data["yaw"]
        if self.should_send == MessageType.POSITION_ESTIMATE:
            x, y, z = data["x"], data["y"], data["z"]
            self.timestamp = data["ts"]
            self.mav.send_vision_position_estimate(
                self.timestamp, [x, y, z], self.current_attitude, reset_counter=self.reset_counter
            )

    def check_temperature(self):
        now = time.time()
        if now - self.last_temperature_check_time < self.temperature_check_interval_s:
            return
        self.last_temperature_check_time = now
        try:
            response_text = request(f"http://{self.hostname}/api/v1/about/status")
            if not response_text:
                return

            status = json.loads(response_text)

            temp = float(status["temperature"])
            if temp > self.temperature_too_hot:
                self.report_status(f"DVL is too hot ({temp} C). Please cool it down.")
                self.mav.send_statustext(f"DVL is too hot ({temp} C). Please cool it down.")
        except Exception as e:
            self.report_status(e)

    def run(self):
        """
        Runs the main routing
        """
        self.load_settings()
        self.look_for_dvl()
        self.setup_connections()
        self.wait_for_vehicle()
        self.setup_mavlink()
        self.setup_params()
        time.sleep(1)
        self.report_status("Running")
        self.last_recv_time = time.time()
        buf = ""
        connected = True
        while True:
            if not self.enabled:
                time.sleep(1)
                buf = ""  # Reset buf when disabled
                continue

            r, _, _ = select([self.socket], [], [], 0)
            data = None
            if r:
                try:
                    recv = self.socket.recv(1024).decode()
                    connected = True
                    if recv:
                        self.last_recv_time = time.time()
                        buf += recv
                except socket.error as e:
                    logger.warning(f"Disconnected: {e}")
                    connected = False
                except Exception as e:
                    logger.warning(f"Error receiving: {e}")

            # Extract 1 complete line from the buffer if available
            if len(buf) > 0:
                lines = buf.split("\n", 1)
                if len(lines) > 1:
                    buf = lines[1]
                    data = json.loads(lines[0])

            if not connected:
                buf = ""
                self.report_status("restarting")
                self.reconnect()
                time.sleep(0.003)
                continue

            if not data:
                if time.time() - self.last_recv_time > self.timeout:
                    buf = ""
                    self.report_status("timeout, restarting")
                    connected = self.reconnect()
                time.sleep(0.003)
                continue

            self.status = "Running"

            if "type" not in data:
                continue

            if data["type"] == "velocity":
                self.handle_velocity(data)
            elif data["type"] == "position_local":
                self.handle_position_local(data)

            self.check_temperature()
            time.sleep(0.003)
        logger.error("Driver Quit! This should not happen.")
