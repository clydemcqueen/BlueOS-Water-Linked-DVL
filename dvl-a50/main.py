#!/usr/bin/env python3
"""
Driver for the Water Linked DVL A-50
"""

import json

from flask import Flask

from dvl import DvlDriver
from terrain_ekf import EKFParams

# set the project root directory as the static folder, you can set others.
app = Flask(__name__, static_url_path="/static", static_folder="static")
thread = None
sim = None


class API:
    dvl = None

    def __init__(self, dvl: DvlDriver):
        self.dvl = dvl

    def get_status(self) -> str:
        """
        Returns the driver status as a JSON containing the keys
        status, orientation, hostname, and enabled
        """
        return json.dumps(self.dvl.get_status())

    def set_enabled(self, enabled: str) -> bool:
        """
        Enables/Disables the DVL driver
        """
        if enabled in ["true", "false"]:
            return self.dvl.set_enabled(enabled == "true")
        return False

    def set_orientation(self, orientation: int) -> bool:
        """
        Sets the DVL mounting orientation:
        1 = Down
        2 = Forward
        """
        return self.dvl.set_orientation(orientation)

    def set_hostname(self, hostname: str) -> bool:
        """
        Sets the Hostname or IP where the driver tries to connect to the DVL
        """
        return self.dvl.set_hostname(hostname)

    def set_current_position(self, lat: str, lon: str) -> bool:
        """
        Sets the EKF origin to lat, lon
        """
        return self.dvl.set_current_position(float(lat), float(lon))

    def set_use_as_rangefinder(self, enabled: str) -> bool:
        """
        Enables/disables usage of DVL as rangefinder
        """
        if enabled in ["true", "false"]:
            return self.dvl.set_use_as_rangefinder(enabled == "true")
        return False

    def set_terrain_ekf_enabled(self, enabled: str) -> bool:
        """
        Enables/disables the Terrain EKF computation
        """
        if enabled in ["true", "false"]:
            return self.dvl.set_terrain_ekf_enabled(enabled == "true")
        return False

    def set_ekf_enabled(self, enabled: str) -> bool:
        """
        Enables/disables the EKF output sending
        """
        if enabled in ["true", "false"]:
            return self.dvl.set_ekf_enabled(enabled == "true")
        return False

    def set_ekf_params(self, params: EKFParams) -> bool:
        """
        Sets the parameters for TerrainEKF
        """
        return self.dvl.set_ekf_params(params)

    def load_params(self, selector: str) -> bool:
        """
        Load parameters
        """
        if selector in ["dvl", "dvl_gps"]:
            return self.dvl.load_params(selector)
        return False

    def set_message_type(self, messagetype: str):
        self.dvl.set_should_send(messagetype)


if __name__ == "__main__":
    import os

    is_sim_mode = os.environ.get("DVL_SIM_MODE", "false").lower() == "true"
    if is_sim_mode:
        os.environ["DVL_TEST_MODE"] = "true"  # Ensure discovery & cable guy are bypassed
        from simulator import DvlSimulator

        sim = DvlSimulator(port=16171)
        sim.start()

        driver = DvlDriver()
        driver.mav = sim.mav_helper
        driver.hostname = "127.0.0.1"
        driver.port = 16171

        # Patch load_settings to not overwrite simulator connection
        original_load_settings = driver.load_settings

        def sim_load_settings():
            original_load_settings()
            driver.hostname = "127.0.0.1"
            driver.port = 16171

        driver.load_settings = sim_load_settings
    else:
        driver = DvlDriver()

    api = API(driver)

    @app.route("/get_status")
    def get_status():
        return api.get_status()

    @app.route("/enable/<enable>")
    def set_enabled(enable: str):
        return str(api.set_enabled(enable))

    @app.route("/use_as_rangefinder/<enable>")
    def set_use_rangefinder(enable: str):
        return str(api.set_use_as_rangefinder(enable))

    @app.route("/set_terrain_ekf_enabled/<enable>")
    def set_terrain_ekf_enabled_route(enable: str):
        return str(api.set_terrain_ekf_enabled(enable))

    @app.route("/set_ekf_enabled/<enable>")
    def set_ekf_enabled_route(enable: str):
        return str(api.set_ekf_enabled(enable))

    # pylint: disable=too-many-arguments,too-many-positional-arguments
    @app.route("/set_ekf_params/<delay>/<t_var>/<s_var>/<t_noise>/<s_noise>/<gate>")
    def set_ekf_params_route(delay: str, t_var: str, s_var: str, t_noise: str, s_noise: str, gate: str):
        params = EKFParams(
            delay=float(delay),
            t_var=float(t_var),
            s_var=float(s_var),
            t_noise=float(t_noise),
            s_noise=float(s_noise),
            gate=float(gate),
        )
        return str(api.set_ekf_params(params))

    @app.route("/load_params/<selector>")
    def load_params(selector: str):
        return str(api.load_params(selector))

    @app.route("/orientation/<int:orientation>")
    def set_orientation(orientation: int):
        return str(api.set_orientation(orientation))

    @app.route("/message_type/<messagetype>")
    def set_message_type(messagetype: str):
        return str(api.set_message_type(messagetype))

    @app.route("/setcurrentposition/<lat>/<lon>")
    def set_current_position(lat, lon):
        return str(api.set_current_position(lat, lon))

    @app.route("/register_service")
    def register_service():
        return app.send_static_file("service.json")

    @app.route("/")
    def root():
        return app.send_static_file("index.html")

    driver.start()
    # Use a specific port number if requested
    port = os.environ.get("GUI_PORT", "9001")
    app.run(host="0.0.0.0", port=port)
