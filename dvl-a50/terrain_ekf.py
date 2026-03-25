from typing import NamedTuple

import numpy as np

BEAM_ACCEPT = 0
BEAM_REJECT_MIN_ALT = 1
BEAM_REJECT_RANGE = 2
BEAM_REJECT_TILT = 3
BEAM_REJECT_NIS = 4
EKF_SINGULAR_MATRIX = 5


class EKFParams(NamedTuple):
    delay: float
    t_var: float
    s_var: float
    t_noise: float
    s_noise: float
    gate: float


def project(x, P, Q_per_sec, velocity, sensor_delay):
    """
    The filter runs at t_capture; project forward to t_now.
    """
    vn, ve = velocity
    F = np.array([[1.0, -vn * sensor_delay, -ve * sensor_delay], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    x_proj = F @ x
    Q_proj = Q_per_sec * sensor_delay
    P_proj = F @ P @ F.T + Q_proj

    return x_proj.flatten(), P_proj


# pylint: disable=too-many-statements
class TerrainEKF:
    """
    Tightly coupled Terrain EKF.

        x[0] terrain_d (m): depth of the seafloor (positive down)
        x[1] slope_n (unitless): terrain rises to the north
        x[2] slope_e (unitless): terrain rises to the east
    """

    def __init__(
        self,
        dvl_model,
        initial_terrain_d,
        params: EKFParams,
        initial_slope=(0.0, 0.0),
    ):

        initial_slope_n, initial_slope_e = initial_slope
        self.x = np.array([[initial_terrain_d], [initial_slope_n], [initial_slope_e]], dtype=float)

        self.P = np.diag([params.t_var, params.s_var, params.s_var]).astype(float)

        self.Q_per_sec = np.diag([params.t_noise, params.s_noise, params.s_noise])

        self.gate_threshold = params.gate

        # Beam vectors in body frame
        self.beam_vectors_body = dvl_model.get_beam_vectors_body()
        self.beam_status = [BEAM_ACCEPT] * 4

    def predict(self, vn, ve, dt) -> np.ndarray:
        F = np.array([[1.0, -vn * dt, -ve * dt], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

        Q = self.Q_per_sec * dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Return F for use by the RTS smoother
        return F

    def update(self, beams, rov_d, R_body_to_earth, beam_variance):
        est_terrain_d = self.x[0, 0]
        est_slope_n = self.x[1, 0]
        est_slope_e = self.x[2, 0]
        est_alt = est_terrain_d - rov_d

        self.beam_status = [BEAM_ACCEPT] * 4

        # TODO relax this while the EKF is warming up
        if est_alt < 0.1:
            print(f"est_alt {est_alt} = est_terrain_d {est_terrain_d} - rov_d {rov_d}, skipping update")
            self.beam_status = [BEAM_REJECT_MIN_ALT] * 4
            return

        z_meas_residual_list = []
        H_list = []
        R_list = []

        n_earth = np.array([est_slope_n, est_slope_e, 1.0])

        # For each beam, calculate the residual and reject outliers
        for i in range(4):
            measured_range = beams[i]

            if measured_range <= 0:
                self.beam_status[i] = BEAM_REJECT_RANGE
                continue

            v_body = self.beam_vectors_body[i]
            v_earth = R_body_to_earth @ v_body
            dot_prod = np.dot(v_earth, n_earth)

            if dot_prod <= 0.01:
                self.beam_status[i] = BEAM_REJECT_TILT
                continue

            expected_range = est_alt / dot_prod
            residual = measured_range - expected_range

            # Use NIS (normalized innovation squared) to reject outliers.
            # A production filter should have a restart mechanism.
            dR_dT = 1.0 / dot_prod
            dR_dSn = -(est_alt * v_earth[0]) / (dot_prod**2)
            dR_dSe = -(est_alt * v_earth[1]) / (dot_prod**2)
            H_i = np.array([[dR_dT, dR_dSn, dR_dSe]])
            S_i = (H_i @ self.P @ H_i.T) + beam_variance
            S_scalar = S_i[0, 0]
            nis = (residual**2) / S_scalar

            if nis > self.gate_threshold:
                self.beam_status[i] = BEAM_REJECT_NIS
                # print(f"Beam {i} rejected due to NIS {nis} > {self.gate_threshold}")
                continue

            z_meas_residual_list.append(residual)
            H_list.append(H_i[0])
            R_list.append(beam_variance)

        # Batch update
        if len(z_meas_residual_list) > 0:
            y = np.array(z_meas_residual_list).reshape(-1, 1)
            H = np.array(H_list)
            R_cov = np.diag(R_list)
            S = H @ self.P @ H.T + R_cov

            try:
                # Use solve instead of inv for stability
                K = self.P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
                self.x = self.x + (K @ y)
                I_mat = np.eye(3)
                self.P = (I_mat - (K @ H)) @ self.P
            except np.linalg.LinAlgError:
                for i in range(4):
                    if self.beam_status[i] == BEAM_ACCEPT:
                        self.beam_status[i] = EKF_SINGULAR_MATRIX

    def get_state(self):
        """Returns flattened state array [terrain, slope_n, slope_e]"""
        return self.x.flatten()

    def project(self, velocity, sensor_delay):
        """The filter runs at T-sensor_delay; project forward to T=now. Does not update the internal state."""
        return project(self.x, self.P, self.Q_per_sec, velocity, sensor_delay)
