import numpy as np

from utils import State

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """
    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """

        self.params = params
        self.init_state = init_state
        self.prev_state = self.init_state
        self.state_var = 0
        self.var_f = .0001
        self.var_z = .01

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        filtered_state = State()

        accel = U/self.params.mass - self.params.g

        mu_x = self.prev_state.z_pos + 1/2*accel*time_delta**2 + self.prev_state.z_vel*time_delta
        var_x = self.state_var + self.var_f

        K = (var_x)/(var_x + self.var_z)

        y = z_meas - mu_x

        filtered_state.z_pos = mu_x + K*y
        filtered_state.z_vel = accel*time_delta + self.prev_state.z_vel

        self.state_var = (var_x * self.var_z)/(var_x + self.var_z)

        self.prev_state = filtered_state

        return filtered_state
