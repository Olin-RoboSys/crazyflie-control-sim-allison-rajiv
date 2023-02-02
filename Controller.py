
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains da`taclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd
        self.total_err = 0
        self.prev_vel = 0

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        # e(t)
        err = setpoint.z_pos - state.z_pos
        # Derivative of e(t)
        derr_dt = setpoint.z_vel - state.z_vel
        # Integral of e(t)
        self.total_err += err
        
        z_dotdot = self.kp_z * err + self.kd_z * derr_dt + self.ki_z * self.total_err
        U = self.params.mass * (z_dotdot + self.params.g)

        return U
