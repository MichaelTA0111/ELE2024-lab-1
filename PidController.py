from PdController import PdController


class PidController(PdController):
    """
    Class to define the PidController object
    """

    def __init__(self,
                 kp,
                 kd,
                 ki,
                 ts):
        """
        Constructor for the PidController class
        :param kp: The continuous-time gain for the proportional controller
        :param kd: The continuous-time gain for the differential controller
        :param ki: The continuous-time gain for the integral controller
        :param ts: The sampling time of the controller
        """
        super().__init__(kp, kd, ts)  # Construct a PdController to inherit from
        self.__ki = ki * ts  # Discrete-time ki
        self.__sum_errors = 0.  # The sum of all previous errors calculated
        self.__u = 0.

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The PID control variable
        """
        # Use the control function from the PdController
        u = super().control(y, set_point)

        # Add to u based on the integral controller
        u += self.__ki * self.__sum_errors

        self.__sum_errors += self._error  # Add the error to the sum of all previous errors
        self.__u = u

        return u

    def get_u(self):
        """
        Getter for the steering angle
        :return: The steering angle
        """
        return self.__u
