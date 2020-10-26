from PController import PController


class PdController(PController):
    """
    Class to define the PdController object
    """

    def __init__(self,
                 kp,
                 kd,
                 ts):
        """
        Constructor for the PdController class
        :param kp: The continuous-time gain for the proportional controller
        :param kd: The continuous-time gain for the differential controller
        :param ts: The sampling time of the controller
        """
        super().__init__(kp, ts)  # Construct a PController to inherit from
        self._kd = kd / ts  # Discrete-time kd
        self._error_previous = None  # The error recorded the previous time it was calculated

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The PD control variable
        """
        # Use the control function from the PController
        u = super().control(y, set_point)

        # Add to u based on the differential controller
        if self._error_previous is not None:
            u += self._kd * (self._error - self._error_previous)

        self._error_previous = self._error  # Store the calculated error as the previous error for future use

        return u
