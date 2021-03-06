class PController:
    """
    Class to define the PController object
    """

    def __init__(self,
                 kp,
                 ts):
        """
        Constructor for the PController class
        :param kp: The continuous-time gain for the proportional controller
        :param ts: The sampling time of the controller
        """
        self._kp = kp
        self._ts = ts
        self._error = 0

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The P control variable
        """
        # Calculate the error
        self._error = set_point - y

        # Define u from the proportional controller
        u = self._kp * self._error

        return u
