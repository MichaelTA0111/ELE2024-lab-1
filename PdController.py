class PdController:
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
        self.__kp = kp
        self.__kd = kd / ts  # Discrete-time kd
        self.__ts = ts
        self.__error_previous = None  # The error recorded the previous time it was calculated

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The PD control variable
        """
        # Calculate the error
        error = set_point - y

        # Define u from the proportional controller
        u = self.__kp * error

        # Add to u based on the differential controller
        if self.__error_previous is not None:
            u += self.__kd * (error - self.__error_previous)

        self.__error_previous = error  # Store the calculated error as the previous error for future use

        return u
