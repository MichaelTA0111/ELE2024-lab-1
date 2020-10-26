class PidController:
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
        self.__kp = kp
        self.__kd = kd / ts  # Discrete-time kd
        self.__ki = ki * ts  # Discrete-time ki
        self.__ts = ts
        self.__error_previous = None  # The error recorded the previous time it was calculated
        self.__sum_errors = 0.  # The sum of all previous errors calculated
        self.__u = 0.

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The PID control variable
        """
        # Calculate the error
        error = set_point - y

        # Define u from the proportional controller
        u = self.__kp * error

        # Add to u based on the differential controller
        if self.__error_previous is not None:
            u += self.__kd * (error - self.__error_previous)

        # Add to u based on the integral controller
        u += self.__ki * self.__sum_errors

        self.__error_previous = error  # Store the calculated error as the previous error for future use
        self.__sum_errors += error  # Add the error to the sum of all previous errors
        self.__u = u

        return u

    def get_u(self):
        """
        Getter for the steering angle
        :return: The steering angle
        """
        return self.__u
