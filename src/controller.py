import numpy as np
from collections import deque

class pid:
    
    def __init__(self, kp, ki, kd):
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd

        self.__integrated_error = deque(maxlen=1000)
        self.__prev_error = 0

    def gain(self, error):
        d_error = 0 if self.__prev_error == 0 else (error - self.__prev_error)
        self.__prev_error = error

        self.__integrated_error.append(error)
        
        return self.__kp * error + self.__ki * sum(self.__integrated_error) + self.__kd * d_error