import sys # argv, argc
import datetime
import math
import time
import numpy as np
import random

class PID(object):
    """ PID CONTROLLER """

    def __init__(self):
        
        # PID constants when UGV is very close
        self.Kp_close = 0.4
        self.Ki_close = 0#0.002
        self.Kd_close = 0#0.01
        # PID constants for Z axis
        self.Kp_z = 0.001#0.015
        self.Ki_z = 0.008#0.0025
        self.Kd_z = 0
        ##
        self.prev_time = [0, 0, 0, 0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.error_i = [0.0, 0.0, 0.0, 0.0]
        self.pid_distance = [0, 0, 0]

    def target_pose(self, curr_pose, marker, hasObs):
        target_angles = [0, 0, 0]
        descent = 0
        
        e_XY = [marker[0], marker[1]]
        e_Z = marker[2]
        
        # Compute the PID values for X and Y
        for i in range(len(e_XY)):    
            
            kp = self.Kp_close
            ki = self.Ki_close
            kd = self.Kd_close
            
            self.pid_distance[i] = self.calc_error(e_XY[i], i, kp, ki, kd)

        # Compute the PID values for Z
        if(hasObs):

            if( self.pythagoras(e_XY[0], e_XY[1]) > 0.1  and e_Z <= 0.7 ):
                kp_z = 0;               ki_z = 0;               kd_z = 0

            else:
                kp_z = self.Kp_z;   ki_z = self.Ki_z;   kd_z = self.Kd_z
            
            descent = self.calc_error(e_Z, 2, kp_z, ki_z, kd_z)

            if( self.pythagoras(e_XY[0], e_XY[1]) <= 0.1  and e_Z <= 0.7 ):
                descent = 1.2
        else:
            descent = self.calc_error(0, 2, 0, 0, 0)


        target_angles = [self.pid_distance[0],
                        self.pid_distance[1],
                        descent]

        return target_angles
    
    def calc_error(self, error, i, kp, ki, kd):
        """
            Computes the P I D values for X and Y
        """
        curr_time = time.time()
        dt = 0
    
        if self.prev_time[i] != 0:
            # Time Variation
            dt = curr_time - self.prev_time[i]
        # Error Variation
        de = error - self.prev_error[i]

        # Proportional Error
        error_p = kp * error 
        # Integral Error
        self.error_i[i] += error*dt
        
        #Derivative Error
        error_d = 0
        if dt > 0:
            error_d = de/dt

        #Update time
        self.prev_time[i] = curr_time
        self.prev_error[i] = error 

        #Return
        P = error_p
        I = ki*self.error_i[i]
        D = kd*error_d

        return P + I + D
     
    def compensate_yaw(self, roll, pitch, yaw):
        """
            Change the angles depending on the current yaw
        """
        roll_B = roll*math.cos( yaw ) + pitch*math.sin( yaw )        
        pitch_B = -roll*math.sin( yaw ) + pitch*math.cos( yaw )

        return [roll_B, pitch_B]

    def pythagoras(self, c1, c2):
        return math.sqrt(pow(c1,2)+pow(c2,2))
