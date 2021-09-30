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
        self.Kp_close = 0.3
        self.Ki_close = 0.002#0.002
        self.Kd_close = 0.005#0.01
        # PID constants when UGV is far away
        self.Kp_far = 0.4
        self.Ki_far = 0.05
        self.Kd_far = 0.01#0.001
        # PID constants for Z axis
        self.Kp_z = 0.001#0.015
        self.Ki_z = 0.008#0.0025
        self.Kd_z = 0

        ##
        self.prev_time = [0, 0, 0, 0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.error_i = [0.0, 0.0, 0.0, 0.0]
        self.pid_distance = [0, 0, 0]

    def target_pose(self, marker, hasObs, isUGVmoving):
        target_angles = [0, 0, 0]
        descent = 0
        e_XY = [marker[0], marker[1]]
        e_Z = marker[2]
        
        # Compute the PID values for X and Y
        for i in range(len(e_XY)):
                
            if(isUGVmoving):
                kp = self.Kp_far
                ki = self.Ki_far
                kd = self.Kd_far
            else:
                kp = self.Kp_close
                ki = self.Ki_close
                kd = self.Kd_close

            self.pid_distance[i] = self.calc_error(e_XY[i], i, kp, ki, kd)
        print(self.pythagoras(e_XY[0], e_XY[1]))
        print(e_Z)
        # Compute the PID values for Z
        if(hasObs):
            """
            if( self.pythagoras(e_XY[0], e_XY[1]) > 1  and e_Z > 0.3 ):
                kp_z = 0;               ki_z = 0;               kd_z = 0
            elif( self.pythagoras(e_XY[0], e_XY[1]) > 0.4  and e_Z > 0.3 ):
                kp_z = self.Kp_z_far;   ki_z = self.Ki_z_far;   kd_z = self.Kd_z_far
            elif( self.pythagoras(e_XY[0], e_XY[1]) > 0.1  and e_Z > 0.3 ):
                kp_z = self.Kp_z_close; ki_z = self.Ki_z_close; kd_z = self.Kd_z_close
            elif( self.pythagoras(e_XY[0], e_XY[1]) > 0.1  and e_Z <= 0.3 ):
                kp_z = 0;               ki_z = 0;               kd_z = 0
            else:
                kp_z =  1/e_Z; ki_z = 0.5; kd_z = 0
            """ 

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

        print("Target angles = ", target_angles)
        print("================")
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

    def pythagoras(self, c1, c2):
        return math.sqrt(pow(c1,2)+pow(c2,2))
