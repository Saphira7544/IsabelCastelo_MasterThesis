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
        self.Kp_stat = 0.4
        self.Ki_stat = 0#0.002
        self.Kd_stat = 0#0.01
        # PID constants when UGV is far away
        self.Kp_mov = 0.4
        self.Ki_mov = 0.05
        self.Kd_mov = 0.01#0.001
        # PID constants for Z axis
        self.Kp_z = 0.001#0.015
        self.Ki_z = 0.008#0.0025
        self.Kd_z = 0

        ##
        self.prev_time = [0, 0, 0, 0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.error_i = [0.0, 0.0, 0.0, 0.0]
        self.pid_distance = [0, 0, 0]

    def target_pose(self, marker, hasObs):
        target_angles = [0, 0, 0]
        descent = 0
        e_XY = [marker[0], marker[1]]
        e_Z = marker[2]
        
        # Compute the PID values for X and Y
        for i in range(len(e_XY)):
            # Set different constants for whether the UGV is moving or not    
	    kp = self.Kp_stat
	    ki = self.Ki_stat
	    kd = self.Kd_stat

            # Compute the PID output for X and Y
            self.pid_distance[i] = self.calc_error(e_XY[i], i, kp, ki, kd)

        # Compute the PID output for Z
        if(hasObs): # Only descend if currently seeing a marker
            # Failsafe measure to avoid going bellow a height when not aligned
            if( self.pythagoras(e_XY[0], e_XY[1]) > 0.1  and e_Z <= 0.7 ):
                kp_z = 0;               ki_z = 0;               kd_z = 0

            else:
                kp_z = self.Kp_z;   ki_z = self.Ki_z;   kd_z = self.Kd_z
            
            descent = self.calc_error(e_Z, 2, kp_z, ki_z, kd_z)
            # Aggressive descent when nearly landed
            #if( self.pythagoras(e_XY[0], e_XY[1]) <= 0.1  and e_Z <= 0.7 ):
            #    descent = 0.3
        # Keep calculating the PID outputs with zeros, when not seeing a marker so 
        # that a very old measurement doesn't exponentially increase the integral part
        else:
            descent = self.calc_error(0, 2, 0, 0, 0)

        if(  e_Z <= 0.6 ): 
	        descent = 0.35


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

    def pythagoras(self, c1, c2):
        """
            Compute the pythagoras theorem
        """
        return math.sqrt(pow(c1,2)+pow(c2,2))
