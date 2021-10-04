import sys # argv, argc
import datetime
import rospy
import math
import time
import numpy as np
import random
# Messages Imports
from mavros_msgs.srv import CommandBool
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
# State Machine Imports
import smach
import smach_ros
import smach_viewer
from smach import CBState
import actionlib
# Import other classes
from follow_UGV import Publisher

##========================================================================
## Global Variables
#Subscribers
sub_marker = None
sub_odom = None
#Observations
obs = []
#Other files
publisher = None
#publisher = Publisher()
check = 0
error_threshold = 0.15
ros_node = None

##========================================================================
## STATES
def WaitAtHome(userdata, msg):
    # The return needs to be False to return invalid and the monitoring terminates
    # Since we are returning a "when there are no markers", when this is false
    # it means that there is a marker and we can move on to the next State
    return msg.transforms == []

def CheckMarker(userdata, msg):
    global check
    # The return needs to be False to return invalid and the monitoring terminates
    # Since we are returning a "when there are no markers", when this is false
    # it means that there is a marker and we can move on to the next State
    if msg.transforms == []:
        check = check+1
    if msg.transforms != []:
        check = 0
    return check!=200

class LandingSequence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landed'])
        publisher = Publisher(ros_node)

    def execute(self, userdata):
        global check_landing
        rospy.loginfo("Starting the Landing Sequence")

        return publisher.start()


##========================================================================
## IMPLEMENTATION

def main():
    global ros_node, publisher
    ros_node = rospy.init_node('uav_state_machine')
    time_initial = time.time()

    #Other files
    publisher = Publisher(ros_node)

    #Create a SMACH State Machine
    ##Overall big state machine
    sm = smach.StateMachine(outcomes=['finished', 'aborted', 'preempted'])
    #Open the container
    with sm:

        smach.StateMachine.add('WaitAtHome', smach_ros.MonitorState('/fiducial_transforms', FiducialTransformArray, WaitAtHome), 
                                    transitions={'invalid':'Landing_Con', 'valid':'WaitAtHome', 'preempted':'WaitAtHome'})
        
        #Create sub SMACH state machine
        ##Concurrence State Machine to Monitor the marker topic while landing
        sm_con = smach.Concurrence(outcomes=['landed','lostMarker'],
                                    default_outcome='lostMarker',
                                    # landed will be triggered if LandingSequence ends
                                    # lostMarker will be triggered if CheckMarker ends
                                    # both independent of the other, and CON ends
                                    outcome_map={'landed':{'LandingSequence':'landed'},
                                                'lostMarker': {'CheckMarker':'invalid'}})
        # Open the container
        with sm_con:
            #Add States to the container
            ##Sub State Machine with the states required to reach the UGV and perform the landing
            smach.Concurrence.add('LandingSequence', LandingSequence())
            ##Concurrent State that continuously checks if the marker has been lost
            smach.Concurrence.add('CheckMarker', smach_ros.MonitorState('/fiducial_transforms', FiducialTransformArray, CheckMarker))
        

        # Add the concurrence container to the State Machine
        smach.StateMachine.add('Landing_Con', sm_con,
                                    transitions={'lostMarker':'WaitAtHome',
                                                'landed':'finished'})



    #Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()                     
