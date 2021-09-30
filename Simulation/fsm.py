import sys # argv, argc
import datetime
import rospy
import math
import time
import numpy as np
import random
# Plot Imports
from mpl_toolkits import mplot3d
import matplotlib.markers
import matplotlib.pyplot as plt
import matplotlib.gridspec as spec
from matplotlib.patches import Ellipse
# Messages Imports
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetModelState
from mavros_msgs.srv import CommandBool
# State Machine Imports
import smach
import smach_ros
import smach_viewer
from smach import CBState
import actionlib
# Transformations between frames Imports
import tf
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion
import tf2_ros
from tf2_ros import TransformListener
# Import other classes
from follow_UGV import Publisher
from pid import PID

##========================================================================
## Global Variables

#Subscribers
sub_marker = None
sub_odom = None
#Publishers
pub_vel = None
#Iris Variables
pos = [0, 0, 0, 0, 0, 0]
x_trajectory = []
y_trajectory = []
time_initial = 0
x_trajectory_iris = []
y_trajectory_iris = []
z_trajectory_iris = []
check_landing = False
## Jackal Variables ##
x_trajectory_jackal = []
y_trajectory_jackal = []
z_trajectory_jackal = []
ugv_height = 0.3
#Observations
obs = []
obs_trajectory = np.array([0,0,0,0])
has_obs = False
#Tranforms between TF
listener = None
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

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['reachedHome','failed'])
def GoHome(user_data):
    global check
    check = 0 #Restaure the CheckMarker flag to initial state
    rospy.loginfo('Going Home...')
    
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    rospy.sleep(1)
    msg = PoseStamped()
    msg.pose.position.x=0
    msg.pose.position.y=0
    msg.pose.position.z=6#pos[2]
    #msg.pose.orientation.z = 3.14
    result = pose_pub.publish(msg)

    if result == None:
        return'reachedHome'
    else:
        return'failed'

def CheckMarker(userdata, msg):
    global check
    # The return needs to be False to return invalid and the monitoring terminates
    # Since we are returning a "when there are no markers", when this is false
    # it means that there is a marker and we can move on to the next State
    if msg.transforms == []:
        check = check+1
    return check!=200

class LandingSequence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landed'])
        publisher = Publisher(ros_node)

    def execute(self, userdata):
        global check_landing
        rospy.loginfo("Starting the Landing Sequence")

        return publisher.start()

# Gets called when any child state terminates
def child_term_cb(outcome_map):
    # Terminates all running states if the FollowUGV state finished with landed
    if outcome_map['LandingSequence'] == 'landed':
        return True
    # Terminates all running states if CheckMarker finished with invalid
    if outcome_map['CheckMarker'] == 'invalid':
        return True
    # In any other case, just keep running
    return False

##========================================================================
## FUNCTIONS TO RETRIEVE INFORMATION FROM TOPICS 

"""
> Markers Callback
"""
def callback_marker(msg):
    global obs_trajectory, obs, check

    for marker in msg.transforms:

        try:
            marker_id = "fiducial_T_" + str(marker.fiducial_id)
            (trans, rot) = listener.lookupTransform('base_link', marker_id, rospy.Time(0))

            obs = np.array([marker.fiducial_id,
                -trans[0],
                trans[1],
                trans[2] ]) 

            obs_trajectory = np.vstack((obs_trajectory, obs))
            check = 0

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print ("Fail", e)


    get_jackal_groundtruth()
    get_iris_groundtruth()

"""
    Functions that call the rosservice and stores the ground truth pose
"""
def get_jackal_groundtruth():
    global x_trajectory_jackal, y_trajectory_jackal
    jackal = get_model_pose('jackal0')
    pose_jackal = np.array([jackal.position.x,
                                jackal.position.y,
                                jackal.position.z,
                            euler_from_quaternion([jackal.orientation.x,
                                                jackal.orientation.y,
                                                jackal.orientation.z,
                                                jackal.orientation.w])[0],
                            euler_from_quaternion([jackal.orientation.x,
                                                jackal.orientation.y,
                                                jackal.orientation.z,
                                                jackal.orientation.w])[1],
                            euler_from_quaternion([jackal.orientation.x,
                                                jackal.orientation.y,
                                                jackal.orientation.z,
                                                jackal.orientation.w])[2]])

    x_trajectory_jackal.append(pose_jackal[0])
    y_trajectory_jackal.append(pose_jackal[1])   
    z_trajectory_jackal.append(pose_jackal[2])


def get_iris_groundtruth():
    global x_trajectory_iris, y_trajectory_iris, z_trajectory_iris
    iris = get_model_pose('iris')
    pose_iris = np.array([iris.position.x,
                            iris.position.y,
                            iris.position.z,
                        euler_from_quaternion([iris.orientation.x,
                                            iris.orientation.y,
                                            iris.orientation.z,
                                            iris.orientation.w])[0],
                        euler_from_quaternion([iris.orientation.x,
                                            iris.orientation.y,
                                            iris.orientation.z,
                                            iris.orientation.w])[1],
                        euler_from_quaternion([iris.orientation.x,
                                            iris.orientation.y,
                                            iris.orientation.z,
                                            iris.orientation.w])[2]])

    x_trajectory_iris.append(pose_iris[0])
    y_trajectory_iris.append(pose_iris[1])   
    z_trajectory_iris.append(pose_iris[2])

"""
    Ros service to retrieve the ground truth pose of a certain model in the gazebo simulation
"""
def get_model_pose(model_name, relative_entity_name='world'):
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        res = get_model_state(model_name, relative_entity_name)
        return res.pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

##========================================================================
## IMPLEMENTATION

def plots():
    """
        Executes this function when the code is shut down
    """
    print ('>>> SHUTTING DOWN <<<')
    fig = plt.figure()
    ax=plt.axes(projection ='3d')

    ax.plot3D(x_trajectory_iris, y_trajectory_iris, z_trajectory_iris, 'blue', label='Iris')
    ax.plot3D(x_trajectory_jackal, y_trajectory_jackal, z_trajectory_jackal, 'red', label='Jackal')

    ax.scatter3D(x_trajectory_iris, y_trajectory_iris, z_trajectory_iris, marker="x", c=z_trajectory_iris, cmap='Purples')
    ax.scatter3D(x_trajectory_jackal, y_trajectory_jackal, z_trajectory_jackal, marker="D" ,c=z_trajectory_jackal, cmap='Purples')

    ax.set_title("Robot Trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    #min_x = min(min(x_trajectory_iris), min(x_trajectory_jackal))
    #min_y = min(min(y_trajectory_iris), min(y_trajectory_jackal))
    #max_x = max(max(x_trajectory_iris), max(x_trajectory_jackal))
    #max_y = max(max(x_trajectory_iris), max(x_trajectory_jackal))
    #ax.set_xlim(min(min_x,min_y), max(max_x,max_y))
    #ax.set_ylim(min(min_x,min_y), max(max_x,max_y))

    plt.legend()
    plt.show()

    file = open("trajectories.txt", "a")
    x_iris = str(x_trajectory_iris); y_iris = str(y_trajectory_iris); z_iris = str(z_trajectory_iris); 
    x_jackal = str(x_trajectory_jackal); y_jackal = str(y_trajectory_jackal); z_jackal = str(z_trajectory_jackal); 

    file.write("x_iris = %s\ny_iris = %s\nz_iris = %s\nx_jackal = %s\ny_jackal = %s\nz_jackal = %s\n\n\n"
            %(x_iris,y_iris,z_iris,x_jackal,y_jackal,z_jackal))
    
    file.close()

def main():
    global sub_marker, sub_odom, pub_vel, listener, ros_node, pid, publisher
    ros_node = rospy.init_node('uav_state_machine')
    time_initial = time.time()

    #Other files
    publisher = Publisher(ros_node)

    ## Subscribers ##
    sub_marker = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback_marker)
    #Tramsforms
    listener = tf.TransformListener()

    #Create a SMACH State Machine
    ##Overall big state machine
    sm = smach.StateMachine(outcomes=['finished', 'aborted', 'preempted'])
    #Open the container
    with sm:
        #Add States to the container
        ##State that publishes a command for the UAV to go to position (0,0)
        smach.StateMachine.add('GoHome', CBState(GoHome), 
                                    {'reachedHome':'WaitAtHome', 'failed':'GoHome'})
        ##State that Monitors the markers topic and waits to find a marker
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
                                                'lostMarker': {'CheckMarker':'invalid'}},     
                                    child_termination_cb=child_term_cb)
        # Open the container
        with sm_con:
            #Add States to the container
            ##Sub State Machine with the states required to reach the UGV and perform the landing
            smach.Concurrence.add('LandingSequence', LandingSequence())
            ##Concurrent State that continuously checks if the marker has been lost
            smach.Concurrence.add('CheckMarker', smach_ros.MonitorState('/fiducial_transforms', FiducialTransformArray, CheckMarker))
        
        # Add the concurrence container to the State Machine
        smach.StateMachine.add('Landing_Con', sm_con,
                                    transitions={'lostMarker':'GoHome',
                                                'landed':'DisarmMotors'})
        # Disarm the Motors when it lands
        smach.StateMachine.add('DisarmMotors',
                                smach_ros.ServiceState('/mavros/cmd/arming',
                                            CommandBool,
                                            request = 0),
                                transitions = {'succeeded':'finished'})
    
    # Create and start the instrospection server (to use smach_viewer)
    sis = smach_ros.IntrospectionServer('smach_viewer_sm', sm, '/SM_ROOT')
    sis.start()

    #Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    sis.stop()
    rospy.spin()

    rospy.on_shutdown(plots)

if __name__ == '__main__':
    main()                     