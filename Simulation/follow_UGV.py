import sys # argv, argc
import datetime
import rospy
import math
import numpy as np
# Libraries with the messages structures used by ros services and topics
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import Pose, Twist, Vector3, Transform, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
# Libraries for the transformations between frames
import tf
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion
import tf2_ros
from tf2_ros import TransformListener
# Import other classes
from pid import PID

class Publisher(object):
    """ PUBLISHER / SUBSCRIBER """

    def __init__(self,ros_node):

        rosNode = ros_node

        ## Subscribers ##
        self.sub_marker = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_marker)
        self.sub_odom_jackal = rospy.Subscriber("/jackal0/jackal_velocity_controller/odom", Odometry, self.callback_odom_jackal)
        ## Publishers ##
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        ## Iris Variables ##
        self.pos = [0, 0, 0, 0, 0, 0]
        self.x_trajectory = []
        self.y_trajectory = []
        self.pose_iris = []

        self.obs = []
        self.obs_trajectory = np.array([0,0,0])
        self.hasObs = False
        self.firstObs = False

        self.maxmin_values = [-13, 13]
        self.rate = rospy.Rate(10)

        ## Jackal Variables ##
        self.jackal_vel = []
        self.pose_jackal = []
        self.isUGVmoving = False

        #Tramsforms
        self.listener = tf.TransformListener()

        ## Classes ## 
        self.pid = PID()

    def start(self):

        while( not self.check_landing() ):
            if(self.firstObs == True) :

                target_angles = self.pid.target_pose(self.obs, self.hasObs, self.isUGVmoving) 

                self.vel(target_angles)
                self.rate.sleep()

        return 'landed'
    
    def check_landing(self):

        self.get_jackal_groundtruth()
        self.get_iris_groundtruth()
        
        difference_x = abs(self.pose_iris[0]-self.pose_jackal[0])
        difference_y = abs(self.pose_iris[1]-self.pose_jackal[1])
        z = self.pose_iris[2]
        
        if(z <= 0.3):

            return True

        return False
    
        
    """
    > Markers Callback
    """
    def callback_marker(self, msg):
        
        observations = np.zeros( (len(msg.transforms) , 4) )
        i = 0
        self.hasObs = False

        # Parse of the fiducial_transforms message
        for marker in msg.transforms:

            try:
                marker_id = "fiducial_T_" + str(marker.fiducial_id)
                (trans, rot) = self.listener.lookupTransform('base_link', marker_id, rospy.Time(0))

                # Store the measurements of each marker
                observations[i] = np.array([marker.fiducial_id,
                    -trans[0],
                    trans[1],
                    trans[2] ]) #The x is pointing downwards in the camera frame, therefore it's the z

                self.hasObs = True
                self.firstObs = True
                i+=1
            
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print ("Fail", e)
        
        # Obtain the average of all the measurements taken in this iteration
        if(i!=0):       # If matrix is not empty
            average_x = sum(observations[:,1]) / len(observations)
            average_y = sum(observations[:,2]) / len(observations)
            average_z = sum(observations[:,3]) / len(observations)    

            self.obs = np.array([average_x,
                                average_y,
                                average_z])
            
    """
    > Jackal Odometry Callback
    """
    def callback_odom_jackal(self, msg):
        self.jackal_vel = np.array([msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y])

        if( self.jackal_vel[0] > 0.01 or self.jackal_vel[1] > 0.01 ):
            self.isUGVmoving = True

    """
    Functions that call the rosservice and stores the ground truth pose
    """
    def get_jackal_groundtruth(self):
        
        jackal = self.get_model_pose('jackal0')
        self.pose_jackal = np.array([jackal.position.x,
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


    def get_iris_groundtruth(self):
        iris = self.get_model_pose('iris')
        self.pose_iris = np.array([iris.position.x,
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

    """
        Ros service to retrieve the ground truth pose of a certain model in the gazebo simulation
    """
    def get_model_pose(self, model_name, relative_entity_name='world'):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            res = get_model_state(model_name, relative_entity_name)
            return res.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None


    def vel(self,target_angles):
    
        msg_iris = Twist()
        msg_iris.linear.x = target_angles[0] #-
        msg_iris.linear.y = target_angles[1]
        msg_iris.linear.z = - target_angles[2] #-

        self.pub_vel.publish(msg_iris)
        