#! /usr/bin/env python


##### This file is the "working file". Contains the code that has been tested and works to fulfill
##### the goals of our project.

''' NEW WAY: How to run the code:
    roslaunch cse360-final-project run.launch
'''

''' OLD WAY: How to run the code manually:
    NOTE: Make sure to have used the following command and spawned an object before running.
    rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/cse360-final-project/object.urdf -urdf -x -2 -y -4 -z 1 -model object1

    To remove the object, run:
    rosservice call /gazebo/delete_model "model_name: 'object1'"

    In webshell 1: roslaunch turtlebot_navigation_gazebo amcl_demo.launch
    In webshell 2: roslaunch cse360-final-project move_base_turtlebot_copy.launch
    In webshell 3: rosrun rviz rviz          (make sure the path_planning config is chosen)
    In webshell 4: rosrun cse360-final-project send_commands.py
        - This file will start the robot's random_walk
        - Will first print the x,y coordinates of the spawned box
'''

# THIS FILE SENDS GOALS VIA IT'S CURRENT POSITION, NOT VIA THE GLOBAL/MAP POSITION.

import sys
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

import math


# function that returns the x and y position of our spawned object
def get_object_location():
    rospy.wait_for_service("gazebo/get_model_state")

    # NOTE: if we spawn an object with a different name, this needs to change
    model_name = 'object1'
    # this is a required parameter for call to get_model_state, but can be empty
    relative_entity_name = ''

    try:
        # create the caller of the service
        model_coordinates = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        while True:
            # call the service to get the coordinates of the object we spawned with "model_name"
            resp_coordinates = model_coordinates(model_name, relative_entity_name)
            
            # We continue retrying until we succeed
            # NP: We do this because the launch file run.launch runs things in parallel, so it may
            #       take some time for the object to spawn in.
            if resp_coordinates.success:
                return [resp_coordinates.pose.position.x + 2, resp_coordinates.pose.position.y]
            rospy.sleep(1.0)

    except rospy.ServiceException as e:
        print("Service call failed: ",e)



# function that creates the goal location that will be sent to the robot's move_base_simple
def create_goal_message(x_target, y_target, theta_target, frame='map'):
    """Create a goal message in the indicated frame"""

    quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal

class NavNode(object):

    def __init__(self):
        """ Set up the node. """

        rospy.init_node('nav_node')

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.object_x = 0
        self.object_y = 0

        self.position_robot_x = 0
        self.position_robot_y = 0

        # this call gets odom info and updates robot location for print statements below
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)

    # callback from /odom, gives the robot's position during the 'searching' phase
    def callback(self, msg):
        self.position_robot_x = msg.pose.pose.position.x
        self.position_robot_y = msg.pose.pose.position.y
        # print("Robot x: ", self.position_robot_x)
        # print("Robot y: ", self.position_robot_y)


    # function that finds object location and starts random walk of robot
    def start_process(self):
        location = get_object_location()
        self.object_x = location[0]
        self.object_y = location[1]

        nav_node = NavNode()
        # loop, calling goto_point infinitely, acts as the "random walk"
        while not rospy.is_shutdown():
            # only generates goal locations within the cafeteria room
            x = random.randint(-4, 4)
            y = random.randint(-10, 7)

            nav_node.goto_point(x, y)
            rospy.sleep(1.0)


    # function that makes the robot go to the object position, this is where the functionality ends
    def go_to_object(self):
        print ("in TRACKING MODE")
        curr_object_x = self.object_x
        curr_object_y = self.object_y
        rospy.loginfo("navigating to: ({},{})".format(self.object_x, self.object_y))

        goal = create_goal_message(self.object_x, self.object_y, 0,'map')

        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()

        rospy.loginfo("Sending goal.")
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

        while(True):
            # Update the object location
            location = get_object_location()
            self.object_x = location[0]
            self.object_y = location[1]

            print ("ROBOT IS AT X = ", self.position_robot_x)
            print ("ROBOT IS AT Y = ", self.position_robot_y)
            print("Object x: ", self.object_x)
            print("Object y: ", self.object_y)

            # We send a new goal if the object has moved
            # NB: By implementation of send_goal(), this overrides any previous active goal
            if(curr_object_x != self.object_x or curr_object_y != self.object_y):
                curr_object_x = self.object_x
                curr_object_y = self.object_y
                rospy.loginfo("navigating to: ({},{})".format(self.object_x, self.object_y))

                goal = create_goal_message(self.object_x, self.object_y, 0,'map')

                rospy.loginfo("Waiting for server.")
                self.ac.wait_for_server()

                rospy.loginfo("Sending goal.")
                self.ac.send_goal(goal)
                rospy.loginfo("Goal Sent.")
            
            rospy.sleep(1.0)



    # Meat of the random walk. Sends robot to the given coordinates, notifies when goal is reached.
    def goto_point(self, x_target, y_target, theta_target=0):
        """ Move to a location relative to the robot's current position """

        #nav_node = NavNode()

        rospy.loginfo("navigating to: ({},{})".format(x_target, y_target))

        #goal = create_goal_message(x_target, y_target, theta_target, 'base_link')
        goal = create_goal_message(x_target, y_target, theta_target,'map')

        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()

        rospy.loginfo("Sending goal.")
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")
        print("1")
        # Check in after a while to see how things are going.
        rospy.sleep(1.0)
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))
        print("2")
        # Should be either "ACTIVE", "SUCCEEDED" or "ABORTED"
        state_name = actionlib.get_name_of_constant(GoalStatus,
                                                    self.ac.get_state())
        rospy.loginfo("State      : {}".format(state_name))
        print("3")

        # this loop lasts the duration of the movement from one random walk goal to the next
            # GoalStatus:
                # 0 => PENDING
                # 1 => ACTIVE
            # All other GoalStatus options are indicators to stop
        while (self.ac.get_state() == 0 or self.ac.get_state() == 1):
            location = get_object_location()
            self.object_x = location[0]
            self.object_y = location[1]

            print ("ROBOT IS AT X = ", self.position_robot_x)
            print ("ROBOT IS AT Y = ", self.position_robot_y)
            print("Object x: ", self.object_x)
            print("Object y: ", self.object_y)

            distance = math.sqrt((self.object_x - self.position_robot_x)**2 + (self.object_y - self.position_robot_y)**2)
            print ("distance = ", distance)

            print ("STATE IS: --> ", actionlib.get_name_of_constant(GoalStatus, self.ac.get_state()))

            if distance <= 5:
                print ("will make a call to go to object")
                nav_node.go_to_object()
            

            rospy.sleep(1.0)


# ORIGINAL LINE:
        # Wait until the server reports a result.
#        self.ac.wait_for_result()   # pauses here, but we want to be able to check our location here
        
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))
        print("4")
        # Should be either "SUCCEEDED" or "ABORTED"
        state_name = actionlib.get_name_of_constant(GoalStatus,
                                                    self.ac.get_state())
        rospy.loginfo("State      : {}".format(state_name))


if __name__ == "__main__":
    #get_object_location()
    nav_node = NavNode()
    #nav_node.goto_point(float(2), float(2), float(1))
    nav_node.start_process()