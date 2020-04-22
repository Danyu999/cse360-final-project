#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import random
from sensor_msgs.msg import LaserScan

# NOTE: The turtlebot starts at (0,0)

class Controller(object):
    def __init__(self):            
        rospy.Subscriber("/odom", Odometry, self.callback)
        # rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.callback)
        self.moverosbots_object = MoveRosBots()

        self.E = 0.
        self.old_error = 0.
        self.t = 0.0

        # False is searching mode, true is hunting mode
        self.mode = False

        
    def trajectory_function(self, t):
        # piecewise becomes if statements

        # I1
        if 5 <= self.t < 10:
            x_d = -4
            y_d = 5

        elif 15 <= self.t < 20:
            x_d = -4
            y_d = -5

        return x_d, y_d


    # def new_trajectory_function(self, t):
    #     # once we have info of where the object is we want to go to, set that point as x_d and y_d

    def callback(self, odometry):
        self.t += 0.01
        print (self.t)
        if 0 <= self.t < 5:
            self.mode = False
        elif 5 <= self.t < 10:
            self.mode = True
        elif 10 <= self.t < 15:
            self.mode = False
        elif 15 <= self.t < 20:
            self.mode = True
        else:
            self.mode = False



        if self.mode:
            # every time we call the callback we can increase the time
            self.t += 0.0015  # called very often, so increment with small intervals

            # position of the robot
            x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
            orientation_q = odometry.pose.pose.orientation
            rotation = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            (rot_x, rot_y, phi) = euler_from_quaternion(rotation)

            # if the function moves too fast, then the robot cannot follow it
            # desired point to go
            x_d, y_d = self.trajectory_function(self.t)

            # try going to point (-2,-2)
            #x_d = 2
            #y_d = -2

            # NOTE: in this area, have the trajectory function return the location on a target

            # phi desired
            phi_d = math.atan2(y_d-y, x_d-x)  # these are the components of the vector to the point_desired
            #print "x_d=%.2f y_d=%.2f phi=%.2f"%(x_d,y_d,math.degrees(phi_d))

            # Distance to the desired point
            distance = math.hypot(x_d - x, y_d - y) - 0.2  # do the - 0.1 to prevent spinning around the point

            #print(x, y)
            #print "x_d=%.2f y_d=%.2f phi=%.2f"%(x_d,y_d,math.degrees(phi_d))
            print "t=%.2f d=%.2f"%(self.t,distance)
            print "x_d=%.2f y_d=%.2f phi=%.2f"%(x_d,y_d,math.degrees(phi_d))
            print "x=%.2f y=%.2f phi=%.2f"%(x,y,math.degrees(phi))

            # we want distance to go to 0
            # if linear velocity is too high, it will keep moving around the point
            # Proportional controller for linear velocity

            
            kpv = .2
            v = kpv * distance
            
            # points us in the right direction
            # Proportional controller for angular velocity
            kpw = 0.2  # just guessing this value for rn
            new_phi = phi_d - phi
            if new_phi > math.pi:
                new_phi -= 2*math.pi

            elif new_phi < -math.pi:
                new_phi += 2*math.pi

            w = kpw * (new_phi)

            

            # Message
            twist_object = Twist();
            twist_object.linear.x = v;
            twist_object.angular.z = w;

            # Send message
            self.moverosbots_object.move_robot(twist_object)

        else: #Searching mode
            x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
            distance = math.sqrt(math.pow(x,2) + math.pow(y,2))
            if distance >= 9:

                rotation = random.uniform(.25, 1)

                # Message
                twist_object = Twist();
                twist_object.linear.x =  -.75;
                twist_object.angular.z = rotation;

                # Send message
                self.moverosbots_object.move_robot(twist_object)
            else:
                # Message
                twist_object = Twist();
                twist_object.linear.x = -1.5; #TODO: Go in the other direction so the cameras can see
                twist_object.angular.z = 0.1;

                # Send message
                self.moverosbots_object.move_robot(twist_object)


        
    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    controller = Controller()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        controller.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()