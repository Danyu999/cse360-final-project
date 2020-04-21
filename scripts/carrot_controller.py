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


# NOTE: The turtlebot starts at (0,0)

class Controller(object):
    def __init__(self):            
        rospy.Subscriber("/odom", Odometry, self.callback)
        self.moverosbots_object = MoveRosBots()

        self.E = 0.
        self.old_error = 0.
        self.t = 0.0
        
    def trajectory_function(self, t):
        # piecewise becomes if statements

        # I1
        if 0 <= t < 2:
            t1 = self.t - 0
            x_d = -3.25 + 1*t1 + 4.0*t1**2 - 1.375*t1**3
            y_d = 3.25 + 0*t1 - 0.25*t1**2 + 0.125*t1**3

        # I2
        elif 2 <= t < 4:
            t1 = self.t - 2
            x_d = 4 + 0.5*t1 + 1.625*t1**2 - 0.625*t1**3
            y_d = 3 + 0.5*t1 + 1.5*t1**2 - 0.5*t1**3

        # I3
        elif 4 <= t < 6.0:
            t1 = self.t - 4

            x_d = 6.5 - 0.5*t1 - 1.125*t1**2 + 0.375*t1**3
            y_d = 6 + 0.5*t1 + 1.75*t1**2 - 0.625*t1**3

        # I4
        elif 6.0 <= t < 8:
            t1 = self.t - 6.0

            x_d = 4 - 0.5*t1 - 3.75*t1**2 + 1.25*t1**3
            y_d = 9 - 0.25*t1**2 + 0.125*t1**3

        # I5
        elif 8.0 <= t < 12.0:
            t1 = self.t - 8.0

            x_d =  -2 - 0.5*t1 - 0.75*t1**2 + 0.125*t1**3
            y_d =  9 + 0.5*t1 + 0.875*t1**2 - 0.15625*t1**3

        # I6
        elif 12.0 <= t < 13:
            t1 = self.t - 12.0

            x_d =  -8 - 0.5*t1 - 16.5*t1**2 + 11*t1**3
            y_d =  15 + 0*t1 + 0.5*t1**2 - 0.5*t1**3

        # I7
        elif 13.0 <= t < 14.0:
            t1 = self.t - 13.0

            x_d =  -14 - 0.5*t1 - 16.5*t1**2 + 11*t1**3
            y_d =  15 - 0.5*t1 - 35*t1**2 + 23.5*t1**3

        # I8
        elif 14.0 <= t < 16.0:
            t1 = self.t - 14.0

            x_d =  -18 - 0.5*t1 - 3.75*t1**2 + 1.25*t1**3
            y_d =  3 + 0*t1 - 0.25*t1**2 + 0.125*t1**3

        # I9
        elif 16.0 <= t < 17.0:
            t1 = self.t - 16.0

            x_d =  -26 - 0.5*t1 - 6.5*t1**2 + 4.5*t1**3
            y_d =  3 + 0.5*t1 + 7.5*t1**2 - 5*t1**3

        # I10
        elif 17.0 <= t < 20.0:
            t1 = self.t - 17.0

            x_d =  -28.5 + 0*t1 - 0.16667*t1**2 + 0.055556*t1**3
            y_d =  6 + 0.5*t1 + 1.5*t1**2 - 0.333*t1**3

        # I11
        elif 20.0 <= t < 24.0:
            t1 = self.t - 20.0

            x_d =  -28.5 + 0.5*t1 + 0.09375*t1**2 - 0.015625*t1**3
            y_d =  12 + 0.5*t1 + 0.3125*t1**2 - 0.0625*t1**3

        # I12
        elif 24.0 <= t < 28.0:
            t1 = self.t - 24.0

            x_d =  -26 + 0.5*t1 + 0.75*t1**2 - 0.125*t1**3
            y_d =  15 + 0*t1 + 0.125*t1**2 - 0.03125*t1**3

        # I13
        elif 28.0 <= t < 32:
            t1 = self.t - 28.0

            x_d =  -20 + 0.5*t1 + 0.625*t1**2 - 0.09375*t1**3
            y_d =  15 - 0.5*t1 - 2*t1**2 + 0.34375*t1**3

        # I14
        elif 32.0 <= t < 45:
            t1 = self.t - 32.0

            x_d =  -14 + 1*t1 + 1.5625*t1**2 - 0.28125*t1**3
            y_d =  3 + 0*t1 + 0*t1**2 + 0*t1**3

        return x_d, y_d

    # after t = 5, follow different line


    def new_trajectory_function(self, t):
        # once we have info of where the object is we want to go to, set that point as x_d and y_d



    def callback(self, odometry):
        # every time we call the callback we can increase the time
        self.t += 0.0015  # called very often, so increment with small intervals

        # position of the robot
        x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        orientation_q = odometry.pose.pose.orientation
        rotation = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        (rot_x, rot_y, phi) = euler_from_quaternion(rotation)

        # if the function moves too fast, then the robot cannot follow it
        # desired point to go
        #x_d, y_d = self.trajectory_function(self.t)

        # try going to point (-2,-2)
        x_d = 2
        y_d = -2

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

        
        kpv = .1
        v = kpv * distance
        
        # points us in the right direction
        # Proportional controller for angular velocity
        kpw = 0.3  # just guessing this value for rn
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