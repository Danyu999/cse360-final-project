# Needy Robot

### About
Project for CSE 360 (Mobile Robotics) as Lehigh University
Authors: Lauren Bright and Dan Yu


### Description
  The goal of this project was to create a robot that would walk indefinitely around a defined space, avoiding obstacles along the way, until the robot was in close proximity to a target object, which the robot would then run into in a “needy” way. The name, “Needy Robot” reflects the last step of incessantly running into the target object, similar to how animals act when they need attention from their owners. This robot could be used to give potential pet owners an idea of what it’s like having an animal around, moving in random patterns around a house and begging for attention.

  The robot functions with two modes: searching mode and tracking mode. In searching mode, the robot moves around the environment, in this case an empty cafeteria, indefinitely. In tracking mode, the robot changes from randomly moving in the space, to moving towards and running into a target object in the environment. This tracking mode is initiated when the robot is within a small distance of the object. The object in this project was a blue rectangular box, randomly positioned in the environment at the start of each run.

### Specifications
* Coded in the Robot Ignite Academy environment with ROS and Python
* Used the environment provided in "ROS Navigation in 5 Days, Unit 1: Basic Concepts".
  * Robot used: a differential wheel robot, the Turtlebot

### Directory
* [launch] contains the ROS launch files for the robot
* [src] contains the Python src files
  * [send_commands.py] contains the logic for the Needy Robot's behavior
  
### Run
* Easy way:
  * roslaunch cse360-final-project run.launch

* Manual way:
  * NOTE: Make sure to have used the following command and spawned an object before running.
    * rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/cse360-final-project/object.urdf -urdf -x -2 -y -4 -z 1 -model object1
    * To remove the object, run: rosservice call /gazebo/delete_model "model_name: 'object1'"
  * In webshell 1: roslaunch turtlebot_navigation_gazebo amcl_demo.launch
  * In webshell 2: roslaunch cse360-final-project move_base_turtlebot_copy.launch
  * In webshell 3: rosrun rviz rviz          (make sure the path_planning config is chosen)
  * In webshell 4: rosrun cse360-final-project send_commands.py
