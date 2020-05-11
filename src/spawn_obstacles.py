#! /usr/bin/env python

import subprocess
import random

def main():
    object_name = 'object1'

    # First, we delete the object if it already exists
    args = ['rosservice', 'call', '/gazebo/delete_model', "model_name: " + object_name]
    out = subprocess.check_output(args)
    print(out)

    # Spawn in the new object
    # TODO: Randomize the coordinates
    file_path = '/home/user/catkin_ws/src/cse360-final-project/object.urdf'
    x = str(random.randint(-3, 3))
    y = str(random.randint(-9, -5))
    z = '0'
    args = ['rosrun', 'gazebo_ros', 'spawn_model', '-file', file_path, '-urdf', '-x', x, '-y', y, '-z', z, '-model', object_name]
    out = subprocess.check_output(args)
    print(out)


if __name__ == "__main__":
    main()