# Solution to the First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

## By Georgii A. Kurshakov

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
> This branch contains the Sphinx documentation.

### Required packages

-  roscpp
-  rospy
-  message_generation
-  actionlib
-  actionlib_msgs

### Description of the system

The package consists of the following nodes:

- **state_machine**
- **random_position_server**
- **go_to_point**

#### state_machine

A node managing the robot behaviour. 
It receives commands from **user_interface** via a service and implements a simple finite state machine.
It can request the **random_position_server** for a random position, send this position as a goal
for **go_to_point** action and cancel the goal if requested.

#### random_position_server

A node implementing a service for generating random positions. 
Provides the **state_machine** with random positions for **go_to_point**.

#### go_to_point

A node implementing an action for reaching the goal position using a publisher on topic */cmd_vel* and a subscriber on topic */odom*.
**state_machine** is able to send a goal and cancel it. Once the goal is cancelled, the robot stops immediately.

### Instructions to run the code

All the instructions can be found in the **interface.ipynb** Jupyter Notebook file.
