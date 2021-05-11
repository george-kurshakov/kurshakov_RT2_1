# Solution to the First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

## By Georgii A. Kurshakov

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.

### Required packages

-  roscpp
-  rospy
-  message_generation
-  actionlib
-  actionlib_msgs

### Description of the system

The package consists of the following nodes:

- **user_interface**
- **state_machine**
- **random_position_server**
- **go_to_point**

#### user_interface

A simple node allowing the user to communicate with the system setting the robot behaviour.
Using digit keys the user is able to make the robot reach a random goal, stop upon reaching it or stop immediately.
The node is connected to the **state_machine** via a service, passing the user commands to it.

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

To launch the simulation, please run:
```
roslaunch rt2_assignment1 sim.launch
```

Use digit keys to control the robot behavoiur:
- **0** to stop upon reaching the goal;
- **1** to start moving or set a new goal;
- any other digit to stop immediately.
