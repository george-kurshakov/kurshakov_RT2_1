## 
#  @file user_interface.py
#  @brief A node implementing the user interface.
#  
#  The user can use the following digit keys to control the behaviour of the robot:
#  Key        |    Behaviour
#  ---------- | ------------
#  1          | Start reaching a new random goal
#  any other  | Stop when the current goal is reached 
#  
import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
