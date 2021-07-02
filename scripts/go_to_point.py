## 
#  @file go_to_point.py
#  @brief A node implementing goal reaching algorithm
#  @author Georgii A. Kurshakov
#  @version 1.0
#  @date 02/07/2021
#  
#! /usr/bin/env python

import rospy
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

# robot state variables
position_ = Point()
yaw_ = 0
state_ = 0
pub_ = None

#action server
act_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

## 
#  @brief Odometry callback
#  
#  @param  msg /odom message
#  @return nothing
#  
#  @details Getting current position and yaw.
#  
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

## 
#  @brief Change goal reaching state
#  
#  @param  state New state
#  @return nothing
#  
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

## 
#  @brief Keep the angle within the constraints
#  
#  @param  angle An angle to normalize
#  @return Normalized angle
#  
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

## 
#  @brief Fix the initial yaw
#  
#  @param  des_pos Desired position value
#  @return nothing
#  
def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    ang_coef = rospy.get_param("ang_vel")
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw * ang_coef
        if twist_msg.angular.z > ub_a * ang_coef:
            twist_msg.angular.z = ub_a * ang_coef
        elif twist_msg.angular.z < lb_a * ang_coef:
            twist_msg.angular.z = lb_a * ang_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

## 
#  @brief Go straight ahead reaching the desired position
#  
#  @param  des_pos Desired position value
#  @return nothing
#  
def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    lin_coef = rospy.get_param("lin_vel")
    ang_coef = rospy.get_param("ang_vel")

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3 * lin_coef
        if twist_msg.linear.x > ub_d * lin_coef:
            twist_msg.linear.x = ub_d * lin_coef

        twist_msg.angular.z = kp_a*err_yaw * ang_coef
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

## 
#  @brief Fix the final yaw
#  
#  @param  des_yaw Desired yaw value
#  @return nothing
#  
def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    ang_coef = rospy.get_param("ang_vel")
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw * ang_coef
        if twist_msg.angular.z > ub_a * ang_coef:
            twist_msg.angular.z = ub_a * ang_coef
        elif twist_msg.angular.z < lb_a * ang_coef:
            twist_msg.angular.z = lb_a * ang_coef
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
## 
#  @brief Set all the velocities to 0
#  
#  @return nothing
#  
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
  
## 
#  @brief Action implementation
#  
#  @param  goal New goal
#  @return nothing
#  
#  @details A simple state machine for reaching the goal.
#  
def planning(goal):

    global state_
    global act_s

    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta

    change_state(0)
    rate = rospy.Rate(20)
    success = True

    feedback = rt2_assignment1.msg.PositionFeedback()
    result = rt2_assignment1.msg.PositionResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            # Set velocities to 0!
            done()
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.state = "fixing initial yaw"
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position)
        elif state_ == 1:
            feedback.state = "reaching target position"
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position)
        elif state_ == 2:
            feedback.state = "fixing final yaw"
            act_s.publish_feedback(feedback)
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
  

def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.PositionAction, planning, auto_start=False)
    act_s.start()
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()
