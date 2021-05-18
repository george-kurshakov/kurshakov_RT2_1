## 
#  @file go_to_point.py
#  @brief A node implementing goal reaching algorithm
#  The main and only difference with the original file is implementing separate publishers for left and right wheel.
#  
#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_l = None
pub_r = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 100

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


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    rospy.loginfo('entering fix_yaw')
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    rospy.loginfo('needed twist received')
    pub_l.publish(twist_msg.angular.z)
    pub_r.publish(-twist_msg.angular.z)
    rospy.loginfo('velocities published')
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        rospy.loginfo('yaw fixed')
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    rospy.loginfo('entering go_ahead')
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 5.0
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_l.publish(twist_msg.linear.x + twist_msg.angular.z)
        pub_r.publish(twist_msg.linear.x - twist_msg.angular.z)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    rospy.loginfo('entering fix_final_yaw')
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_l.publish(twist_msg.angular.z)
    pub_r.publish(-twist_msg.angular.z)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    rospy.loginfo('entering done')
    pub_l.publish(0.0)
    pub_r.publish(0.0)
    
def go_to_point(req):
    rospy.loginfo('service request received')
    desired_position = Point()
    desired_position.x = req.x
    desired_position.y = req.y
    des_yaw = req.theta
    change_state(0)
    while True:
    	if state_ == 0:
            #print("calling fix_yaw")
    		fix_yaw(desired_position)
    	elif state_ == 1:
            #print("calling go_straight_ahead")
    		go_straight_ahead(desired_position)
    	elif state_ == 2:
            #print("calling fix_final_yaw")
    		fix_final_yaw(des_yaw)
    	elif state_ == 3:
            #print("calling done")
    		done()
    		break
    return True

def main():
    global pub_l
    global pub_r
    rospy.init_node('go_to_point')
    rospy.loginfo('go_to_point started')
    pub_l = rospy.Publisher('/leftwheel_vel', Float32, queue_size=1)
    pub_r = rospy.Publisher('/rightwheel_vel', Float32, queue_size=1)
    rospy.loginfo('publishers initialized')
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.loginfo('subscriber initialized')
    service = rospy.Service('/go_to_point', Position, go_to_point)
    rospy.loginfo('service initialized')
    rospy.spin()

if __name__ == '__main__':
    main()
