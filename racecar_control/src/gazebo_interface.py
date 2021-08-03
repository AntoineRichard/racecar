#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from math import hypot, tan, sin, pi
from ackermann_msgs.msg import AckermannDriveStamped

class GZInterface:
    def __init__(self):
        rospy.Subscriber("~cmd_drive", AckermannDriveStamped, self.cmdCallback)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)
        self.wheel_lr_dist = rospy.get_param("~wheel_lr_dist", 0.1)
        self.wheel_fr_dist = rospy.get_param("~wheel_fr_dist", 0.325)
        self.theta_max = rospy.get_param("~theta_max", 1.0)
        self.theta_min = rospy.get_param("~theta_min", -1.0)

        self.front_left_wheel_pub = rospy.Publisher('~front_left_wheel_vel_ctrl', Float64, queue_size=1)
        self.front_right_wheel_pub = rospy.Publisher('~front_right_wheel_vel_ctrl', Float64, queue_size=1)
        self.rear_left_wheel_pub = rospy.Publisher('~rear_left_wheel_vel_ctrl', Float64, queue_size=1)
        self.rear_right_wheel_pub = rospy.Publisher('~rear_right_wheel_vel_ctrl', Float64, queue_size=1)


        self.left_steering_hinge_pub  = rospy.Publisher('~left_steering_hinge_position_ctrl', Float64, queue_size=1)
        self.right_steering_hinge_pub = rospy.Publisher('~right_steering_hinge_position_ctrl', Float64, queue_size=1)

    def cmdCallback(self, data):
        linear_speed = 2*data.drive.speed/self.wheel_radius
        theta = data.drive.steering_angle
        # left right slip differential (to be improved)
        r = self.wheel_fr_dist/(tan(theta) + 1e-9)
        rear_ratio = (r + 0.1)/(r - 0.1)

        self.rear_right_wheel_pub.publish(linear_speed*rear_ratio)
        self.rear_left_wheel_pub.publish(linear_speed*(1/rear_ratio))
        self.front_right_wheel_pub.publish(linear_speed*rear_ratio)
        self.front_left_wheel_pub.publish(linear_speed*(1/rear_ratio))

        if theta > self.theta_max:
            theta = self.theta_max
        elif theta < self.theta_min:
            theta = self.theta_min

        self.left_steering_hinge_pub.publish(theta)
        self.right_steering_hinge_pub.publish(theta)


if __name__ == '__main__':
    rospy.init_node('ackermann_gazebo_interface')
    GZInterface()
    rospy.spin()
