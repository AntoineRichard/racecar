#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from math import hypot, tan, sin, pi, atan2, fmod
from numpy import sign
from ackermann_msgs.msg import AckermannDriveStamped

class GZInterface:
    def __init__(self):
        rospy.Subscriber("~cmd_drive", AckermannDriveStamped, self.cmdCallback)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)
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
        linear_speed = abs(data.drive.speed)*20.0
        theta = data.drive.steering_angle

        # Bound theta
        if theta > self.theta_max:
            theta = self.theta_max
        elif theta < self.theta_min:
            theta = self.theta_min
        #Remap Theta
        theta = theta / 2.0

        r = self.wheel_fr_dist/(tan(theta) + 1e-9)
        omega = linear_speed/r
        #front_left
        v_wx = linear_speed - omega*self.wheel_lr_dist
        v_wy = omega*self.wheel_fr_dist
        vel = hypot(v_wy,v_wx)/self.wheel_radius
        steer = atan2(v_wy, v_wx)
        steer = fmod(steer+2.5*pi, pi) - pi*0.5
        #self.front_left_wheel_pub.publish(vel*sign(data.drive.speed))
        self.left_steering_hinge_pub.publish(steer)
        #front_right
        v_wx = linear_speed + omega*self.wheel_lr_dist
        v_wy = omega*self.wheel_fr_dist
        vel = hypot(v_wy,v_wx)/self.wheel_radius
        steer = atan2(v_wy, v_wx)
        steer = fmod(steer+2.5*pi, pi) - pi*0.5
        #self.front_right_wheel_pub.publish(vel*sign(data.drive.speed))
        self.right_steering_hinge_pub.publish(steer)
        #rear_left
        v_wx = linear_speed - omega*self.wheel_lr_dist
        v_wy = 0
        vel = hypot(v_wy,v_wx)/self.wheel_radius
        self.rear_left_wheel_pub.publish(vel*sign(data.drive.speed))
        #rear_right
        v_wx = linear_speed + omega*self.wheel_lr_dist
        v_wy = 0
        vel = hypot(v_wy,v_wx)/self.wheel_radius
        self.rear_right_wheel_pub.publish(vel*sign(data.drive.speed))


if __name__ == '__main__':
    rospy.init_node('ackermann_gazebo_interface')
    GZInterface()
    rospy.spin()
