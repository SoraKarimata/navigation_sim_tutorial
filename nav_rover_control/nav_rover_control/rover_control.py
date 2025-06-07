#!/usr/bin/python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.01  # 50Hz
        self.wheel_seperation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        self.steering_track = self.wheel_seperation - 2 * self.wheel_steering_y_offset

        self.pos = np.array([0.0, 0.0, 0.0, 0.0])  # lf_str, rf_str, lb_tire, rb_tire
        self.vel = np.array([0.0, 0.0, 0.0, 0.0])  # same order

        self.pub_pos = self.create_publisher(Float64MultiArray, '/str_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/tire_controller/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)

        self.latest_twist = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.latest_twist = msg

    def timer_callback(self):
        linear_velocity = self.latest_twist.linear.x
        angular_velocity = self.latest_twist.angular.z

        if(linear_velocity > 0.1):
            linear_velocity = 5
        else:
            linear_velocity = 0

        if angular_velocity > 0.3:
            self.pos[0] = -0.5 * 1.2
            self.pos[1] = 0.5
            self.pos[2] = -0.5 * 1.2
            self.pos[3] = 0.5

            self.vel[0] = 5
            self.vel[1] = -5
            self.vel[2] = 5
            self.vel[3] = -5
        elif angular_velocity < -0.3:
            self.pos[0] = 0.5
            self.pos[1] = -0.5 * 1.2
            self.pos[2] = 0.5
            self.pos[3] = -0.5 * 1.2

            self.vel[0] = 5
            self.vel[1] = -5
            self.vel[2] = 5
            self.vel[3] = -5
        else:
            self.pos[0] = 0
            self.pos[1] = 0
            self.pos[2] = 0
            self.pos[3] = 0

            self.vel[0] = linear_velocity
            self.vel[1] = -linear_velocity
            self.vel[2] = linear_velocity
            self.vel[3] = -linear_velocity





        pos_array = Float64MultiArray(data=self.pos.tolist())
        vel_array = Float64MultiArray(data=self.vel.tolist())
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)

        self.vel[:] = 0
        self.pos[:] = 0

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
