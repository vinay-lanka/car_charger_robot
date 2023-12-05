#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class fk_publish_node(Node):
    def __init__(self):
        super().__init__('fk_publish_node')
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.fk_pub)
        self.theta1 = math.radians(0)
        self.theta2 = math.radians(-0)
        self.theta3 = math.radians(-0)
        self.theta4 = math.radians(0)
        self.theta5 = math.radians(-0)
        self.theta6 = math.radians(0)
        # self.theta1 = math.radians(78)
        # self.theta2 = math.radians(90)
        # self.theta3 = math.radians(24)
        # self.theta4 = math.radians(42)
        # self.theta5 = math.radians(-60)
        # self.theta6 = math.radians(-10)


    def fk_pub(self):
        joint_positions = Float64MultiArray()
        # joint_positions.data = [-0.24758989043619475, -0.08689624450498447, -0.03089158464277693, 0.1606936459312108, 0.02003126191680069, 0.01086032272597642]
        # joint_positions.data = [-0.2517909541181752, -0.08844823146463594, -0.03848248216788787, 0.16334272265353988, 0.02495348453073981, 0.013528997637148236]
        # joint_positions.data = [-1.5707, 1.5707, 0.0, 0.38, 0.58, 0.0]
        joint_positions.data = [self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6]
        self.joint_position_pub.publish(joint_positions)

def main(args=None):
    rclpy.init(args=args)
    node = fk_publish_node()
    node.fk_pub()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()