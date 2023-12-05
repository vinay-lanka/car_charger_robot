#!/usr/bin/env python3

#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from time import sleep
from scipy.spatial.transform import Rotation as R
import numpy as np

from inverse_kinematics import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class fk_publish_node(Node):
    def __init__(self):
        super().__init__('fk_publish_node')
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile)
        
        self.goal_subscription = self.create_subscription(
            Twist,
            '/coords',
            self.goal_callback,
            10)
        self.goal = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).astype(np.float64)
        self.trajectory = np.array([[]]).astype(np.float64)

        self.joint_angles = np.array([math.radians(0),math.radians(-15),math.radians(-30),math.radians(0),math.radians(-45),math.radians(0)]).astype(np.float64)
        # self.joint_angles = np.array([0.0,0.0,0.0,0.0,0.0,0.0]).astype(np.float64)
        self.current_position = t0p_function(self.joint_angles[0],self.joint_angles[1],
                                              self.joint_angles[2],self.joint_angles[3],
                                              self.joint_angles[4],self.joint_angles[5])[:,3]
        print(self.current_position)
        camera_rotation_matrix = R.from_euler("ZYX", [0, 0, -90], degrees=True)
        self.base_to_camera = np.hstack((np.vstack((camera_rotation_matrix.as_matrix(),[0,0,0])),
                                          [[-0.5],[-0.5],[1],[1]]))

        self.delta_t = 0.1
        timer_period = self.delta_t + 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.ik_iteration)
        self.move = False
        self.iteration = 0

        #Start position
        joint_positions = Float64MultiArray()
        joint_positions.data = [self.joint_angles[0],self.joint_angles[1],
                                self.joint_angles[2],self.joint_angles[3],
                                self.joint_angles[4],self.joint_angles[5]]
        self.joint_position_pub.publish(joint_positions)

        #Start orientation
        goal_orientation = R.from_euler("XYX", [90, 90, 90], degrees=True)
        # self.goal_orientation = goal_orientation.as_euler('XYX', degrees=False)
        # print(self.goal_orientation)
        self.current_rotation_mat = R.from_matrix(t0p_function(self.joint_angles[0],self.joint_angles[1],self.joint_angles[2],self.joint_angles[3],self.joint_angles[4],self.joint_angles[5])[:-1,:-1])
        self.current_rotation_rpy = self.current_rotation_mat.as_euler('xzx', degrees=False)
        self.goal_orientation = self.current_rotation_mat.as_euler('xzx', degrees=False)
        print(self.current_rotation_rpy)
        print([(self.goal_orientation[0]-self.current_rotation_rpy[0]),
            (self.goal_orientation[1]-self.current_rotation_rpy[1]),
            (self.goal_orientation[2]-self.current_rotation_rpy[2])])



    def joint_state_callback(self, msg):
        self.joint_angles[0] = msg.position[2]
        self.joint_angles[1] = msg.position[0]
        self.joint_angles[2] = msg.position[1]
        self.joint_angles[3] = msg.position[3]
        self.joint_angles[4] = msg.position[4]
        self.joint_angles[5] = msg.position[5]
        # print(self.joint_angles)
    
    def goal_callback(self, msg):
        if self.move == False:
            # goal_pose = self.base_to_camera @ np.array([msg.linear.x,msg.linear.y,msg.linear.z,1])
            goal_pose = np.array([0.15,0.45,0.5409,1])
            # print(goal_pose)
            x_arr = np.linspace(self.current_position[0], goal_pose[0], num=1000)
            y_arr = np.linspace(self.current_position[1], goal_pose[1], num=1000)
            z_arr = np.linspace(self.current_position[2], goal_pose[2], num=1000)
            r_arr = np.linspace(self.current_rotation_rpy[0], self.goal_orientation[0], num=1000)
            p_arr = np.linspace(self.current_rotation_rpy[1], self.goal_orientation[1], num=1000)
            yaw_arr = np.linspace(self.current_rotation_rpy[2], self.goal_orientation[2], num=1000)
            trajectory = np.column_stack((x_arr,y_arr,z_arr,r_arr,p_arr,yaw_arr))
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.plot([row[0] for row in trajectory[0:]], [row[1] for row in trajectory[0:]], [row[2] for row in trajectory[0:]])
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            ax.set_title('Computed End Effector Trajectory Zoomed In')
            plt.show()
            self.trajectory = trajectory
            self.move = True

    def ik_iteration(self):
        if self.move == True:
            if self.iteration < len(self.trajectory):
                # print(self.iteration)
                ref_position = self.trajectory[self.iteration]
                j = jacobian_function(self.joint_angles[0],self.joint_angles[1],
                                    self.joint_angles[2],self.joint_angles[3],
                                    self.joint_angles[4],self.joint_angles[5])
                
                damping_factor = 0.01
                w2 = (damping_factor**2)*np.eye(j.shape[0])
                w2[3,3] = 10
                w2[4,4] = 10
                w2[5,5] = 10
                j_star = np.dot(np.transpose(j),np.linalg.inv(np.dot(j,np.transpose(j))+w2))
                print(ref_position[3:])
                print(self.current_rotation_rpy)
                # instant_velocity = np.array([(0.0), 
                #                             (0.0),
                #                             (0.0),
                #                             (ref_position[3]-self.current_rotation_rpy[0]),
                #                             (ref_position[4]-self.current_rotation_rpy[1]),
                #                             (ref_position[5]-self.current_rotation_rpy[2])]).astype(np.float64)

                instant_velocity = np.array([(ref_position[0]-self.current_position[0]), 
                                            (ref_position[1]-self.current_position[1]),
                                            (ref_position[2]-self.current_position[2]),
                                            (ref_position[3]-self.current_rotation_rpy[0]),
                                            (ref_position[4]-self.current_rotation_rpy[1]),
                                            (ref_position[5]-self.current_rotation_rpy[2])]).astype(np.float64)
                # print((ref_position[3]-self.current_rotation_rpy[0]),
                #     (ref_position[4]-self.current_rotation_rpy[1]),
                #     (ref_position[5]-self.current_rotation_rpy[2]))
                print(instant_velocity)

                instant_joint_velocity = (j_star) @ instant_velocity
                joint_angles_current = self.joint_angles + (instant_joint_velocity)

                # print(joint_angles_current)
                self.current_position = t0p_function(joint_angles_current[0],joint_angles_current[1],
                                                    joint_angles_current[2],joint_angles_current[3],
                                                    joint_angles_current[4],joint_angles_current[5])[:,3]

                self.current_rotation_mat = R.from_matrix(t0p_function(self.joint_angles[0],self.joint_angles[1],self.joint_angles[2],self.joint_angles[3],self.joint_angles[4],self.joint_angles[5])[:-1,:-1])
                self.current_rotation_rpy = self.current_rotation_mat.as_euler('xzx', degrees=False)

                joint_positions = Float64MultiArray()

                joint_positions.data = [joint_angles_current[0],joint_angles_current[1],
                                    joint_angles_current[2],joint_angles_current[3],
                                    joint_angles_current[4],joint_angles_current[5]]
                self.joint_position_pub.publish(joint_positions)
                self.iteration += 1
            else:
                print("TRAJECTORY COMPLETE")
                self.move = False

def main(args=None):
    rclpy.init(args=args)
    node = fk_publish_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()