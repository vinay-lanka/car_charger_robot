#!/usr/bin/env python3

#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
from sympy import *
import math
import numpy as np
from time import sleep
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def degree_to_radians(degree):
  radians = degree * math.pi / 180
  return radians

# Parameters - Joint , ai, di, alphai
def dh_to_transformation_matrix_symbols(theta_sym,ai,di,alphai):
  a = symbols('a')
  d = symbols('d')
  theta = theta_sym
  cos_theta= cos(theta)
  sin_theta= sin(theta)
  alpha = symbols('alpha')
  cos_alpha= cos(alpha)
  sin_alpha= sin(alpha)
  Rztheta = Matrix([
    [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
    [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
    [0, sin_alpha, cos_alpha, d],
    [0, 0, 0, 1]
  ])
  dh_transform_symbols = Rztheta.subs({a: ai,d: di, alpha: alphai})
  return dh_transform_symbols

theta1 = symbols('theta1')
theta2 = symbols('theta2')
theta3 = symbols('theta3')
theta4 = symbols('theta4')
theta5 = symbols('theta5')
theta6 = symbols('theta6')

T01_symbols = dh_to_transformation_matrix_symbols(-theta1+degree_to_radians(90),0,0.245,degree_to_radians(90))
T12_symbols = dh_to_transformation_matrix_symbols(theta2+degree_to_radians(90),0.710,0.2604,degree_to_radians(-180))
T23_symbols = dh_to_transformation_matrix_symbols(-theta3,0,0.2604,degree_to_radians(-90))
T34_symbols = dh_to_transformation_matrix_symbols(-theta4,0,0.540,degree_to_radians(-90))
T45_symbols = dh_to_transformation_matrix_symbols(theta5,0,0.150,degree_to_radians(90))
T5P_symbols = dh_to_transformation_matrix_symbols(theta6,0,0.152,degree_to_radians(0))

# T01_symbols = dh_to_transformation_matrix_symbols(theta1,0,0.245,degree_to_radians(90))
# T12_symbols = dh_to_transformation_matrix_symbols(theta2,0.710,0.2604,degree_to_radians(-180))
# T23_symbols = dh_to_transformation_matrix_symbols(theta3,0,0.2604,degree_to_radians(-90))
# T34_symbols = dh_to_transformation_matrix_symbols(theta4,0,0.540,degree_to_radians(-90))
# T45_symbols = dh_to_transformation_matrix_symbols(theta5,0,0.150,degree_to_radians(90))
# T5P_symbols = dh_to_transformation_matrix_symbols(theta6,0,0.152,degree_to_radians(0))

T0P_symbols = T01_symbols * T12_symbols * T23_symbols * T34_symbols * T45_symbols * T5P_symbols

T02_symbols = T01_symbols * T12_symbols
T03_symbols = T02_symbols * T23_symbols
T04_symbols = T03_symbols * T34_symbols
T05_symbols = T04_symbols * T45_symbols
T0P_symbols = T05_symbols * T5P_symbols

P = T0P_symbols.col(-1)
P.row_del(-1)

DP_q1 = diff(P,theta1)
DP_q2 = diff(P,theta2)
DP_q3 = diff(P,theta3)
DP_q4 = diff(P,theta4)
DP_q5 = diff(P,theta5)
DP_q6 = diff(P,theta6)

Z_01 = T01_symbols.col(2)
Z_01.row_del(-1)
Z_02 = T02_symbols.col(2)
Z_02.row_del(-1)
Z_03 = T03_symbols.col(2)
Z_03.row_del(-1)
Z_04 = T04_symbols.col(2)
Z_04.row_del(-1)
Z_05 = T05_symbols.col(2)
Z_05.row_del(-1)
Z_06 = T0P_symbols.col(2)
Z_06.row_del(-1)

J1 = Matrix.vstack(DP_q1, Z_01)
J2 = Matrix.vstack(DP_q2, Z_02)
J3 = Matrix.vstack(DP_q3, Z_03)
J4 = Matrix.vstack(DP_q4, Z_04)
J5 = Matrix.vstack(DP_q5, Z_05)
J6 = Matrix.vstack(DP_q6, Z_06)

jacobian_matrix = Matrix.hstack(J1,J2,J3,J4,J5,J6)

#####TEST TRAJECTORY
radius = 0.100 #mm

trajectory = []

t1 = 0
t2 = math.radians(-30)
t3 = math.radians(-15)
t4 = 0
t5 = math.radians(-45)
t6 = 0

iterations = 50000
delta_t = 500/iterations

trajectory = []

T0P_values = np.array(N(T0P_symbols.subs({theta1: t1,theta2: t2, theta3: t3, theta4: t4,theta5: t5,theta6: t6})).tolist()).astype(np.float64)

T0P_symbols.subs(zip([theta1], [t1]))
for i in np.arange(start=0, stop=2*np.pi, step=2*np.pi/iterations):
  # local_frame_point = np.array([(radius * np.cos(i-(np.pi/2))),(radius * np.sin(i-(np.pi/2)))+0.100,0] + [1])
  local_frame_point = np.array([(radius * np.sin(i)-0.1),(radius * np.cos(i)),0] + [1])
  base_frame_point = T0P_values @ local_frame_point
  trajectory.append(base_frame_point)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter([row[0] for row in trajectory], [row[1] for row in trajectory], [row[2] for row in trajectory])
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_title('Reference Trajectory of End Effector')
# fig.set_box_aspect([np.ptp(arr) for arr in [ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]])
plt.show()


jacobian_function = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],jacobian_matrix)
t0p_function = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],T0P_symbols)

class fk_publish_node(Node):
    def __init__(self):
        super().__init__('fk_publish_node')
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_angles = np.array([0.000001,math.radians(-30),math.radians(-15),0.000001,math.radians(-45),0.0]).astype(np.float64)
        T0P_values = np.array(N(T0P_symbols.subs({theta1: self.joint_angles[0],theta2: self.joint_angles[1], theta3: self.joint_angles[2], theta4: self.joint_angles[3],theta5: self.joint_angles[4],theta6: self.joint_angles[5]})).tolist()).astype(np.float64)
        point = np.array([0,0,0,1])
        self.current_position = T0P_values @ point
        # print(self.current_position)
        self.current_rotation_mat = R.from_matrix(t0p_function(self.joint_angles[0],self.joint_angles[1],self.joint_angles[2],self.joint_angles[3],self.joint_angles[4],self.joint_angles[5])[:-1,:-1])
        self.current_rotation_rpy = self.current_rotation_mat.as_euler('zyx', degrees=False)
        self.init_rotation_rpy = self.current_rotation_rpy
        # print(self.current_rotation_rpy)
        # self.current_position = t0p_function(self.joint_angles[0],self.joint_angles[1],self.joint_angles[2],
        #                                     self.joint_angles[3],self.joint_angles[4],self.joint_angles[5])[:,3]
        self.ref_trajectory = []
    def fk_pub(self):
        for ref_position in trajectory:
            # print(self.init_rotation_rpy.as_euler('zyx', degrees=False))
            # print([self.theta1,self.theta2,self.theta3,self.theta4,self.theta5,self.theta6])
            inv_jacobian_at_point = np.linalg.pinv(jacobian_function(self.joint_angles[0],self.joint_angles[1],
                                                                     self.joint_angles[2],self.joint_angles[3],
                                                                     self.joint_angles[4],self.joint_angles[5]))
            # print(self.joint_angles)
            instant_velocity = np.array([(ref_position[0]-self.current_position[0])/delta_t, 
                                         (ref_position[1]-self.current_position[1])/delta_t,
                                         (ref_position[2]-self.current_position[2])/delta_t,
                                         (0.0),
                                         (0.0),
                                         (0.0)]).astype(np.float64)
            # instant_velocity = np.array([(ref_position[0]-self.current_position[0])/delta_t, 
            #                              (ref_position[1]-self.current_position[1])/delta_t,
            #                              (ref_position[2]-self.current_position[2])/delta_t,
            #                              (self.init_rotation_rpy[0] - self.current_rotation_rpy[0])/delta_t,
            #                              (self.init_rotation_rpy[1] - self.current_rotation_rpy[1])/delta_t,
            #                              (self.init_rotation_rpy[2] - self.current_rotation_rpy[2])/delta_t]).astype(np.float64)
            # print(self.current_rotation_rpy)
            # print(self.init_rotation_rpy)
            # print(instant_velocity)
            # print([self.init_rotation_rpy[0] - self.current_rotation_rpy[0],
            #       self.init_rotation_rpy[1] - self.current_rotation_rpy[1],
            #       self.init_rotation_rpy[2] - self.current_rotation_rpy[2]])
            # instant_velocity = np.array([(ref_position[0]-self.current_position[0])/delta_t, 
            #                              (ref_position[1]-self.current_position[1])/delta_t,
            #                              (ref_position[2]-self.current_position[2])/delta_t,
            #                              (-1.5707 - self.current_rotation_rpy[0])/delta_t,
            #                              (0.0 - self.current_rotation_rpy[1])/delta_t,
            #                              (-1.5707  - self.current_rotation_rpy[2])/delta_t]).astype(np.float64)

            instant_joint_velocity = inv_jacobian_at_point @ instant_velocity
            joint_angles_current = self.joint_angles + (instant_joint_velocity * delta_t)
            print(joint_angles_current)
            self.current_position = t0p_function(joint_angles_current[0],joint_angles_current[1],
                                                joint_angles_current[2],joint_angles_current[3],
                                                joint_angles_current[4],joint_angles_current[5])[:,3]
            self.current_rotation_mat = R.from_matrix(t0p_function(joint_angles_current[0],joint_angles_current[1],
                                                joint_angles_current[2],joint_angles_current[3],
                                                joint_angles_current[4],joint_angles_current[5])[:-1,:-1])
            self.current_rotation_rpy = self.current_rotation_mat.as_euler('zyx', degrees=False)
            # print(self.current_rotation_rpy)
            self.ref_trajectory.append(self.current_position)
            print(self.current_position)
            joint_positions = Float64MultiArray()
            
            # print([-joint_angles_current[0]+degree_to_radians(90),joint_angles_current[1]-degree_to_radians(90),
            #                       -joint_angles_current[2],-joint_angles_current[3],
            #                       joint_angles_current[4],joint_angles_current[5]])
            # joint_positions.data = [-joint_angles_current[0]+degree_to_radians(90),joint_angles_current[1]-degree_to_radians(90),
            #                       -joint_angles_current[2],-joint_angles_current[3],
            #                       joint_angles_current[4],joint_angles_current[5]]
            joint_positions.data = [joint_angles_current[0],joint_angles_current[1],
                                  joint_angles_current[2],joint_angles_current[3],
                                  joint_angles_current[4],joint_angles_current[5]]
            self.joint_position_pub.publish(joint_positions)
            self.joint_angles = joint_angles_current
            # sleep(delta_t)


def main(args=None):
    rclpy.init(args=args)
    node = fk_publish_node()
    node.fk_pub()
    ref_trajectory = node.ref_trajectory
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter([row[0] for row in ref_trajectory[1:]], [row[1] for row in ref_trajectory[1:]], [row[2] for row in ref_trajectory[1:]])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_title('Computed End Effector Trajectory Zoomed In')
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()