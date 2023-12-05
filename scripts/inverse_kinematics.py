from sympy import *
import math

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
T5P_symbols = dh_to_transformation_matrix_symbols(theta6,0,0.308,degree_to_radians(0))

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

jacobian_function = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],jacobian_matrix)
t0p_function = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],T0P_symbols)