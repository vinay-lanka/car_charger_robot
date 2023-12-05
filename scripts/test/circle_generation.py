from inverse_kinematics import *
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

#####TEST TRAJECTORY
radius = 0.100 #mm

trajectory = []

# t1 = math.radians(-0)
# t2 = math.radians(-15)
# t3 = math.radians(-30)
# t4 = math.radians(-0)
# t5 = math.radians(-45)
# t6 = math.radians(-0)

t1 = 0
t2 = 0
t3 = 0
t4 = 0
t5 = 0
t6 = 0

iterations = 5000
delta_t = 10/iterations

trajectory = []
# start_joint_angles= t0p_function(t1,t2,t3,t4,t5,t6)
T0P_values = t0p_function(t1,t2,t3,t4,t5,t6)

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