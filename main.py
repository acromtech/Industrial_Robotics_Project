import os
os.system("pip3 install numpy")
os.system("pip3 install matplotlib")

from UR3 import UR3
import numpy as np

# Create an instance of the UR3 robot and initialize the q parameters
robot = UR3()
qdeg = [np.pi/2, 0, 0]

# Modify the values of the robot constants via the terminal
robot.input_const_param_to_ROM()
# or
#robot.set_const_param_to_ROM(o0o1=152, o1o2=120, o2o3=244, la=296, lb=92)  # Modify the values of the robot constants easily - parameters are optional
robot.read_const_param()  # Read the values of the robot constants

# Enter the values of points (A, B), V1, and V2 via the terminal
robot.input_trajectory_param_to_ROM()
# or
#robot.set_trajectory_param_to_ROM(xA=200, yA=200, zA=100, xB=300, yB=350, zB=300, V1=1, V2=1.5)  # Enter the values of points (A, B), V1, and V2
robot.read_trajectory_param()  # Read the values of points (A, B), V1, and V2

# Test the traj(A, B, V1, V2) function - parameters are optional
q,dotq=robot.traj()

# Display the three curves of s(t), s*(t), s**(t) (with the display of the switching times)
robot.display_s()

# Display the three curves x(t), x*(t), x**(t) (with the display of the switching times)
robot.display_x()

# Display the three curves y(t), y*(t), y**(t) (with the display of the switching times)
robot.display_y()

# Display the three curves z(t), z*(t), z**(t) (with the display of the switching times)
robot.display_z()

# Display the curve of the end-effector speed
robot.display_endEffector_speed()

# Display the trajectory X(s) in the operational space (∈R³)
robot.display_Xs()

# Display the trajectories of the curves in q(t) q point(t) for each joint (with the display of the switching times)
robot.display_q_dotq(q,dotq)
