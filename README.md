# UR3 Robot Arm Simulation

This Python program simulates the trajectory of a reduce UR3 robot arm (RRR) in 3D space using the Denavit-Hartenberg parameters and motion laws for each segment.

## Overview

The program is organized into a `UR3` class, representing the UR3 robot arm. The class contains methods for setting parameters, reading parameters, and displaying trajectory data. The main functionality includes:

### Initialization

```python
def __init__(self, o0o1: float = 152, o1o2: float = 120, o2o3: float = 244, la: float = 296, lb: float = 92)
```

- **Parameters**:
  - `o0o1`: Distance between joint 0 and joint 1.
  - `o1o2`: Distance between joint 1 and joint 2.
  - `o2o3`: Distance between joint 2 and joint 3.
  - `la`: Length of link A.
  - `lb`: Length of link B.

### Setting Parameters

```python
def set_param(self, param_dict, param_type="const")
```

- **Parameters**:
  - `param_dict`: A dictionary containing parameter names and their values.
  - `param_type`: Type of parameters, either "const" for constant or "trajectory" for trajectory-related.

### Reading Parameters

```python
def read_param(self, param_dict, param_type)
```

- **Parameters**:
  - `param_dict`: A dictionary containing parameter names and their values.
  - `param_type`: Type of parameters, either "const" for constant or "trajectory" for trajectory-related.

### Inputting Parameters

```python
def input_param(self, param_dict, param_type)
```

- **Parameters**:
  - `param_dict`: A dictionary containing parameter names and their corresponding attribute names.
  - `param_type`: Type of parameters, either "const" for constant or "trajectory" for trajectory-related.

### Displaying Data

```python
def display_data(self, data: list, title: str, labels: list[str])
```

- **Parameters**:
  - `data`: List of data to be plotted.
  - `title`: Title of the plot.
  - `labels`: List of labels for each data series.

### Setting Constant Parameters

```python
def set_const_param_to_ROM(self, o0o1: float = None, o1o2: float = None, o2o3: float = None, la: float = None, lb: float = None)
```

- **Parameters**:
  - `o0o1`, `o1o2`, `o2o3`, `la`, `lb`: Constant parameters.

### Reading Constant Parameters

```python
def read_const_param(self)
```

### Inputting Constant Parameters

```python
def input_const_param_to_ROM(self)
```

### Setting Trajectory Parameters

```python
def set_trajectory_param_to_ROM(self, xA: float = None, yA: float = None, zA: float = None, xB: float = None, yB: float = None, zB: float = None, V1: float = None, V2: float = None)
```

- **Parameters**:
  - `xA`, `yA`, `zA`: Initial position coordinates.
  - `xB`, `yB`, `zB`: Final position coordinates.
  - `V1`: Initial speed.
  - `V2`: Final speed.

### Reading Trajectory Parameters

```python
def read_trajectory_param(self)
```

### Inputting Trajectory Parameters

```python
def input_trajectory_param_to_ROM(self)
```

### Trajectory Calculation

```python
def traj(self, A: list[float] = None, B: list[float] = None, V1: float = None, V2: float = None)
```

- **Parameters**:
  - `A`: Initial position coordinates.
  - `B`: Final position coordinates.
  - `V1`: Initial speed.
  - `V2`: Final speed.
  
- **Returns**:
  - `q0`: Joint angles for each time step.
  - `q1`: Joint speeds for each time step.

### Displaying $s(t)$, $\dot{s}(t)$, $\ddot{s}(t)$ 

```python
def display_s(self)
```

### Displaying $x(t)$, $\dot{x}(t)$, $\ddot{x}(t)$ 

```python
def display_x(self)
```

### Displaying $y(t)$, $\dot{y}(t)$, $\ddot{y}(t)$ 

```python
def display_y(self)
```

### Displaying $z(t)$ $\dot{z}(t)$ $\ddot{z}(t)$ 

```python
def display_z(self)
```

### Displaying end-effector speed

```python
def display_tool_speed(self)
```

### Displaying Operational Trajectory

```python
def display_Xs(self)
```

### Displaying Joint Positions and Velocities

```python
def display_q_dotq(self, q, dotq)
```

- **Parameters**:
  - `q`: List of joint angles for each time step.
  - `dotq`: List of joint velocities for each time step.

### Foreward Kinematics (MGD)

```python
def MGD(self, q: list[float])
```

- **Parameters**:
  - `q`: Joint angles.

- **Returns**:
  - `X, Y, Z`: Cartesian coordinates of the end-effector.

### Inverse Kinematics (MGI)

```python
def MGI(self, X: float, Y: float, Z: float)
```

- **Parameters**:
  - `X, Y, Z`: Cartesian coordinates of the end-effector.

- **Returns**:
  - `all_q`: List of possible joint angle solutions for the given Cartesian coordinates.

### Jacobian Calculation

```python
def Jacobian(self,q)
```

- **Parameters**:
  - `q`: List of joint angles for a given time step.

- **Returns**:
  - `t`: Jacobian matrix.

### Motion Differential Inverse (MDI)

```python
def MDI(self, q, dotX)
```

- **Parameters**:
  - `q`: List of joint angles for a given time step.
  - `dotX`: List of Cartesian velocities for each time step.

- **Returns**:
  - Joint velocities `qdot` calculated using the inverse Jacobian and transpose of Cartesian velocities.

## Usage

1. **Initialization**: Create an instance of the `UR3` class.

   ```python
   ur3_robot = UR3()
   ```

2. **Setting Parameters**: Set constant and trajectory parameters using the appropriate methods.

   ```python
   ur3_robot.set_const_param_to_ROM(o0o1=152, o1o2=120, o2o3=244, la=296, lb=92)
   ur3_robot.set_trajectory_param_to_ROM(xA=0, yA=0, zA=0, xB=300, yB=100, zB=150, V1=50, V2=50)
   ```

3. **Trajectory Calculation**: Calculate the trajectory using the `traj` method.

   ```python
   q0, q1 = ur3_robot.traj()
   ```

4. **Displaying Trajectory**: Visualize the trajectory using the `display_Xs` method.

   ```python
   ur3_robot.display_Xs()
   ```

5. **Additional Displays**: Check other trajectory-related data using methods like `display_s`, `display_x`, `display_y`, `display_z`, `display_tool_speed`, and `display_q_dotq`.

6. **Jacobian Calculation and Motion Differential Inverse (MDI)**: Calculate the Jacobian and use the MDI method as needed.

   ```python
   jacobian = ur3_robot.Jacobian(q=q0[0])
   q_dot = ur3_robot.MDI(q=q0[0], dotX=... )  # Provide Cartesian velocities as needed
   ```

### Main example
#### main.py
```python
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
```

#### Terminal

```txt
Input const parameters
o0o1 = 152
o0o1 set to 152.0
o1o2 = 120
o1o2 set to 120.0
o2o3 = 244
o2o3 set to 244.0
la = 296
la set to 296.0
lb = 92
lb set to 92.0

Read const parameters
o0o1    =       152.0
o1o2    =       120.0
o2o3    =       244.0
la      =       296.0
lb      =       92.0

Input trajectory parameters
xA = 200
xA set to {self.A[0]}
yA = 200
yA set to {self.A[1]}
zA = 100
zA set to {self.A[2]}
xB = 300
yB set to {self.B[0]}
yB = 350
yB set to {self.B[1]}
zB = 300
zB set to {self.B[2]}
v1 = 1
v1 set to {self.v1}
v2 = 1.5
v2 set to {self.v2}

Read trajectory parameters
A       =       [200.0, 200.0, 100.0]
B       =       [300.0, 350.0, 300.0]
v1      =       1.0
v2      =       1.5

Graph of the imposed trajectory checking the laws of evolution s(t), s*(t) and s**(t)
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]

Graph of the imposed trajectory checking the laws of evolution x(t), x*(t) et x**(t)
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]

Graph of the imposed trajectory checking the laws of evolution y(t), y*(t) et y**(t)
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]

Graph of the imposed trajectory checking the laws of evolution z(t), z*(t) et z**(t)
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]

Check speed profile of the tool against s*(t)
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]
switch_times    =        [76.93092581620719, 153.86185163241439, 230.79277744862156, 307.72370326482877]

```


## Note

- The program uses the Denavit-Hartenberg parameters to model the robot arm's geometry and kinematics.
- The trajectory is calculated based on specified initial and final positions and speeds.
- Inverse kinematics methods (`MGI` and `MGD`) are provided for converting between Cartesian coordinates and joint angles.

Feel free to explore and modify the program to suit your specific use case.