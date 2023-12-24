import numpy as np
import string
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class UR3:
    def __init__(self, o0o1: float = 152, o1o2: float = 120, o2o3: float = 244, la: float = 296, lb: float = 92):
        # Constants
        self.o0o1, self.o1o2, self.o2o3, self.la, self.lb = o0o1, o1o2, o2o3, la, lb

        # DHM parameters
        self.a = [0, 0, self.o2o3]
        self.alpha = [0, np.pi/2, 0]
        self.r = [self.o0o1, -self.o1o2, 0]

        # Trajectory
        self.A, self.B = [], []
        self.v1, self.v2 = 0, 0
        self.t, self.T = 0, 0
        self.s, self.Xs, self.x, self.y, self.z = [], [], [], [], []
        self.endEffector_speed = []
        
    #Tools
    def set_param(self, param_dict, param_type="const"):
        print(f"\nSet {param_type} parameters")
        for param_name, param_value in param_dict.items():
            if param_value is not None:
                setattr(self, param_name, param_value)
                print(f"{param_name} set to {param_value}")

    def read_param(self, param_dict, param_type):
        print(f"\nRead {param_type} parameters")
        for param_name, param_value in param_dict.items():
            print(f"{param_name}\t=\t{param_value}")

    def input_param(self, param_dict, param_type):
        print(f"\nInput {param_type} parameters")
        for param_name, attr_name in param_dict.items():
            while True:
                try:
                    value = float(input(f"{param_name} = "))
                    setattr(self, attr_name, value)
                    print(f"{attr_name} set to {value}")
                    break
                except ValueError:
                    print("Error: Please enter a valid floating-point number.")

    def display_data(self,data:list[int],title:string,labels:list[string]):
        print(title)
        fig, axes = plt.subplots(1, len(labels), figsize=(5*len(labels), 3))
        for i in range(len(labels)):
            axes[i].plot(self.t, data[i], label=labels[i])
            axes[i].set_xlabel('time')
            axes[i].set_ylabel(labels[i])
            axes[i].legend()
            axes[i].set_title(f'{labels[i]} over time')

            # Add switch times as markers
            switch_times = [self.T, 2 * self.T, 3 * self.T, 4 * self.T]
            for switch_time in switch_times:
                axes[i].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)

        # Print switch times on terminal
        print("switch_times\t=\t",switch_times)

        plt.tight_layout()
        plt.show()

    # Const parameters
    def set_const_param_to_ROM(self, o0o1: float = None, o1o2: float = None, o2o3: float = None, la: float = None, lb: float = None):
        self.set_param({'o0o1': o0o1, 'o1o2': o1o2, 'o2o3': o2o3, 'la': la, 'lb': lb}, "const")

    def read_const_param(self):
        self.read_param({'o0o1': self.o0o1, 'o1o2': self.o1o2, 'o2o3': self.o2o3, 'la': self.la, 'lb': self.lb}, "const")

    def input_const_param_to_ROM(self):
        self.input_param({'o0o1': 'o0o1', 'o1o2': 'o1o2', 'o2o3': 'o2o3', 'la': 'la', 'lb': 'lb'}, "const")

    # Trajectory parameters
    def set_trajectory_param_to_ROM(self, xA: float = None, yA: float = None, zA: float = None, xB: float = None, yB: float = None, zB: float = None, V1: float = None, V2: float = None):
        self.set_param({'A': [xA, yA, zA], 'B': [xB, yB, zB], 'v1': V1, 'v2': V2}, "trajectory")

    def read_trajectory_param(self):
        self.read_param({'A': self.A, 'B': self.B, 'v1': self.v1, 'v2': self.v2}, "trajectory")

    def input_trajectory_param_to_ROM(self):
        print("\nInput trajectory parameters")
        self.A = []
        self.A.append(float(input("xA = ")))
        print("xA set to {self.A[0]}")
        self.A.append(float(input("yA = ")))
        print("yA set to {self.A[1]}")
        self.A.append(float(input("zA = ")))
        print("zA set to {self.A[2]}")

        self.B = []
        self.B.append(float(input("xB = ")))
        print("yB set to {self.B[0]}")
        self.B.append(float(input("yB = ")))
        print("yB set to {self.B[1]}")
        self.B.append(float(input("zB = ")))
        print("zB set to {self.B[2]}")

        self.v1 = float(input("v1 = "))
        print("v1 set to {self.v1}")
        self.v2 = float(input("v2 = "))
        print("v2 set to {self.v2}")
    
    # Trajectory calculation
    def traj(self,A:list[int]=None,B:list[int]=None,V1:float=None,V2:float=None):
        # Initialization
        if A==None:A=self.A
        if B==None:B=self.B
        if V1==None:V1=self.v1
        if V2==None:V2=self.v2

        self.t = 0
        s0, s1, s2 = [], [], []  # position, speed, acceleration
        xt, yt, zt = [], [], []
        self.Xs = [xt, yt, zt]
        x0, y0, z0 = [], [], []  # x, y, z position
        x1, y1, z1 = [], [], []  # x, y, z speed
        x2, y2, z2 = [], [], []  # x, y, z acceleration
        self.s, self.x, self.y, self.z = [s0, s1, s2], [x0, x1, x2], [y0, y1, y2], [z0, z1, z2]
        self.endEffector_speed = []
        q0, q1 = [], []

        dAB = np.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2 + (A[2] - B[2]) ** 2)
        self.T = dAB / (V2 + 2 * V1)
        K1 = V1 / self.T
        K2 = (V2 - V1) / self.T

        # Calculate the laws at each sampling instant
        self.t = np.linspace(0, 4 * self.T, 100)
        for ti in self.t:
            if ti < self.T:
                s2.append(K1)
                s1.append(K1 * ti)
                s0.append((1 / 2) * K1 * ti ** 2)
            elif self.T <= ti < 2 * self.T:
                s2.append(K2)
                s1.append(K2 * (ti - self.T) + V1)
                s0.append((K2 / 2) * (ti - self.T) ** 2 + V1 * (ti - self.T) + (1 / 2) * K1 * self.T ** 2)
            elif 2 * self.T <= ti < 3 * self.T:
                s2.append(-K2)
                s1.append(-K2 * (ti - 2 * self.T) + V2)
                s0.append((-K2 / 2) * (ti - 2 * self.T) ** 2 + V2 * (ti - 2 * self.T) + ((self.T ** 2) / 2) * (K2 + K1) + V1 * self.T)
            elif ti >= 3 * self.T:
                s2.append(-K1)
                s1.append(-K1 * (ti - 3 * self.T) + V1)
                s0.append((-K1 / 2) * ((ti - 3 * self.T) ** 2) + V1 * (ti - 3 * self.T) + (self.T ** 2) / 2 * K1 + self.T * (V1 + V2))

            # Calculate the operational trajectory X(s) of line segments
            xt.append(((B[0]-A[0])/dAB)*s0[-1]+A[0])
            yt.append(((B[1]-A[1])/dAB)*s0[-1]+A[1])
            zt.append(((B[2]-A[2])/dAB)*s0[-1]+A[2])

        for i in range(len(self.t)):
            # Calculate s(t), s*(t), and s**(t) for motion generation in the task space X(t)
            x0.append(((B[0] - A[0]) / dAB) * s0[i] + A[0])
            y0.append(((B[1] - A[1]) / dAB) * s0[i] + A[1])
            z0.append(((B[2] - A[2]) / dAB) * s0[i] + A[2])
            x1.append(((B[0] - A[0]) / dAB) * s1[i])
            y1.append(((B[1] - A[1]) / dAB) * s1[i])
            z1.append(((B[2] - A[2]) / dAB) * s1[i])
            x2.append(((B[0] - A[0]) / dAB) * s2[i])
            y2.append(((B[1] - A[1]) / dAB) * s2[i])
            z2.append(((B[2] - A[2]) / dAB) * s2[i])

            # Calculate the speed of the tool in the task space X(t)
            self.endEffector_speed.append(np.sqrt(x1[i] ** 2 + y1[i] ** 2 + z1[i] ** 2))

            # Calculate the q(t) and q_dot(t) trajectory for each joints
            # As we have not studied the singularity, we impose the first possibility [0]
            q0.append(self.MGI(x0[i],y0[i],z0[i])[0])
            q1.append(self.MDI(q0[i],[x1[i],y1[i],z1[i]]))
        
        return q0,q1

    def display_s(self):
        self.display_data(self.s,"\nGraph of the imposed trajectory checking the laws of evolution s(t), s*(t) and s**(t)",["s","s*","s**"])

    def display_x(self):
        self.display_data(self.x,"\nGraph of the imposed trajectory checking the laws of evolution x(t), x*(t) et x**(t)",["x","x*","x**"])

    def display_y(self):
        self.display_data(self.y,"\nGraph of the imposed trajectory checking the laws of evolution y(t), y*(t) et y**(t)",["y","y*","y**"])

    def display_z(self):
        self.display_data(self.z,"\nGraph of the imposed trajectory checking the laws of evolution z(t), z*(t) et z**(t)",["z","z*","z**"])

    def display_endEffector_speed(self):
        self.display_data([self.endEffector_speed,self.s[1]],"\nCheck speed profile of the tool against s*(t)",["endEffector_speed","s*"])

    def display_Xs(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.Xs[0], self.Xs[1], self.Xs[2], label='operational trajectory X(s)')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()
    
    def display_q_dotq(self, q, dotq):
        num_joints = len(q[0])  # Assuming q[0] contains joint positions for a given time

        fig, axes = plt.subplots(num_joints, 2, figsize=(10, 3 * num_joints))

        for i in range(num_joints):
            axes[i, 0].plot(self.t, [q_t[i] for q_t in q], label=f'Joint {i + 1}')
            axes[i, 0].set_xlabel('time')
            axes[i, 0].set_ylabel(f'q{i + 1}(t)')
            axes[i, 0].legend()
            axes[i, 0].set_title(f'q{i + 1}(t)')

            axes[i, 1].plot(self.t, [dotq_t[i] for dotq_t in dotq], label=f'Joint {i + 1}')
            axes[i, 1].set_xlabel('time')
            axes[i, 1].set_ylabel(f'dq{i + 1}(t)')
            axes[i, 1].legend()
            axes[i, 1].set_title(f'dq{i + 1}(t)')

            # Add switch times as markers
            switch_times = [self.T, 2 * self.T, 3 * self.T, 4 * self.T]
            for switch_time in switch_times:
                axes[i, 0].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)
                axes[i, 1].axvline(x=switch_time, color='r', linestyle='--', linewidth=1)

        # Print switch times on terminal
        print("switch_times\t=\t", switch_times)

        plt.tight_layout()
        plt.show()

    def MGD(self,q):
        for i in range(3):
            T = np.array([[np.cos(q[i]), -np.sin(q[i]), 0, self.a[i]],
                          [np.cos(self.alpha[i]) * np.sin(q[i]), np.cos(self.alpha[i]) * np.cos(q[i]), -np.sin(self.alpha[i]), -self.r[i] * np.sin(self.alpha[i])],
                          [np.sin(self.alpha[i]) * np.sin(q[i]), np.sin(self.alpha[i]) * np.cos(q[i]), np.cos(self.alpha[i]), self.r[i] * np.cos(self.alpha[i])],
                          [0, 0, 0, 1]])
            T0N = T0N @ T if i != 0 else T
        T0E = T0N @ np.array([[1, 0, 0, self.la], [0, 1, 0, 0], [0, 0, 1, self.lb], [0, 0, 0, 1]])
        X, Y, Z = T0E[:3, 3]
        phi,nu,psi = [np.arctan2(-T0E[1, 2], T0E[2, 2])**(2), np.arcsin(T0E[0, 2]), np.arctan2(T0E[0, 1], T0E[0, 0])]
        return X, Y, Z
    
    def MGI(self,X,Y,Z):
        z1 = []
        q1, q2, q3 = [], [], []
        all_q = []

        z1.append(np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        z1.append(-np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        z1.append(np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        z1.append(-np.sqrt(X**(2) + Y**(2) - ((self.lb - self.o1o2)**(2))))
        z2 = Z - self.o0o1
        c3 = (z1[0]**(2) + z2**(2) - self.o2o3**(2) - self.la**(2)) / (2 * self.o2o3 * self.la)
        q3.append(np.arctan2(np.sqrt(1 - c3**(2)), c3))
        q3.append(np.arctan2(np.sqrt(1 - c3**(2)), c3))
        q3.append(np.arctan2(-np.sqrt(1 - c3**(2)), c3))
        q3.append(np.arctan2(-np.sqrt(1 - c3**(2)), c3))

        for i in range (len(q3)):
            b1=self.o2o3+self.la*np.cos(q3[i])
            b2=self.la*np.sin(q3[i])

            s2=(b1*z2-b2*z1[i])/(b1**(2)+b2**(2))
            c2=(b1*z1[i]+b2*z2)/(b1**(2)+b2**(2))
            q2.append(np.arctan2(s2,c2))

            s1=(X*(-self.lb+self.o1o2)-Y*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))/((self.lb-self.o1o2)*(-self.lb+self.o1o2)-(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i]))*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))
            c1=(Y*(self.lb-self.o1o2)-X*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))/((self.lb-self.o1o2)*(-self.lb+self.o1o2)-(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i]))*(self.la*np.cos(q2[i]+q3[i])+self.o2o3*np.cos(q2[i])))
            q1.append(np.arctan2(s1,c1))

            # print([q1[i],q2[i],q3[i]])
            all_q.append([q1[i],q2[i],q3[i]])
        
        return all_q

    def Jacobian(self,q):
        return np.array([   [-self.la*np.sin(q[0])*np.cos(q[1]+q[1])+self.lb*np.cos(q[0])-self.o1o2*np.cos(q[0])-self.o2o3*np.cos(q[1])*np.sin(q[0])    ,   -self.la*np.cos(q[0])*np.sin(q[1]+q[2])-self.o2o3*np.cos(q[0])*np.sin(q[1]) ,   -self.la*np.cos(q[0])*np.sin(q[1]+q[2])  ], 
                        [ self.la*np.cos(q[0])*np.cos(q[1]+q[2])+(self.o0o1+self.o1o2-self.o2o3)*np.sin(q[0])-self.o2o3*np.cos(q[1])*np.cos(q[0])   ,   -self.la*np.sin(q[0])*np.sin(q[1]-q[2])-self.o2o3*np.sin(q[1])*np.sin(q[0]) ,   self.la*np.sin(q[0])*np.sin(q[1]-q[2])   ],
                        [ 0                                                                                                                         ,   self.la*np.cos(q[1]+q[2])+self.o2o3*np.cos(q[1])                            ,   self.la*np.cos(q[2]+q[1])                ]])

    def MDI (self, q, dotX):
        return np.dot(np.linalg.inv(self.Jacobian(q)),np.transpose(dotX)) # Returns qdot (the inverse of the Jacobian multiplied by the transpose of Xdot)