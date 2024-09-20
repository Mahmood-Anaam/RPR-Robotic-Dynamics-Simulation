#dynamics.py

import spatialmath as sm
import roboticstoolbox as rtb
import numpy as np

# ---------- Question 4.1 ---------- #

def RPR_robot_kinematics() -> rtb.ETS:
    """
    This function returns the ETS description of the RPR robot.
    The robot has 3 joints as shown in figure 1 of dynamics.pdf.

    Returns
    -------
    robot
        an ETS object representing the RPR robot
    """

    # Define link lengths
    L1 = 0.4  # Length of the first link (in meters)
    L3 = 0.1  # Length of the third link (in meters)

    # Build the ETS sequence
    ets = (
        rtb.ET.Rz() *       # Revolute joint q1 about z-axis
        rtb.ET.tx(L1) *     # Fixed link of length L1 along x-axis
        rtb.ET.tx() *       # Prismatic joint q2 along x-axis
        rtb.ET.Rz() *       # Revolute joint q3 about z-axis
        rtb.ET.tx(L3)       # Fixed link of length L3 along x-axis
    )

    return ets
    

# ---------- Question 4.2 ---------- #
def simulateDynamics(q_start: np.ndarray, q_end: np.ndarray, n:int, t_total:float) -> np.ndarray:
    """
    This function simulates the dynamics of the RPR robot.
    Refer to the dynamics.pdf for the robot description.

    Parameters
    ----------
    q_start
        the initial joint angles [in radians/metres] given as an array of shape (3,)
    q_end
        the final joint angles [in radians/metres] given as an array of shape (3,) 
    n
        the number of time steps for the simulation
    t_total
        the total time for the trajectory [in seconds]

    Returns
    -------
    Q
        The joint torques at each time step as an array of shape (3,n)
    """
    
    # your code here
    robot = RPR_robot_kinematics()
    robot = rtb.Robot(robot)
   

    # generate quintic trajectory for each joint
    traj_q1 = rtb.jtraj(q_start[0], q_end[0], n)
    traj_q2 = rtb.jtraj(q_start[1], q_end[1], n)
    traj_q3 = rtb.jtraj(q_start[2], q_end[2], n)

    # position, velocity, acceleration can be obtained using traj.q, traj.qd, traj.qdd

    radius = 0.1  # Radius of the cylindrical links
    # Calculate the inertia tensor for each link (cylinders)
    I_cylinder = lambda m, r, h: np.array([0.5 * m * r**2, (1/12) * m * (3 * r**2 + h**2), 
                                           (1/12) * m * (3 * r**2 + h**2), 0, 0, 0])

    # set appropriate dynamic propoerties for each link

    robot.links[0].m = 1.0  # Mass of the first link (kg)
    robot.links[0].r = np.array([0, 0, 0.2])  # Center of mass (in the link frame)
    robot.links[0].I = I_cylinder(robot.links[0].m, radius, 0.4)  # Inertia tensor
    
    robot.links[1].m = 1.0  # Mass of the second link (kg)
    robot.links[1].r = np.array([0, 0, 0.05])  # Center of mass (in the link frame)
    robot.links[1].I = I_cylinder(robot.links[1].m, radius, 0.1)  # Inertia tensor
    
    robot.links[2].m = 1.0  # Mass of the third link (kg)
    robot.links[2].r = np.array([0, 0, 0.05])  # Center of mass (in the link frame)
    robot.links[2].I = I_cylinder(robot.links[2].m, radius, 0.1)  # Inertia tensor

    # Initialize an array to store the joint torques/forces
    Q = np.zeros((3, n))

    # Loop through each time step to calculate the torques/forces
    for i in range(n):
        # Joint positions, velocities, and accelerations at time step i
        q = np.array([traj_q1.q[i], traj_q2.q[i], traj_q3.q[i]]).reshape((1,-1))
        qd = np.array([traj_q1.qd[i], traj_q2.qd[i], traj_q3.qd[i]]).reshape((1,-1))
        qdd = np.array([traj_q1.qdd[i], traj_q2.qdd[i], traj_q3.qdd[i]]).reshape((1,-1))

        # Calculate the joint torques/forces using the recursive Newton-Euler algorithm
        tau = robot.rne(q, qd, qdd)
        
        # Store the computed torques/forces
        Q[:, i] = tau

    return Q



