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
    
    # your code here
    E1 = rtb.ET.Ry()
    E2 = rtb.ET.tx(0.4)
    E3 = rtb.ET.tx()
    E4 = rtb.ET.tx(0.1)
    E5 = rtb.ET.Rx()
    
    ets = E1 * E2 * E3 * E4 * E5 

    robot = rtb.Robot(ets) 
    return robot

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
    # generate quintic trajectory for each joint
    # traj_q1 = # insert code here
    # traj_q2 = # insert code here
    # traj_q3 = # insert code here
    # position, velocity, acceleration can be obtained using traj.q, traj.qd, traj.qdd

    # set appropriate dynamic propoerties for each link
    # robot.links[0].m = 0 # mass of the link
    # robot.links[0].r = np.array([0, 0, 0]) # center of mass in the link frame
    # robot.links[0].I = np.array([0, 0, 0, 0, 0, 0]) # inertia tensor about the center of mass in the link frame Ixx, Iyy, Izz, Ixy, Iyz, Ixz

    # calculate joint torques using recursive Newton-Euler algorithm for each time step
    # use the robot.rne() function with appropriate joint parameters and link dynamics parameters at each time step

    # return the joint torques
    robot = RPR_robot_kinematics()
   

    # generate quintic trajectory for each joint
    traj_q1 = rtb.quintic(q_start[0], q_end[0], np.linspace(0, t_total, n)) 
    traj_q2 = rtb.quintic(q_start[1], q_end[1], np.linspace(0, t_total, n)) 
    traj_q3 = rtb.quintic(q_start[2], q_end[2], np.linspace(0, t_total, n))

    # position, velocity, acceleration can be obtained using traj.q, traj.qd, traj.qdd

    # set appropriate dynamic propoerties for each link
    robot.links[0].m = 1.0  # Mass of the first link (kg)
    robot.links[0].r = np.array([0.2, 0, 0])  # Center of mass (in the link frame)
    robot.links[0].I = np.array([0.005000, 0.015833, 0.015833, 0, 0, 0])  # Inertia tensor
    
    robot.links[2].m = 1.0  # Mass of the third link (kg)
    robot.links[2].r = np.array([0.05, 0, 0])  # Center of mass (in the link frame)
    robot.links[2].I = np.array([0.00500, 0.0033333, 0.0033333, 0, 0, 0])  # Inertia tensor
    
    gravity = np.array([0, 0, 9.81]) 
    
    robot.gravity = gravity

    # Initialize an array to store the joint torques/forces
    Q = np.zeros((3, n))

    # Loop through each time step to calculate the torques/forces
    for i in range(n):
        
        Link2 = traj_q2.q[i]
        for_q1 = 1/12 * (3 * 0.1**2 + Link2**2)
        robot.links[1].m = 1 # mass of the links
        robot.links[1].r = np.array([Link2/2, 0, 0]) # center of mass in the link frame
        robot.links[1].I = np.array([0.005000, for_q1, for_q1, 0, 0, 0]) # inertia tensor about the center of mass in the link frame Ixx, Iyy, Izz, Ixy, Iyz, Ixz
 


        # Joint positions, velocities, and accelerations at time step i
        q = np.array([traj_q1.q[i], traj_q2.q[i], traj_q3.q[i]])
        qd = np.array([traj_q1.qd[i], traj_q2.qd[i], traj_q3.qd[i]])
        qdd = np.array([traj_q1.qdd[i], traj_q2.qdd[i], traj_q3.qdd[i]])

        # Calculate the joint torques/forces using the recursive Newton-Euler algorithm
        tau = robot.rne(q, qd, qdd, gravity = gravity)
        
        # Store the computed torques/forces
        Q[:, i] = tau

    return Q
