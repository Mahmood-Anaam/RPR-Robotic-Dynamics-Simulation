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
    robot = (
        rtb.ET.Ry() *       # Revolute joint q1 about y-axis
        rtb.ET.tx(L1) *     # Fixed link of length L1 along x-axis
        rtb.ET.tx() *       # Prismatic joint q2 along x-axis
        rtb.ET.Rx() *       # Revolute joint q3 about x-axis
        rtb.ET.tx(L3)       # Fixed link of length L3 along x-axis
        )

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
    robot = RPR_robot_kinematics()
    robot = rtb.Robot(robot)

    # Time array
    t = np.linspace(0, t_total, n)

    # Quintic trajectory generation for each joint
    traj_q1 = rtb.tools.trajectory.jtraj(q_start[0], q_end[0], t)
    traj_q2 = rtb.tools.trajectory.jtraj(q_start[1], q_end[1], t)
    traj_q3 = rtb.tools.trajectory.jtraj(q_start[2], q_end[2], t)

    # Initialize arrays for torques at each time step
    Q = np.zeros((3, n))

    # Set dynamic properties for each link
    robot.links[0].m = 1  # mass of the link
    robot.links[1].m = 1
    robot.links[2].m = 1

    for i in range(n):
        # Joint positions, velocities, and accelerations at time step i
        q = np.array([traj_q1.q[i], traj_q2.q[i], traj_q3.q[i]]).reshape((1,-1))
        qd = np.array([traj_q1.qd[i], traj_q2.qd[i], traj_q3.qd[i]]).reshape((1,-1))
        qdd = np.array([traj_q1.qdd[i], traj_q2.qdd[i], traj_q3.qdd[i]]).reshape((1,-1))
        
        # Compute the torques using RNE

        Q[:, i] = robot.rne(q, qd, qdd)

    return Q
    


