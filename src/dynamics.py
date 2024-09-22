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
def simulateDynamics(q_start: np.ndarray, q_end: np.ndarray, n: int, t_total: float) -> np.ndarray:
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

    # Initialize the robot kinematics model
    robot_kin = RPR_robot_kinematics()  # Generate the ETS description of the robot
    robot = rtb.ERobot(robot_kin)  # Convert to an ERobot model to simulate dynamics

    # Time steps array
    time_steps = np.linspace(0, t_total, n)

    # Generate quintic trajectory for each joint
    traj_q1 = rtb.jtraj(q_start[0], q_end[0], time_steps)  # Trajectory for joint 1 (Revolute about y-axis)
    traj_q2 = rtb.jtraj(q_start[1], q_end[1], time_steps)  # Trajectory for joint 2 (Prismatic along x-axis)
    traj_q3 = rtb.jtraj(q_start[2], q_end[2], time_steps)  # Trajectory for joint 3 (Revolute about x-axis)

    # Initialize an array to store joint torques for each time step
    Q = np.zeros((3, n))  # Shape (3, n) for the 3 joints over n time steps

    # Define robot's physical properties
    L1 = 0.4  # Length of link 1 (meters)
    L3 = 0.1  # Length of link 3 (meters)
    mass = 1.0  # Mass of each link (kg)

    # Define the centers of mass for each link
    com = np.array([
        [L1 / 2, 0, 0],  # Center of mass for link 1 (midway along x-axis)
        [0, 0, 0],       # Center of mass for prismatic link 2 (at joint)
        [L3 / 2, 0, 0]   # Center of mass for link 3 (midway along x-axis)
    ])

    # Inertia tensor for each cylindrical link
    I_cylinder = np.array([0.5 * mass * (0.1 ** 2), 0.5 * mass * (0.1 ** 2), 0.5 * mass * (0.1 ** 2)])

    # Set dynamic properties for each link
    for i in range(3):
        robot.links[i].m = mass  # Set mass for each link
        robot.links[i].r = com[i]  # Set the center of mass directly
        robot.links[i].I = I_cylinder  # Set the inertia tensor directly

    # Set gravity vector (acting in the negative z direction)
    robot.gravity = np.array([0, 0, -9.81])

    # Loop through each time step to calculate the joint torques
    for i in range(n):
        # Get the joint positions, velocities, and accelerations at the current time step
        q = np.array([traj_q1.q[i][0], traj_q2.q[i][0], traj_q3.q[i][0]])  # Extract values as scalars
        qd = np.array([traj_q1.qd[i][0], traj_q2.qd[i][0], traj_q3.qd[i][0]])  # Extract values as scalars
        qdd = np.array([traj_q1.qdd[i][0], traj_q2.qdd[i][0], traj_q3.qdd[i][0]])  # Extract values as scalars

        # Calculate joint torques using recursive Newton-Euler algorithm with gravity
        tau = robot.rne(q, qd, qdd)  # Compute torques
        Q[:, i] = tau  # Store the torques in the matrix

    # Return the joint torques matrix
    return Q
    


