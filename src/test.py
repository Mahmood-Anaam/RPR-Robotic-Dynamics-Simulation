# test.py

from dynamics import simulateDynamics
import numpy as np
import matplotlib.pyplot as plt


if __name__=='__main__':
    
    # Define initial and final joint configurations
    q_start = np.array([0.0, 0.0, 0.0])  # Starting at zero position
    q_end = np.array([np.deg2rad(90), 0.3, np.deg2rad(45)])  # Desired end position
    n = 100  # Number of time steps
    t_total = 5.0  # Total time of 5 seconds

    # Simulate dynamics
    tau = simulateDynamics(q_start, q_end, n, t_total)
    print('tau shape:',tau.shape)
    print(tau)
    
    # Time vector
    t = np.linspace(0, t_total, n)
    # Plot the joint torques/forces
    plt.figure(figsize=(12, 6))
    plt.plot(t, tau[0, :], label='Torque at Joint q1 (Revolute)')
    plt.plot(t, tau[1, :], label='Force at Joint q2 (Prismatic)')
    plt.plot(t, tau[2, :], label='Torque at Joint q3 (Revolute)')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm) / Force (N)')
    plt.title('Joint Torques and Forces over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
