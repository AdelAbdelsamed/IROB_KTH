#! /usr/bin/env python3

"""
    # {Adel Khaled Mostafa Abdelsamed}
    # {akmab@kth.se}
"""
import numpy as np

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    # Solution follows the algebraic solution in Intro to Robotics, Chapter 4.4
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35
    x_tilde = x - l0

    # Define q3 (translational joint)
    q[2] = z

    # Define q2 (rotational joint)
    c2 = (np.power(x_tilde,2) + np.power(y,2) - np.power(l1,2) - np.power(l2,2))/(2*l1*l2)
    # Check if solution exists c2 in [-1, 1]
    if np.abs(c2) > 1:
        raise Exception("Solution does not exist!")
    # Here multiple solutions exist; i choose thepositive one 
    s2 = np.sqrt(1 - c2**2)
    q[1] = np.arctan2(s2, c2)

    # Define q1 (rotational joint)
    k1 = l1 + l2*c2
    k2 = l2*s2
    q[0] = np.arctan2(y, x_tilde) - np.arctan2(k2, k1)
    
    return q


############################################### Assignment 2 ############################################### 

# Define the system parameters
L = 0.4
M = 0.39
l_e = 0.078
l_b = 0.311

# Define DH parameters from assignment page
dh_params = [
    (np.pi/2, 0, 0),
    (-np.pi/2, 0, 0),
    (-np.pi/2, L, 0),
    (np.pi/2, 0, 0),
    (np.pi/2, M, 0),
    (-np.pi/2, 0, 0),
    (0, 0, 0)
]

# Helper Functions
def compute_transformation_matrix(alpha, d, a, theta):
    # 1. Compute the sines and cosines
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # 2. Return Transformation matrix from Frame i to Frame i-1
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def compute_orientation_error(Re, Rd):
    # Here we follow equation (3.85) in R-MPC p.139
    e_o = np.zeros(3)
    
    for i in range(3):
        c = np.cross(Re[:,i], Rd[:,i])
        e_o += c

    return 0.5*e_o

def kuka_compute_transformation(dh_params, joint_variables):
    # Initial transformation matrix to start the recursion    
    T = np.eye(4)
    R = np.eye(3)

    # Obtain z_i and tilde_p_i for i = 0 ... 6
    z_i = []
    z_i.append(np.array([[0],[0],[1]])) # Add z_0
    tilde_p_i = []
    tilde_p_i.append(np.array([[0],[0],[0],[1]])) # Add tilde_p_0

    i = 1
    # Compute the transformation matrix in a recursive fashion and obtain z_i's and p_i's for Jacobian computation
    for (alpha, d, a), q in zip(dh_params, joint_variables):
        Ti = compute_transformation_matrix(alpha, d, a, q[0])
        Ri = Ti[0:3,0:3]

        R = np.dot(R, Ri)
        T = np.dot(T, Ti)

        if i <= 6:
            z_i.append( np.dot(R, z_i[0]) )
            tilde_p_i.append( np.dot(T, tilde_p_i[0]) )
    
        i = i+1

    # Compute pe
    pe = T[0:3, 3]

    # Compute the Jacobian
    J = np.zeros((6, 7))
    for col in range(J.shape[1]):
        J[:3, col] = np.cross(z_i[col][:,0], pe - tilde_p_i[col][:3, 0])
        J[3:, col] = z_i[col].flatten()

    return T, J


def kuka_FK(q):
    # 1. Compute the transformation matrix to get the position and rotation matrices
    T, J = kuka_compute_transformation(dh_params, q)

    # 2. Extract the rotation matrix
    R = T[0:3,0:3]

    # Extract and compute the position of end-effector by accounting for base frame and end-effector frame
    # Note frame 7 does not coincide with end-effector frame! 
    tilde_p = np.dot(T, np.array([0, 0, l_e, 1])) + np.array([0, 0, l_b, 0])
    p = tilde_p[:3] 

    return p, R, J

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    # Define the error tolerance in pose epsilon_pose
    epsilon_pose_tol = 1e-8

    # Define the desired position
    p_d = np.array(point)
    # Define the desired orientation by defining the desired R-matrix
    R_d = np.array(R)

    # Initial estimate of joint angles (q_hat)
    q_hat = np.array(q).reshape(-1,1)

    while True:
        # 1. Obtain the position, orientation  and Jacobian
        p_e, R_e, J = kuka_FK(q_hat)
        
        # 2. Compute the error position and orientation (here we follow 3.7 in R-MPC)
        e_p = p_e - p_d
        e_o = compute_orientation_error(R_e, R_d)
        epsilon_pose = np.hstack((e_p, e_o)).reshape(-1, 1)

        # 3. Compute the error in the joint variables 
        epsilon_q = np.dot(np.linalg.pinv(J) , epsilon_pose)

        # 4. Update the joint variables estimate
        q_hat = q_hat - epsilon_q

        # 5. Check termination condition: Norm of the pose error vector less than tolerance defined
        if np.linalg.norm(epsilon_pose) <= epsilon_pose_tol:
            break

    # Transform q_hat back into a list
    q = q_hat.flatten().tolist()

    return q



    
