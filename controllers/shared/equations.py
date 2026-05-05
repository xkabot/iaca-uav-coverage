# This file is all the equations described in the paper
# This file isnt actually used when running simulations but was used to get them down before being copied in.

import numpy as np
import math


def delta_p_ij_k(i, j, i_k, j_k, lam):
    """
    Equation 1 in paper.
    Calculate the un-normalized pheromone contribution from a drone k to cell (i, j).
    :param i & j: the (i, j) coordinate of the pheromone cell
    :param i_k & j_k: the (ik, jk) are the grid coordinates of drone k
    :param lam: lambda (the spatial decay factor)
    :return: the un-normalized pheromone contribution from drone k to cell (i, j)
    """
    i_diff = abs(i - i_k)
    j_diff = abs(j - j_k)
    return lam ** max(i_diff, j_diff)


def get_p_new(i, j, lam, p_max, drone_positions):
    """
    Equation 2 in the paper
    Calculate the total new pheromone contribution for a given pheromone cell (i, j)
    :param i & j: the (i, j) coordinate of the pheromone cell
    :param lam: lambda (the spatial decay factor)
    :param p_max: the max pheromone value allowed
    :param drone_positions: the (ik, jk) coordinates of all drones
    :return: the total drone pheromone contribution for a given (i, j) pheromone cell
    """
    total = 0
    for drone_i, drone_j in drone_positions:
        total += delta_p_ij_k(i, j, drone_i, drone_j, lam)

    return p_max * total


def get_updated_pheromone_cell(p_cur, p_new, alpha):
    """
    Equation 3 in the paper.
    Blends current and new pheromone via exponential smoothing.
    Works elementwise on scalars or full numpy arrays.
    :param p_cur: current pheromone value(s)
    :param p_new: new pheromone contribution(s)
    :param alpha: smoothing factor
    :return: updated pheromone value(s)
    """
    return alpha * p_cur + (1 - alpha) * p_new


def get_raw_inverted_priority(P, epsilon, gamma):
    """
    Equation 4 in paper.
    Gets the raw reciprocal inversion of the pheromone matrix.
    R_ij = (1 / (P_ij + epsilon)) ^ gamma; where R_ij is the raw priority value for cell (i, j), P_ij is the pheromone value for cell (i, j), and epsilon is a small constant to prevent division by zero.
    The paper also mentions raising the inverse of the pheromone intensity to gamma. 
    So this function can be applied elementwise to the entire pheromone matrix to get the raw priority matrix.
    :return: R, the raw priority value for full map (using numpy arrays)
    """
    return 1 / (P + epsilon) ** gamma


def normalize_priority(ranks, rows, cols):
    """
    Equation 5 in paper.
    Normalize the raw priority values.
    R_ij is the raw priority value for cell (i, j). Rank_Rij is the index of R_ij in ascendingly sorted list of all matrix values.
    N is the total number of cells in the grid.
    Using numpy we can do it all in 1 step with the argsort trick to get the ranks of each cell in the flattened array, then reshape back to the original dimensions.
    :return: the fully normalized priority map
    """
    N = rows * cols
    return (ranks / N).reshape(rows, cols)


def get_drone_attractive_force(Q_k, theta_k, r_k, D_max):
    """
    Equation 6 in paper.
    Get the attractive force for a drone at cell (i, j) based on the normalized priority value for that cell.
    Q_k is the normalized priority value for the cell that drone k is currently in. r_k vector from the drone to the center of cell k,
    θ_k is the angle between r_k and the drone’s previous velocity vector v_old, and D_max is the maximum diagonal distance
    between neighboring cells

    :return: F_k, the attractive force for a drone at cell (i, j)
    """
    norm = np.linalg.norm(r_k)  # || r_k ||

    term1 = 0.1 * math.cos(theta_k)
    term2 = 0.1 * (norm / D_max)
    weight = Q_k + term1 + term2

    return weight * (r_k / norm)


def get_total_attracting_force(neighbors, drone_pos, v_old, D_max):
    """
    Equation 7 in paper.
    Get the total attracting force for a drone at cell by summing the individual contributions from all considered neighbors.
    neighbors is the normalized priority value for the neighboring cells. v_old is drone’s previous velocity vector. D_max is maximum diagonal distance between neighboring cells. Not sure how to calc this one...
    :return: F_a, the total attracting force for a drone
    """
    F_a = np.zeros(2, dtype=float)

    v_old_norm = np.linalg.norm(v_old)

    for cell_center, Q_k in neighbors:
        r_k = np.array(cell_center) - np.array(drone_pos)
        r_norm = np.linalg.norm(r_k)

        # angle between r_k and v_old
        if v_old_norm == 0:
            theta_k = 0.0
        else:
            cos_theta = np.dot(r_k, v_old) / (r_norm * v_old_norm)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            theta_k = math.acos(cos_theta)

        weight = Q_k + 0.1 * math.cos(theta_k) + 0.1 * (r_norm / D_max)
        F_k = weight * (r_k / r_norm)

        F_a += F_k

    return F_a


def get_velocity_adjustment(F_a, v_max):
    """
    Equation 8 in paper.
    Update the drone velocity by blending the previous velocity with the new attractive force.
    F_a is the total attractive force acting on the UAV, v_max is the maximum allowable change in speed per update cycle.
    :return: the velocity adjustment for the drone
    """
    return v_max * (F_a / np.linalg.norm(F_a))


def update_drone_velocity(v_current, F_a, v_max):
    """
    Equation 9 in paper.
    Update the drone velocity by blending the previous velocity with the new attractive force.
    :param v_current: the current velocity of the drone
    :param F_a: the total attractive force acting on the UAV
    :param v_max: the maximum allowable change in speed per update cycle
    :return: the new velocity for the drone
    """
    return v_current + get_velocity_adjustment(F_a, v_max)


def stability_aware_velocity_adjustment(v_t_minus_1, v_new, alpha):
    """
    Equation 10 in paper.
    Suppresses abrupt trajectory changes.
    :param v_t_minus_1: the prior velocity of the drone at time t-1
    :param v_new: the most recent computed velocity
    :param alpha: the smoothing factor in range [0, 1]
    :return: the filtered velocity at time t
    """
    return alpha * v_t_minus_1 + (1 - alpha) * v_new


def global_to_body_velocity(v_global, psi):
    """
    Equation 11 in paper.
    Transform a 2D global velocity vector into the drone body frame.
    :param v_global: the velocity vector in the global frame, given as an iterable like [vx, vy]
    :param psi: the yaw angle of the drone in radians
    :return: the velocity vector in the drone body frame, given as a numpy array [vx_body, vy_body]
    """
    v_global = np.asarray(v_global, dtype=float)

    rotation = np.array([
        [np.cos(psi), np.sin(psi)],
        [-np.sin(psi), np.cos(psi)]
    ])

    v_body = rotation @ v_global
    return v_body


def estimate_velocity(p_t, p_prev, dt=0.032):
    """
    Equation 12 in paper.
    Estimate velocity from two position measurements.
    :param p_t: current position [x, y] (or [x, y, z])
    :param p_prev: previous position [x, y] (or [x, y, z])
    :param dt: timestep (default 0.032 seconds)
    :return: velocity vector as numpy array
    """
    p_t = np.asarray(p_t, dtype=float)
    p_prev = np.asarray(p_prev, dtype=float)

    v = (p_t - p_prev) / dt
    return v


def desired_pitch(v_ex, v_ex_dot, k_p_v=2.0, k_d_v=0.5):
    """
    Equation 13 in paper.
    Get the desired pitch angle
    :param v_ex: body-frame x velocity error
    :param v_ex_dot: time derivative of body-frame x velocity error
    :param k_p_v: proportional gain
    :param k_d_v: derivative gain
    :return: desired pitch theta_d
    """
    return k_p_v * np.clip(v_ex, -1.0, 1.0) + k_d_v * v_ex_dot


def desired_roll(v_ey, v_ey_dot, k_p_v=2.0, k_d_v=0.5):
    """
    Equation 14 in paper.
    Get the desired roll angle.
    :param v_ey: body-frame y velocity error
    :param v_ey_dot: time derivative of body-frame y velocity error
    :param k_p_v: proportional gain
    :param k_d_v: derivative gain
    :return: desired roll phi_d
    """
    return -k_p_v * np.clip(v_ey, -1.0, 1.0) - k_d_v * v_ey_dot


def altitude_thrust(
    e_z, integral_e_z, derivative_e_z, k_p_z=10.0, k_i_z=0.0, k_d_z=5.0, F_hover=48.0
):
    """
    Equation 15 in the paper.
    Get the thrust command for altitude control.

    Didnt give use the value for k_i_z, so probably keep around 0, maybe like 0.1

    :param e_z: altitude error
    :param integral_e_z: accumulated altitude error integral
    :param derivative_e_z: altitude error derivative
    :param k_i_z: integral gain
    :param k_p_z: proportional gain
    :param k_d_z: derivative gain
    :param F_hover: hover thrust offset
    :return: the thrust command F_z
    """
    return k_p_z * e_z + k_i_z * integral_e_z + k_d_z * derivative_e_z + F_hover


def control_u_phi(e_phi, e_phi_dot, k_p_phi, k_d_phi):
    """
    Equation 16 in paper.
    Compute roll control output u_phi.
    :param e_phi: roll error
    :param e_phi_dot: time derivative of roll error
    :param k_p_phi: proportional gain
    :param k_d_phi: derivative gain
    :return: roll control output u_phi
    """
    return k_p_phi * np.clip(e_phi, -1.0, 1.0) + k_d_phi * e_phi_dot


def control_u_theta(e_theta, e_theta_dot, k_p_theta, k_d_theta):
    """
    Equation 17 in paper.
    Compute pitch control output u_theta.
    :param e_theta: pitch error
    :param e_theta_dot: time derivative of pitch error
    :param k_p_theta: proportional gain
    :param k_d_theta: derivative gain
    :return: pitch control output u_theta
    """
    return -k_p_theta * np.clip(e_theta, -1.0, 1.0) - k_d_theta * e_theta_dot


def control_u_psi(e_psi, k_p_psi):
    """
    Equation 18 in paper.
    Compute yaw control output u_psi.
    :param e_psi: yaw error
    :param k_p_psi: proportional gain
    :return: yaw control output u_psi
    """
    return k_p_psi * np.clip(e_psi, -1.0, 1.0)


def motor_m1(F_z, u_phi, u_theta, u_psi):
    """
    Equation 19 in paper.
    Compute motor 1 thrust.
    :param F_z: total thrust from altitude controller
    :param u_phi: roll control output
    :param u_theta: pitch control output
    :param u_psi: yaw control output
    :return: motor 1 thrust (clamped to [0, 600])
    """
    m1 = F_z - u_phi + u_theta + u_psi
    return np.clip(m1, 0.0, 600.0)


def motor_m2(F_z, u_phi, u_theta, u_psi):
    """
    Equation 20 in paper.
    Compute motor 2 thrust.
    :param F_z: total thrust from altitude controller
    :param u_phi: roll control output
    :param u_theta: pitch control output
    :param u_psi: yaw control output
    :return: motor 2 thrust (clamped to [0, 600])
    """
    m2 = F_z - u_phi - u_theta - u_psi
    return np.clip(m2, 0.0, 600.0)


def motor_m3(F_z, u_phi, u_theta, u_psi):
    """
    Equation 21 in paper.
    Compute motor 3 thrust.
    :param F_z: total thrust from altitude controller
    :param u_phi: roll control output
    :param u_theta: pitch control output
    :param u_psi: yaw control output
    :return: motor 3 thrust (clamped to [0, 600])
    """
    m3 = F_z + u_phi - u_theta + u_psi
    return np.clip(m3, 0.0, 600.0)


def motor_m4(F_z, u_phi, u_theta, u_psi):
    """
    Equation 22 in paper.
    Compute motor 4 thrust.
    :param F_z: total thrust from altitude controller
    :param u_phi: roll control output
    :param u_theta: pitch control output
    :param u_psi: yaw control output
    :return: motor 4 thrust (clamped to [0, 600])
    """
    m4 = F_z + u_phi + u_theta - u_psi
    return np.clip(m4, 0.0, 600.0)
