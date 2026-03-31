# File to hold the equations / algorithms described in the paper
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
    for (drone_i, drone_j) in drone_positions:
        total += delta_p_ij_k(i, j, drone_i, drone_j, lam)

    return p_max * total

def get_updated_pheromone_cell(i, j, p_cur, p_max, alpha, lam, drone_positions):
    """
    Equation 3 in the paper.
    Calculated new pheromone cell value by blending current and historical information via exponential smoothing.
    :param i & j: the (i, j) coordinate of the pheromone cell
    :param p_cur: the current value in the pheromone cell being replaced
    :param p_max: the max pheromone value allowed
    :param alpha: Variable controlling long-term memory and memory decay
    :param lam: lambda (the spatial decay factor)
    :param drone_positions: the (ik, jk) coordinates of all drones
    :return: the new value to go in the pheromone grid at (i, j)
    """
    p_new = get_p_new(i, j, lam, p_max, drone_positions)
    p_t = alpha * p_cur + (1 - alpha) * p_new
    return p_t


def get_raw_inverted_priority(Pij, epsilon):
    """
    Equation 4 in paper.
    Gets the raw reciprocal inversion of the pheromone matrix.
    R_ij = 1 / (P_ij + epsilon); where R_ij is the raw priority value for cell (i, j), P_ij is the pheromone value for cell (i, j), and epsilon is a small constant to prevent division by zero.
    :return: R_ij, the raw priority value for cell (i, j)
    """
    return 1 / (Pij + epsilon)

def normalize_priority(rank_Rij, N):
    """
    Equation 5 in paper.
    Normalize the raw priority values.
    R_ij is the raw priority value for cell (i, j). Rank_Rij is the index of R_ij in ascendingly sorted list of all matrix values.
    N is the total number of cells in the grid.
    :return: N_ij, the normalized priority value for cell (i, j)
    """
    return rank_Rij / N

def get_drone_attractive_force(Q_k, theta_k, r_k, D_max):
    """
    Equation 6 in paper.
    Get the attractive force for a drone at cell (i, j) based on the normalized priority value for that cell.
    Q_k is the normalized priority value for the cell that drone k is currently in. r_k vector from the drone to the center of cell k,
    θ_k is the angle between r_k and the drone’s previous velocity vector v_old, and D_max is the maximum diagonal distance
    between neighboring cells

    :return: F_k, the attractive force for a drone at cell (i, j)
    """
    norm = np.linalg.norm(r_k) # || r_k ||

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

    for (cell_center, Q_k) in neighbors:
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



# This paper is hella poorly/confusingly written
# they seemingly cant decide on anything lmao
# the pheromone map is initialized differently depending on which place you read (all zero vs border-to-center decay)
# They say the priority is normalized by diff functions at different points in paper
# They say that the grid is always 502x502 and later it is always 102x102
# for the alpha used in exp smoothing they say it is 0.99 and later say its 0.9, and then again later say its 0.7 all for same equation...
# they dont give us their values for certain variables used in smoothing


# From what i see iaca params will look something like below. probably missing a few
# def iaca(
#         N, # grid size; said to be 100, and also 500
#         p_max, # set to 220
#         spatial_decay, # lambda; maybe 0.9 or maybe 0.1
#         alpha, # described as equal to 0.99, 0.9, and 0.7 at diff points in paper
#         epsilon, # 10^(-30)
#         gamma, # pheromone intensity power: 4.0
#         num_drones, # variable
#         delta_v_max, # 0.3 m/s
#         velocity_smoothing, # value is never given, we are told in range of [0, 1]
#         max_steps # sim runs for 100,000 steps
# ):
#     """
#     Described flow from flowchart in the paper:
#         init pheromone matrix (border-to-center decay)
#         spawn drones at center
#         loop:
#             detect drone positions
#             update pheromone matrix
#             compute priority map
#             send direction priorities to drones
#             drones compute attractive force
#             update drone velocity
#     """

