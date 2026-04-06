# File to hold the algorithm described in the paper

# Currently just a placeholder
import numpy as np
from equations import *


# This paper is hella poorly/confusingly written
# they seemingly cant decide on anything lmao
# the pheromone map is initialized differently depending on which place you read (all zero vs border-to-center decay)
# They say the priority is normalized by diff functions at different points in paper
# They say that the grid is always 502x502 and later it is always 102x102
# for the alpha used in exp smoothing they say it is 0.99 and later say its 0.9, and then again later say its 0.7 all for same equation...
# they dont give us their values for certain variables used in smoothing


# From what i see iaca params will look something like below. probably missing a few params
def iaca(
        N, # grid size; said to be 100, and also 500
        p_max, # set to 220
        spatial_decay, # lambda; maybe 0.9 or maybe 0.1
        alpha, # described as equal to 0.99, 0.9, and 0.7 at diff points in paper
        epsilon, # 10^(-30)
        gamma, # pheromone intensity power: 4.0
        num_drones, # variable
        delta_v_max, # 0.3 m/s
        velocity_smoothing, # value is never given, we are told in range of [0, 1]
        max_steps # sim runs for 100,000 steps
):
    """
    Described flow from flowchart in the paper:
        init pheromone matrix
        spawn drones at center
        loop:
            detect drone positions
            update pheromone matrix
            compute priority map
            send direction priorities to drones
            drones compute attractive force
            update drone velocity
    """

