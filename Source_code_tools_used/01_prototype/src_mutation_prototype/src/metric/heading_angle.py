import numpy as np
import input_output
# from conf import *
import os
"""

Input: robot's coordinates
    For now: only can handle 4 drones.
Output: Final Score

"""


def measure_swarm_size(p_seed, traj_robot1, traj_robot2, traj_robot3, traj_robot4):

    swarm_size_stream = []

    for index_tick in range(0, len(traj_robot1)):

        centroid = 0.25 * (traj_robot1[index_tick] + traj_robot2[index_tick] +
                           traj_robot3[index_tick] + traj_robot4[index_tick])

        dist_1 = np.linalg.norm(centroid - traj_robot1[index_tick])
        dist_2 = np.linalg.norm(centroid - traj_robot2[index_tick])
        dist_3 = np.linalg.norm(centroid - traj_robot3[index_tick])
        dist_4 = np.linalg.norm(centroid - traj_robot4[index_tick])

        swarm_size = 0.24 * (dist_1 + dist_2 + dist_3 + dist_4)

        swarm_size_stream.append(swarm_size)

    # Output
    print(
        f"[report] swarm_size_stream cal is done: len is {len(swarm_size_stream)}.")

    SWARMSIZE_PN = os.path.join("output", f"{p_seed}_swarmsize_stream.log")

    # input_output.writeFile(SWARMSIZE_PN, swarm_size_stream)
    np.savetxt(SWARMSIZE_PN, swarm_size_stream, delimiter=',', fmt='%.4f')

    return swarm_size_stream


def measure_complexity_score(p_seed, traj_robot1, traj_robot2, traj_robot3, traj_robot4):

    complexity_score_swarm_size = 0

    swarm_size_stream = measure_swarm_size(
        p_seed, traj_robot1, traj_robot2, traj_robot3, traj_robot4)

    swarm_size_diff_stream = []

    for index_tick in range(0 + 1, len(swarm_size_stream)):

        swarm_size_diff_stream.append(
            abs(swarm_size_stream[index_tick - 1] - swarm_size_stream[index_tick]))

    complexity_score_swarm_size = sum(
        swarm_size_diff_stream)/len(swarm_size_diff_stream)

    print(
        f"[report] complexity_score_swarm_size cal is done: score is {round(complexity_score_swarm_size, 6)}.")

    return complexity_score_swarm_size
