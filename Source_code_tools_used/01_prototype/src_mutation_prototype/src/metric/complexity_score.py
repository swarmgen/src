import numpy as np
import metric.heading_angle as heading_angle


def isImproved():
    return True


def measure_complexity_score(p_seed, metric_type, traj_robot_1, traj_robot_2, traj_robot_3, traj_robot_4):
    """_summary_
    * measure_complexity_score is for 
    - heading_angle
    """
    if metric_type == "heading_angle":
        complexity_score = heading_angle.measure_complexity_score(
            p_seed, traj_robot_1, traj_robot_2, traj_robot_3, traj_robot_4)
        return complexity_score


def distance_between_dcc(comparison_1, comparison_2, mode):
    # logging.debug("distance_between_dcc starting")
    distance = 0
    temp_element_wise = 0
    sum_temp_1 = 0
    sum_temp_2 = 0
    sum_temp_3 = 0

    start_tick = 1

    end_tick = 100  # TODO: 100 -> 200 -> interpoliation: current: did it outside
    # logging.debug("distance_between_dcc starting 1-2: len(comparison_1):[%i], len(comparison_1)[%i]" %(len(comparison_1), len(comparison_2)))
    if len(comparison_1) <= len(comparison_2):
        end_tick = len(comparison_1)
    else:
        end_tick = len(comparison_2)
    # logging.debug("distance_between_dcc starting 2-2")

    num_object = 8
    if mode == 'normal':
        # logging.debug("distance_between_dcc starting 2-1")
        for tick_index in range(start_tick, end_tick):
            for object_index in range(0, num_object):
                temp_temp = math.pow((comparison_1[tick_index][object_index] -
                                      comparison_2[tick_index][object_index]), 2)
                temp_element_wise += temp_temp

        distance = math.sqrt(temp_element_wise)

    elif mode == 'ncc':
        # logging.debug("distance_between_dcc starting 2-2")
        for tick_index in range(start_tick, end_tick):
            for object_index in range(0, num_object):
                temp_temp_1 = comparison_1[tick_index][object_index] * \
                    comparison_2[tick_index][object_index]
                sum_temp_1 += temp_temp_1

                temp_temp_2 = math.pow(
                    comparison_1[tick_index][object_index], 2)
                sum_temp_2 += temp_temp_2

                temp_temp_3 = math.pow(
                    comparison_2[tick_index][object_index], 2)
                sum_temp_3 += temp_temp_3

        distance = sum_temp_1 / math.sqrt(sum_temp_2 * sum_temp_3)

    return distance
