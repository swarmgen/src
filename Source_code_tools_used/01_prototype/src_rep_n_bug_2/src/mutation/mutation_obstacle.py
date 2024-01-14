import numpy as np

import new_tools
import obstacle


def what_is_next_mutation_operator(isImproved, previous_mutation_operator, mutation_operator_set):
    """_summary_

    Args:
        mutation_operator_set = ["Inserting", "Deleting", "Moving",
        "Changing_increasing", "Changing_decreasing", "Changing_rotating"]

        Select the next one in order of this.

    """
    current_mutation_operator = "None"

    if isImproved:
        current_mutation_operator = previous_mutation_operator

    else:
        for mutation_operator_index in range(0, len(mutation_operator_set)):
            if previous_mutation_operator == mutation_operator_set[len(mutation_operator_set) - 1]:
                current_mutation_operator = mutation_operator_set[0]

            elif previous_mutation_operator == mutation_operator_set[mutation_operator_index]:
                current_mutation_operator = mutation_operator_set[mutation_operator_index + 1]

    if current_mutation_operator == "None":
        raise ValueError(
            "current_mutation_operator is None, it should be assigned.")

    return current_mutation_operator


def making_new_obs_for_inserting(MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y):
    '''
    Inserting
    - Input: TODO
    - Output: TODO
    '''
    # TODO: AREA TO AVOID

    new_obs_size_x = 0.3
    new_obs_size_y = 0.3
    new_wall_1_l_b_x = (float(np.random.rand(
        1)) * MAP_BOUNDS_METER_new_x * 2) - MAP_BOUNDS_METER_new_x
    new_wall_1_l_b_y = (float(np.random.rand(
        1)) * MAP_BOUNDS_METER_new_y * 2) - MAP_BOUNDS_METER_new_y
    new_wall_1_r_t_x = new_wall_1_l_b_x + new_obs_size_x
    new_wall_1_r_t_y = new_wall_1_l_b_y + new_obs_size_y
    new_obstacle = np.array([[new_wall_1_l_b_x, new_wall_1_l_b_y], [new_wall_1_r_t_x, new_wall_1_l_b_y], [
        new_wall_1_r_t_x, new_wall_1_r_t_y], [new_wall_1_l_b_x, new_wall_1_r_t_y]])

    return new_obstacle


def inserting(OBSTACLE_SET, MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y):

    while True:
        new_obstacle = making_new_obs_for_inserting(
            MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y)

        if not isInAreaofAvoid(new_obstacle):
            INSERTED_OBSTACLE_SET = OBSTACLE_SET.append(new_obstacle)
            break

    return INSERTED_OBSTACLE_SET


def deleting(OBSTACLE_SET):
    """_summary_ 1) Randomly select one of OBSTACLE_SET and delete

    Args:
        OBSTACLE_SET (_type_): _description_

        random number 4~ len(OBSTACLE_SET)

    """

    import random

    idx_obs_to_be_deleted = random.randint(4, len(OBSTACLE_SET))

    SUBSTRACTED_OBSTACLE_SET = OBSTACLE_SET.pop(idx_obs_to_be_deleted)

    print(
        f"[Mut.Op: Del] We delete {idx_obs_to_be_deleted}-th obstacle. \
        Now we have {len(OBSTACLE_SET) - 1} obstacles.")

    return SUBSTRACTED_OBSTACLE_SET

    #


def mutate_obstacle(OBSTACLE_SET, MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y, mutation_type):
    '''
    mutation_obstacle
    - Input: TODO
    - Output: TODO
    '''

    if mutation_type == "Inserting":
        NEW_OBSTACLE_SET = inserting(
            OBSTACLE_SET, MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y)

    elif mutation_type == "Deleting":
        NEW_OBSTACLE_SET = deleting(OBSTACLE_SET)

    return NEW_OBSTACLE_SET


def isInAreaofAvoid(object):
    """_summary_

    Args:
        object (_type_): [x][y] [x][y] [x][y] [x][y]

    Returns:
        _type_: _description_
    """

    buffer_area_to_avoid = 0.3

    """
    Area to avoid #1: XY_GOAL
    """

    xy_goal_area_x = [obstacle.get_XY_GOAL()[0] - buffer_area_to_avoid,
                      obstacle.get_XY_GOAL()[0] + buffer_area_to_avoid]
    xy_goal_area_y = [obstacle.get_XY_GOAL()[1] - buffer_area_to_avoid,
                      obstacle.get_XY_GOAL()[1] + buffer_area_to_avoid]

    for all_four_corners in range(0, 3):
        if new_tools.isPosInArea(object[all_four_corners], xy_goal_area_x, xy_goal_area_y):
            print(
                f"[Report] New generated obstacle is in the area to avoid ({object[all_four_corners]} in [{xy_goal_area_x}, {xy_goal_area_y}]), so re-gen.")
            return True

    """
    Area to avoid #2: XY_START
    """
    xy_start_area_x = [obstacle.get_XY_START()[0] - buffer_area_to_avoid,
                       obstacle.get_XY_START()[0] + buffer_area_to_avoid]
    xy_start_area_y = [obstacle.get_XY_START()[1] - buffer_area_to_avoid,
                       obstacle.get_XY_START()[1] + buffer_area_to_avoid]

    for all_four_corners in range(0, 3):
        if new_tools.isPosInArea(object[all_four_corners], xy_start_area_x, xy_start_area_y):
            print(
                f"[Report] New generated obstacle is in the area to avoid ({object[all_four_corners]} in [{xy_start_area_x}, {xy_start_area_y}]), so re-gen.")
            return True

    return False
