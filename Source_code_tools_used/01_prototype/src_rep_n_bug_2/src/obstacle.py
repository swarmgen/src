import numpy as np


'''
Environment configuration

* Big map (slow) 
    MAP_SIZE_new_x = 1000
    MAP_SIZE_new_y = 1000
    MAP_BOUNDS_METER_new_x = 5.0
    MAP_BOUNDS_METER_new_y = 5.0
    
* Small map (fast)
    MAP_SIZE_new_x = 500
    MAP_SIZE_new_y = 500
    MAP_BOUNDS_METER_new_x = 2.5
    MAP_BOUNDS_METER_new_y = 2.5
'''

MAP_SIZE_new_x = 500
MAP_SIZE_new_y = 500
MAP_BOUNDS_METER_new_x = 2.5
MAP_BOUNDS_METER_new_y = 2.5

XY_START = np.array([-1.0, 1.3])
XY_GOAL = np.array([1.0, 1.3])
# XY_GOAL = np.array([1.0, 1.0-0.5])  # TEMP


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# local_variable for obs

wall_1_l_b_x = 0.0
wall_1_l_b_y = 0.0
wall_1_r_t_x = 0.3
wall_1_r_t_y = 1.5

wall_2_l_b_x = -12.5
wall_2_l_b_y = -12.5
wall_2_r_t_x = -12.5
wall_2_r_t_y = -12.5

wall_3_l_b_x = -12.5
wall_3_l_b_y = -12.5
wall_3_r_t_x = -12.5
wall_3_r_t_y = -12.5

wall_4_l_b_x = -12.5
wall_4_l_b_y = -12.5
wall_4_r_t_x = -12.5
wall_4_r_t_y = -12.5

wall_5_l_b_x = -12.5
wall_5_l_b_y = -12.5
wall_5_r_t_x = -12.5
wall_5_r_t_y = -12.5

wall_6_l_b_x = -12.5
wall_6_l_b_y = -12.5
wall_6_r_t_x = -12.5
wall_6_r_t_y = -12.5

wall_7_l_b_x = -12.5
wall_7_l_b_y = -12.5
wall_7_r_t_x = -12.5
wall_7_r_t_y = -12.5

wall_8_l_b_x = -12.5
wall_8_l_b_y = -12.5
wall_8_r_t_x = -12.5
wall_8_r_t_y = -12.5

wall_9_l_b_x = -2.5
wall_9_l_b_y = 0.0
wall_9_r_t_x = 2.5
wall_9_r_t_y = 0.03

'''
XY_START = np.array([-2.0, 2.0])
XY_GOAL = np.array([-2.5, 2.5])
direction vector 1.0, 0.0
'''

THICKNESS_WALL = 0.03

COMPLEX_OBS_T_1 = [

    # room
    # bottom
    np.array([[-MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y], [MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y],
             [MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y+THICKNESS_WALL], [-MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y+THICKNESS_WALL]]),
    # top
    np.array([[-MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y-THICKNESS_WALL], [MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y-THICKNESS_WALL],
             [MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y], [-MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y]]),
    # left
    np.array([[-MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y], [-MAP_BOUNDS_METER_new_x+THICKNESS_WALL, -MAP_BOUNDS_METER_new_y],
             [-MAP_BOUNDS_METER_new_x+THICKNESS_WALL, MAP_BOUNDS_METER_new_y], [-MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y]]),
    # right
    np.array([[MAP_BOUNDS_METER_new_x-THICKNESS_WALL, -MAP_BOUNDS_METER_new_y], [MAP_BOUNDS_METER_new_x, -MAP_BOUNDS_METER_new_y],
             [MAP_BOUNDS_METER_new_x, MAP_BOUNDS_METER_new_y], [MAP_BOUNDS_METER_new_x-THICKNESS_WALL, MAP_BOUNDS_METER_new_y]]),

    # From here, OBSTACLE
    # wall
    np.array([[wall_1_l_b_x, wall_1_l_b_y], [wall_1_r_t_x, wall_1_l_b_y], [
        wall_1_r_t_x, wall_1_r_t_y], [wall_1_l_b_x, wall_1_r_t_y]]),
    np.array([[wall_2_l_b_x, wall_2_l_b_y], [wall_2_r_t_x, wall_2_l_b_y], [
        wall_2_r_t_x, wall_2_r_t_y], [wall_2_l_b_x, wall_2_r_t_y]]),
    np.array([[wall_3_l_b_x, wall_3_l_b_y], [wall_3_r_t_x, wall_3_l_b_y], [
        wall_3_r_t_x, wall_3_r_t_y], [wall_3_l_b_x, wall_3_r_t_y]]),
    np.array([[wall_4_l_b_x, wall_4_l_b_y], [wall_4_r_t_x, wall_4_l_b_y], [
        wall_4_r_t_x, wall_4_r_t_y], [wall_4_l_b_x, wall_4_r_t_y]]),

    np.array([[wall_5_l_b_x, wall_5_l_b_y], [wall_5_r_t_x, wall_5_l_b_y], [
        wall_5_r_t_x, wall_5_r_t_y], [wall_5_l_b_x, wall_5_r_t_y]]),
    np.array([[wall_6_l_b_x, wall_6_l_b_y], [wall_6_r_t_x, wall_6_l_b_y], [
        wall_6_r_t_x, wall_6_r_t_y], [wall_6_l_b_x, wall_6_r_t_y]]),

    np.array([[wall_7_l_b_x, wall_7_l_b_y], [wall_7_r_t_x, wall_7_l_b_y], [
        wall_7_r_t_x, wall_7_r_t_y], [wall_7_l_b_x, wall_7_r_t_y]]),  # for normal
    np.array([[wall_8_l_b_x, wall_8_l_b_y], [wall_8_r_t_x, wall_8_l_b_y], [
        wall_8_r_t_x, wall_8_r_t_y], [wall_8_l_b_x, wall_8_r_t_y]]),  # for normal
    np.array([[wall_9_l_b_x, wall_9_l_b_y], [wall_9_r_t_x, wall_9_l_b_y], [
        wall_9_r_t_x, wall_9_r_t_y], [wall_9_l_b_x, wall_9_r_t_y]]),  # for normal

]

#########################################################################
# setter
# getter
#########################################################################

"""_summary_
obstacle set
"""


def set_obstacle_set(new_obstacle_set):

    COMPLEX_OBS_T_1 = new_obstacle_set


def get_obstacle_set():
    return COMPLEX_OBS_T_1


"""
XY_START
"""


def set_XY_START(new_XY_START):

    XY_START = new_XY_START


def get_XY_START():
    return XY_START


"""
XY_GOAL
"""


def set_XY_GOAL(new_XY_GOAL):

    XY_GOAL = new_XY_GOAL


def get_XY_GOAL():
    return XY_GOAL
