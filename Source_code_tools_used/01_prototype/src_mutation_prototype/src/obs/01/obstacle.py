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

XY_START = np.array([1.5, 1.0-0.5])
XY_GOAL = np.array([1.5, -1.5-0.5])
# XY_GOAL = np.array([1.0, 1.0-0.5])  # TEMP


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# local_variable for obs

wall_1_l_b_x = -10.0
wall_1_l_b_y = -2.0
wall_1_r_t_x = -8.0
wall_1_r_t_y = 0.5

wall_2_l_b_x = 7.5
wall_2_l_b_y = -0.5
wall_2_r_t_x = 8.0
wall_2_r_t_y = 2.0

wall_3_l_b_x = -0.5 - 0.5  # +1.0 # 8.0
wall_3_l_b_y = 0.0 - 1.0  # -0.5
wall_3_r_t_x = 5.0 - 0.5  # + 1.0 # 8.5
wall_3_r_t_y = 0.3 - 1.0  # -0.5# 2.5

wall_4_l_b_x = -0.5 - 0.5  # - 0.5
wall_4_l_b_y = 0.0 - 1.0  # + 0.5
wall_4_r_t_x = -0.2 - 0.5  # - 0.5
wall_4_r_t_y = 1.0 - 1.0 + 1.5 + 0.2  # HERE

wall_5_l_b_x = -12.5
wall_5_l_b_y = -12.5
wall_5_r_t_x = -12.5
wall_5_r_t_y = -12.5

wall_6_l_b_x = -12.5
wall_6_l_b_y = -12.5
wall_6_r_t_x = -12.5
wall_6_r_t_y = -12.5

'''
XY_START = np.array([-2.0, 2.0])
XY_GOAL = np.array([-2.5, 2.5])
direction vector 1.0, 0.0
'''

THICKNESS_WALL = 0.03

COMPLEX_OBS_T_1 = [
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
             wall_6_r_t_x, wall_6_r_t_y], [wall_6_l_b_x, wall_6_r_t_y]]),  # for normal
    # np.array([[wall_6_l_b_x, wall_6_l_b_y], [wall_6_r_b_x, wall_6_r_b_y], [wall_6_r_t_x, wall_6_r_t_y], [wall_6_l_t_x, wall_6_l_t_y]]), # for waypoint

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


    # moving obstacle
    # np.array([[-2.4, 2.3], [-2.3, 2.3], [-2.3, 2.4], [-2.4, 2.4]]), # from back_old
    # np.array([[0.5, 0.0], [0.6, 0.0], [0.6, 0.1], [0.5, 0.1]]),  # from front_old

    # from far (A)
    # Spot A in fig 7
    # np.array([[-2.0, 2.0], [-1.9, 2.0], [-1.9, 2.1], [-2.0, 2.1]]),

    # for fuzzing
    np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),

    # obs2 obs[-3] l_t_r
    np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),
    # np.array([[5.6, 1.4], [5.7, 1.4], [5.7, 1.3], [5.6, 1.3]]),
    # np.array([[0.6, 1.4], [0.7, 1.4], [0.7, 1.3], [0.6, 1.3]]), #old
    # obs1 obs[-2] dia
    np.array([[999.6, 1.4], [999.5, 1.4], [999.5, 1.5], [999.6, 1.5]]),
]
