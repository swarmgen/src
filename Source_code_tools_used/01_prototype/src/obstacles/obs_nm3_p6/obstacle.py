import numpy as np



# local_variable for obs
# bottom
wall_1_l_b_x = -5.0
wall_1_l_b_y = -4.0
wall_1_r_t_x = 5.0
wall_1_r_t_y = -3.8
# right wall
wall_2_l_b_x = 4.8
wall_2_l_b_y = -4.8
wall_2_r_t_x = 5.0
wall_2_r_t_y = 5.0
# left wall
wall_3_l_b_x = -5.0
wall_3_l_b_y = -5.0
wall_3_r_t_x = -4.8
wall_3_r_t_y = 5.0
# left wall of the door
wall_4_l_b_x = -5.0 
wall_4_l_b_y = 4.8
wall_4_r_t_x = -3.0
wall_4_r_t_y = 5.0
# right wall of the door
wall_5_l_b_x = -1.0#-2.0
wall_5_l_b_y = 4.8
wall_5_r_t_x = 5.0
wall_5_r_t_y = 5.0#0.2

wall_6_l_b_x = 12.5#4.0#2.0
wall_6_l_b_y = 12.5#-2.5#-3.0
wall_6_r_t_x = 12.5#4.2#2.2
wall_6_r_t_y = 12.5#2.0#1.0

'''
XY_START = np.array([-2.0, 2.0])
XY_GOAL = np.array([-2.5, 2.5])
direction vector 1.0, 0.0
'''
COMPLEX_OBS_T_1 = [
    # wall
    np.array([[wall_1_l_b_x, wall_1_l_b_y], [wall_1_r_t_x, wall_1_l_b_y], [wall_1_r_t_x, wall_1_r_t_y], [wall_1_l_b_x, wall_1_r_t_y]]),
    np.array([[wall_2_l_b_x, wall_2_l_b_y], [wall_2_r_t_x, wall_2_l_b_y], [wall_2_r_t_x, wall_2_r_t_y], [wall_2_l_b_x, wall_2_r_t_y]]),
    np.array([[wall_3_l_b_x, wall_3_l_b_y], [wall_3_r_t_x, wall_3_l_b_y], [wall_3_r_t_x, wall_3_r_t_y], [wall_3_l_b_x, wall_3_r_t_y]]),
    np.array([[wall_4_l_b_x, wall_4_l_b_y], [wall_4_r_t_x, wall_4_l_b_y], [wall_4_r_t_x, wall_4_r_t_y], [wall_4_l_b_x, wall_4_r_t_y]]),
    np.array([[wall_5_l_b_x, wall_5_l_b_y], [wall_5_r_t_x, wall_5_l_b_y], [wall_5_r_t_x, wall_5_r_t_y], [wall_5_l_b_x, wall_5_r_t_y]]),
    np.array([[wall_6_l_b_x, wall_6_l_b_y], [wall_6_r_t_x, wall_6_l_b_y], [wall_6_r_t_x, wall_6_r_t_y], [wall_6_l_b_x, wall_6_r_t_y]]), # for normal
    # np.array([[wall_6_l_b_x, wall_6_l_b_y], [wall_6_r_b_x, wall_6_r_b_y], [wall_6_r_t_x, wall_6_r_t_y], [wall_6_l_t_x, wall_6_l_t_y]]), # for waypoint

    # room
    # bottom
    np.array([[-5.0, -5.0], [5.0, -5.0], [5.0, -4.97], [-5.0, -4.97]]),
    # top
    np.array([[-5.0, 4.97], [5.0, 4.97], [5.0, 5.0], [-5.0, 5.0]]),
    # left
    np.array([[-5.0, -4.97], [-4.97, -4.97], [-4.97, 4.97], [-5.0, 4.97]]),
    # right
    np.array([[4.97, -4.97], [5.0, -4.97], [5.0, 4.97], [4.97, 4.97]]),

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


'''COMPLEX_OBS_T_basic_4 (way point, hollowed mid only in running time)

wall_1_l_b_x = -3.0
wall_1_l_b_y = -0.5
wall_1_r_t_x = -2.5
wall_1_r_t_y = 2.5

wall_2_l_b_x = -2.5
wall_2_l_b_y = -2.5
wall_2_r_t_x = -2.5
wall_2_r_t_y = -2.5

wall_3_l_b_x = 2.5
wall_3_l_b_y = 0.0
wall_3_r_t_x = 3.0
wall_3_r_t_y = 2.5

wall_4_l_b_x = -2.5
wall_4_l_b_y = -2.5
wall_4_r_t_x = -2.5
wall_4_r_t_y = -2.5

wall_5_l_b_x = -2.5
wall_5_l_b_y = -2.5
wall_5_r_t_x = -2.5
wall_5_r_t_y = -2.5

wall_6_l_b_x = -2.0
wall_6_l_b_y = -2.5
wall_6_r_b_x = 2.0
wall_6_r_b_y = -2.5
wall_6_l_t_x = 0.0
wall_6_l_t_y = 0.0
wall_6_r_t_x = 0.1
wall_6_r_t_y = 0.0

'''


'''COMPLEX_OBS_T_basic_3 (wind effect, hollowed mid)
wall_1_l_b_x = -3.0
wall_1_l_b_y = -0.5
wall_1_r_t_x = -2.5
wall_1_r_t_y = 2.5

wall_2_l_b_x = -2.5
wall_2_l_b_y = -2.5
wall_2_r_t_x = -2.5
wall_2_r_t_y = -2.5

wall_3_l_b_x = 2.5
wall_3_l_b_y = 0.0
wall_3_r_t_x = 3.0
wall_3_r_t_y = 2.5

wall_4_l_b_x = -2.5
wall_4_l_b_y = -2.5
wall_4_r_t_x = -2.5
wall_4_r_t_y = -2.5

wall_5_l_b_x = -2.5
wall_5_l_b_y = -2.5
wall_5_r_t_x = -2.5
wall_5_r_t_y = -2.5

wall_6_l_b_x = -2.5
wall_6_l_b_y = -2.5
wall_6_r_t_x = 2.5
wall_6_r_t_y = -2.47


XY_START = np.array([-2.0, 2.0])
XY_GOAL = np.array([-2.5, 2.5])
direction vector 1.0, 0.0
'''

'''COMPLEX_OBS_T_basic_2

XY_START = np.array([-3.0, -1.5])
XY_GOAL = np.array([4.0, 1.0])
direction vector 0.0, 1.0

wall_1_l_b_x = -3.5
wall_1_l_b_y = 0.0
wall_1_r_t_x = -2.0
wall_1_r_t_y = 0.3

wall_2_l_b_x = -2.3
wall_2_l_b_y = -2.5
wall_2_r_t_x = -2.0
wall_2_r_t_y = 0.0

wall_3_l_b_x = 0.0
wall_3_l_b_y = -2.5
wall_3_r_t_x = 2.0
wall_3_r_t_y = -0.5

wall_4_l_b_x = 0.0
wall_4_l_b_y = 0.5
wall_4_r_t_x = 2.0
wall_4_r_t_y = 2.5

wall_5_l_b_x = -2.5
wall_5_l_b_y = -2.5
wall_5_r_t_x = -2.5
wall_5_r_t_y = -2.5

wall_6_l_b_x = -2.5
wall_6_l_b_y = -2.5
wall_6_r_t_x = 2.5
wall_6_r_t_y = -2.47
'''

''' COMPLEX_OBS_T_basic_1
XY_START = np.array([-4.0, 0.7])
XY_GOAL = np.array([4.0, -1.0])
direction vector 1.0, 0.0


wall_1_l_b_x = -3.0
wall_1_l_b_y = -0.5
wall_1_r_t_x = -2.5
wall_1_r_t_y = 2.5

wall_2_l_b_x = -0.5
wall_2_l_b_y = -2.5
wall_2_r_t_x = 0.0
wall_2_r_t_y = 1.0

wall_3_l_b_x = 2.5
wall_3_l_b_y = 0.0
wall_3_r_t_x = 3.0
wall_3_r_t_y = 2.5

wall_4_l_b_x = -2.5
wall_4_l_b_y = -2.5
wall_4_r_t_x = -2.5
wall_4_r_t_y = -2.5

wall_5_l_b_x = -2.5
wall_5_l_b_y = -2.5
wall_5_r_t_x = -2.5
wall_5_r_t_y = -2.5

wall_6_l_b_x = -2.5
wall_6_l_b_y = -2.5
wall_6_r_t_x = 2.5
wall_6_r_t_y = -2.47

'''


OBS_for_ADSWARM = [
    # wall
    np.array([[-1.0, 0], [2.5, 0.], [2.5, 0.3], [-1.0, 0.3]]),
    # np.array([[0.5, 1.6], [0.8, 1.6], [0.8, 2.5], [0.5, 2.5]]),
    np.array([[0.5, 2.0], [0.8, 2.0], [0.8, 2.5], [0.5, 2.5]]),
    np.array([[0.0, -2.5], [0.3, -2.5], [0.3, -1.6], [0.0, -1.6]]),
    np.array([[-1.3, 0.3], [-0.7, 0.3], [-0.7, 0.6], [-1.3, 0.6]]),
    np.array([[-1.0, 1.8], [-0.7, 1.8], [-0.7, 2.5], [-1.0, 2.5]]),
    np.array([[-2.0, -2.47], [-1.4, -2.47], [-1.4, -1.2], [-2.0, -1.2]]),

    # room
    np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
    np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
    np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
    np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]),

    # moving obstacle

    # for fuzzing
    np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),

    # obs2 obs[-3] l_t_r
    np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),
    
    # obs1 obs[-2] dia
    np.array([[-2.6, 1.4], [-2.5, 1.4], [-2.5, 1.5], [-2.6, 1.5]]),
]