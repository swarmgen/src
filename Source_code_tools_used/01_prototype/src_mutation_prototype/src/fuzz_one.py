#!/usr/bin/env python

"""
Autonumous navigation of robots formation with Layered path-planner:
- global planner: RRT
- local planner: Artificial Potential Fields
"""

import os
import sys
import time
import glob
import signal
import pickle
import shutil
import argparse
import itertools
import numpy as np
import matplotlib.pyplot as plt
import logging

from tqdm import tqdm
from numpy.linalg import norm

from conf import *
from common import *
from tools import *
from rrt import *
from potential_fields import *
from new_tools import *
from attack_drone import *
from input_output import *

import global_vari

np.set_printoptions(threshold=np.inf, linewidth=np.inf)

arg_01 = 0
metrics = Metrics()

"""
USAGE:
    $ python3 src/planner.py tgswarm -k vanilla -rs 1 -re 2
    rs is starting number of random seed
    re is last number of random seed
"""


def exit_gracefully(original_sigint):
    def _exit_gracefully(signum, frame):
        signal.signal(signal.SIGINT, original_sigint)
        try:
            if sys.version_info[0] == 2:
                if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):
                    sys.exit(1)
            else:
                if input("\nReally quit? (y/n)> ").lower().startswith('y'):
                    sys.exit(1)
        except KeyboardInterrupt:
            print("Ok ok, quitting")
            sys.exit(1)
        signal.signal(signal.SIGINT, _exit_gracefully)
    return _exit_gracefully


"""
Here, in move_obstacles, it just moves atk drone simply to the target coordination
Hence, it requires a function that can offer the realistic (physically) target following certain logic.

"""


def modify_obstacle(obs, x):
    modified_obs = [(
        obs[-x][0][0] + obs[-x][1][0] + obs[-x][2][0] + obs[-x][3][0])
        * 0.25,
        (obs[-x][0][1] + obs[-x][1][1] + obs[-x][2][1] + obs[-x][3][1])
        * 0.25]
    return modified_obs


def fuzz_one(args, obstacle_type, vis,  check_crash=False, manual_mode=False, param_setting='None', randometest_order=1):
    # Each run initialize parameters using below code.
    start_time = time.time()

    params = Params()
    if vis == "Y":
        params.animate_rrt = True
        params.visualize = True

    # TODO: this is not good, but use temporaly
    p_target, p_seed, p_coef, p_purpose, p_tick, p_idx = args
    print(f"==============================================================")
    print(f"Exp Summary")
    print(
        f"Args [p_target, p_seed, p_coef, p_purpose, p_tick, p_idx]: "+str(args))
    print(f"Starting point: {xy_start[-1]}")

    params.rand_seed = p_seed
    params.target_index = p_target

    print(f"Seed number: {params.rand_seed}")
    if ATTACK_TARGET_MODE == 'random':
        random_attack_target_gen(params)

    if param_setting == 'record':
        print("Fuzz starts")
        # it can control each run to make it either 'pure' or 'partial replay'.
        # it depends on whether 'params.mode_replay = True and p_idx = 0' or not
        params.contribution = True
        params.mode_replay = False                      # False, either is ok
        print("Mode: Pure")

        params.mode_stop_before_obs = False            # it should go to the goal
        params.crash_check_for_random_testing = False
        params.mode_randomtesting = False
        # if it is False, loading folder is set as 'fixed_pure'.
        params.mode_record_trajectory_replay = False
        params.already_written_allsp = False
        params.mode_defective = False
        # make sure sim is not ended after recording coordinates on all_sp.log

    else:
        print("ERROR: Param setting is not defined by user.")
        sys.exit(1)

    robots = []

    for i in range(params.num_robots):
        robots.append(Robot(i+1))

    robot1 = robots[0]
    robot1.leader = True
    rbt = robot1

    global OBSTACLES
    OBSTACLES = copy.deepcopy(obstacle_type)

    obst_1 = []
    obst_2 = []
    obst_3 = []
    coord_record = []

    """
    rbt: robots
    args: p_target, p_seed, p_coef, p_purpose, p_tick, p_idx
    check_crash: check crash for each tick
    manual_mode: manually modify params coef?
    """

    fliped_coin = []

    msg = ret_init_log(p_seed, p_purpose, p_coef, p_tick)

    if params.contribution is True:
        writeFile(CONTR_PN, "\n" + msg)

    if params.all_sp_record_for_replay is True:
        writeFile(ALLSP_PN, msg + "\n" + HEADER)

    if params.visualize:
        draw_map(OBSTACLES)  # for background environment
        plt.plot(
            xy_start[-1][0], xy_start[-1][1], 'bo', color='red',
            markersize=20, label='start')  # for background environment
        plt.plot(
            xy_goal[0], xy_goal[1], 'bo', color='green',
            markersize=20, label='goal')  # for background environment

    simul_tick = 1

    # SOLVER. focus on input and output!
    #
    #
    #
    #
    P_long = rrt_path(OBSTACLES, xy_start[-1], xy_goal, params)
    # print(f"[DBG] [Before] P_long: {P_long}")
    # print(f"[DBG] [Before] P_long[6,:]: {P_long[6,:]}")
    # print(f"[DBG] [Before] P_long[7,:]: {P_long[7,:]}")

    # P_long[6, 0] = -1.27
    # P_long[7, 0] = -1.27
    # P_long[8, 0] = -1.27
    # P_long[9, 0] = -1.27
    # P_long[10, 0] = -1.27
    # P_long[10, 1] = 1.65
    # P_long[11, 1] = 1.7
    # P_long[12, 1] = 1.7

    # print(f"[DBG] [After] P_long: {P_long}")
    # print(f"[DBG] [After] P_long[6,:]: {P_long[6,:]}")
    # print(f"[DBG] [After] P_long[7,:]: {P_long[7,:]}")

    #
    #
    #
    #

    # print('Path Shortenning...xy_start: ['+xy_start[-1]+"] xy_goal: ["+xy_goal+"]")

    # P = [[xN, yN], ..., [x1, y1], [x0, y0]]
    P = ShortenPath(P_long, OBSTACLES, params, smoothiters=50)
    traj_global = waypts2setpts(P, params)
    P = np.vstack([P, xy_start[-1]])

    if params.visualize or "DEBUG" in os.environ:
        plt.plot(
            P[:, 0], P[:, 1], linewidth=3, color='orange',
            label='Global planner path')
        plt.pause(0.5)

    sp_ind = 0

    rbt.route = np.array([traj_global[0, :]])

    followers_sp = [[0, 0], [0, 0], [0, 0]]

    rbt.sp = copy.deepcopy(drone_01[-1])
    followers_sp[0] = copy.deepcopy(drone_02[-1])
    followers_sp[1] = copy.deepcopy(drone_03[-1])
    followers_sp[2] = copy.deepcopy(drone_04[-1])

    # print("[dbg]followers_sp: "+str(followers_sp))

    for i in range(len(followers_sp)):  # for all followers
        robots[i + 1].sp = followers_sp[i]
        robots[i + 1].route = np.array([followers_sp[i]])

    for p in range(len(followers_sp)):
        followers_sp[p] = robots[p + 1].sp

    print('Main loop starts.')

    writeIndex_dist_obs(p_purpose, p_coef, p_tick, params)

    write_index_contribution_others()

    '''main simulation loop'''
    # loop through all the setpoint from global planner

    res_suc_fail = "Succeeded"
    writeFile(RES, res_suc_fail)

    for x in tqdm(range(MAX_TICK)):

        if "DEBUG" in os.environ:
            start_time = time.time()

        dist_to_goal = norm(rbt.sp - xy_goal)
        # print("[dbg]dist_to_goal: "+str(dist_to_goal))
        if dist_to_goal < params.goal_tolerance:
            print('Goal is reached')

            # update the coordinates for next run
            fall_back = 3.5  # 3.5 when 1.0

            elapsed_time = time.time() - start_time
            msg = "Goal reached: randomseed [%d] modified_value [%s] x[%f] from [%d] \
simul_tick [%d] with [%f] in zone_[%d] speedX [%f] obsSize [%f]" % \
                  (p_seed, p_purpose, p_coef, p_tick,
                   simul_tick, elapsed_time, zone_idx, temp_coef_atk_vel, params.obst_size_bit)

            writeFile(RES, msg)
            ##########################
            # Plot DCC
            ##########################
            try:
                for DCC_robot_index in range(1, 4):

                    DCC_x = np.arange(
                        len(robots[DCC_robot_index].influence_max))
                    DCC_Y = robots[DCC_robot_index].dcc

                    # y_temp = np.random.randint(5, 50, (10, 3))
                    # print(
                    #     f"y_temp.shape = {y_temp.shape()}, DCC_Y_1 = {DCC_Y_1.shape()}")

                    DCC_DF = pd.DataFrame(DCC_Y, index=DCC_x)

                    DCC_DF_divide = DCC_DF.divide(DCC_DF.sum(axis=1), axis=0)
                    DCC_fig_size = int(
                        0.1*len(robots[DCC_robot_index].influence_max))
                    DCC_AX_divide = DCC_DF_divide.plot(
                        kind='area', stacked=True, title=f'DCC_{DCC_robot_index}', figsize=(DCC_fig_size, 5))

                    DCC_LEGEND = DCC_AX_divide.legend(loc='center left', bbox_to_anchor=(1., 0.5),
                                                      labels=["Des", "L", "F2", "F3", "Obs1", "Obs2", "Obs3", "Wall"])

                    # DCC_AX_divide.figure(figsize=(10, 6))

                    DCC_AX_divide.set_ylabel('DCC')
                    DCC_AX_divide.set_xlabel('Time (tick)')
                    DCC_AX_divide.margins(0, 0)

                    # plt.rcParams["figure.figsize"] = (20, 3)
                    plt.xticks(np.arange(min(DCC_x), max(DCC_x), 10))
                    plt.savefig(f"output/DCC_area_plot_test_{params.rand_seed}_drone_{DCC_robot_index}.png",
                                bbox_extra_artists=(DCC_LEGEND,), bbox_inches='tight')

                if SCREENSHOT:
                    plt.cla()
                    visualize2D(simul_tick, OBSTACLES, params,
                                robots, robot1, centroid, traj_global)
                    # plt.draw()
                    # plt.pause(0.01)
                    ts = time.gmtime()
                    time_s = time.strftime("%Y-%m-%d %H:%M:%S", ts)
                    msg_coord = "p_seed %d simul_tick %d leader %.2f %.2f f1 %.2f %.2f f2 %.2f %.2f f3 %.2f %.2f time %s " % \
                        (p_seed, simul_tick, rbt.sp[0], rbt.sp[1], robots[1].sp[0], robots[1].sp[1],
                         robots[2].sp[0], robots[2].sp[1], robots[3].sp[0], robots[3].sp[1], time_s)
                    plt.savefig(f"output/{msg_coord}.png")
            except:
                print("[Report] No plot")
            break

        centroid = copy.deepcopy(robots[1].sp)

        zone_idx = 0
        temp_coef_atk_vel = 2.0

        fuzz_input_special_target_2 = [0, 0]

        # global set point is set here
        rbt.sp_global = traj_global[sp_ind, :]

        # Note this function always on.

        if param_setting != 'randomtest':
            checker_dist_obs(robots, simul_tick, params, OBSTACLES)

        # Note this function always on.
        # TODO: target_obs should be from arg
        # with 3rd OBS for now
        target_obs = '3rd'

        if params.contribution is True:

            contribution_leader(robots, simul_tick,
                                params, OBSTACLES, target_obs)

        # correct leader's pose with local planner
        rbt.new_local_planner(OBSTACLES, params)

        # Below part should be placed after rbt is updated: rbt.new_local_planner(OBSTACLES, params)
        if params.contribution is True:

            if DCC_CAL:
                contribution_others(robots, rbt, 'for_followers', arg_target_index=1, simulation_tick=simul_tick,
                                    followers_sp=followers_sp, params=params, OBSTACLES=OBSTACLES, target_obs=target_obs)
                contribution_others(robots, rbt, 'for_followers', arg_target_index=2, simulation_tick=simul_tick,
                                    followers_sp=followers_sp, params=params, OBSTACLES=OBSTACLES, target_obs=target_obs)
                contribution_others(robots, rbt, 'for_followers', arg_target_index=3, simulation_tick=simul_tick,
                                    followers_sp=followers_sp, params=params, OBSTACLES=OBSTACLES, target_obs=target_obs)

        if "DEBUG" in os.environ:
            print(" - check point1: %f" % (time.time() - start_time))

        followers_sp_global = formation(
            params.num_robots, rbt.sp_global,
            v=normalize(rbt.sp_global - rbt.sp),
            l=params.interrobots_dist)

        for i in range(len(followers_sp_global)):
            robots[i + 1].sp_global = followers_sp_global[i]

        # formation poses correction with
        for p in range(len(followers_sp)):
            # robots repel from each other inside the formation
            # all poses except the robot[p]
            robots_obstacles_sp = [x for i, x in enumerate(
                followers_sp + [rbt.sp]) if i != p]
            # each drone is defined as a small cube for
            # inter-robots collision avoidance
            robots_obstacles = poses2polygons(robots_obstacles_sp)

            # combine exisiting obstacles on the map
            # with other robots[for each i: i!=p] in formation
            obstacles1 = np.array(OBSTACLES + robots_obstacles)
            # follower robot's position correction with local planner

            robots[p + 1].new_local_planner(obstacles1, params)
            followers_sp[p] = robots[p + 1].sp

        if RECORD_COORD:
            msg_coord = "p_seed %d simul_tick %d leader %f %f f1 %f %f f2 %f %f f3 %f %f" % \
                (p_seed, simul_tick, rbt.sp[0], rbt.sp[1], robots[1].sp[0], robots[1].sp[1],
                 robots[2].sp[0], robots[2].sp[1], robots[3].sp[0], robots[3].sp[1])

            writeFile(COORD_PN, msg_coord)

        # visualization  # not needed
        if SCREENSHOT and simul_tick == 300:
            plt.cla()
            visualize2D(simul_tick, OBSTACLES, params,
                        robots, robot1, centroid, traj_global)
            # plt.draw()
            # plt.pause(0.01)
            ts = time.gmtime()
            time_s = time.strftime("%Y-%m-%d %H:%M:%S", ts)
            msg_coord = "p_seed %d simul_tick %d leader %f %f f1 %f %f f2 %f %f f3 %f %f time %s " % \
                (p_seed, simul_tick, rbt.sp[0], rbt.sp[1], robots[1].sp[0], robots[1].sp[1],
                 robots[2].sp[0], robots[2].sp[1], robots[3].sp[0], robots[3].sp[1], time_s)
            plt.savefig(f"output/{msg_coord}.png")

        det_straggler = robots[1].sp[0] <= - \
            2.0 or robots[2].sp[0] <= -2.0 or robots[3].sp[0] <= -2.0
        det_others = robots[0].sp[1] >= -0.5 and robots[1].sp[1] >= - \
            0.5 and robots[2].sp[1] >= -0.5 and robots[3].sp[1] >= -0.5
        if simul_tick >= 300 and det_straggler and det_others:
            break

        """
        added effects
        1. wind
        """
        # print("[dbg] before sp: "+str(rbt.sp) + "|" +str(robots[1].sp) + "|" +str(robots[2].sp) + "|" +str(robots[3].sp))

        if EFFECT_WIND:
            rbt.sp = effectWind(rbt.sp)
            robots[1].sp = effectWind(robots[1].sp)
            robots[2].sp = effectWind(robots[2].sp)
            robots[3].sp = effectWind(robots[3].sp)

        # print("[dbg] after sp: "+str(rbt.sp) + "|" +str(robots[1].sp) + "|" +str(robots[2].sp) + "|" +str(robots[3].sp))

        if "DEBUG" in os.environ:
            print(" - check point2: %f" % (time.time() - start_time))

        # centroid pose:
        centroid = 0
        for robot in robots:
            centroid += robot.sp / len(robots)

        metrics.centroid_path = np.vstack([metrics.centroid_path, centroid])

        # dists to robots from the centroid:
        dists = []
        for robot in robots:
            dists.append(norm(centroid - robot.sp))

        # Formation size estimation
        metrics.mean_dists_array.append(np.mean(dists))

        # Formation max Radius #not needed
        metrics.max_dists_array.append(np.max(dists))

        # visualization  # not needed
        if params.visualize:
            plt.cla()
            visualize2D(simul_tick, OBSTACLES, params,
                        robots, robot1, centroid, traj_global)
            plt.draw()
            plt.pause(0.01)

        if check_crash is True:
            has_crash = checker_crash_simple(simul_tick, robot1)

        # for now this is not used. but let it alive.
        # checker_skip(robot1, simul_tick, centroid, params) ###########

        """
        for all_sp.log for finding out the upper bound in test inputs
        """
        l_sp = copy.deepcopy(rbt.sp)
        f1_sp = copy.deepcopy(robots[1].sp)
        f2_sp = copy.deepcopy(robots[2].sp)
        f3_sp = copy.deepcopy(robots[3].sp)

        """
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        NOTE: Trajectory of swarm (all drones)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        """
        global_vari.append_traj_robot_1(l_sp)
        global_vari.append_traj_robot_2(f1_sp)
        global_vari.append_traj_robot_3(f2_sp)
        global_vari.append_traj_robot_4(f3_sp)

        """
        TODO: below are necessary?
        """
        obst_1 = modify_obstacle(OBSTACLES, 3)
        obst_2 = modify_obstacle(OBSTACLES, 2)
        obst_3 = modify_obstacle(OBSTACLES, 1)

        l_g_sp = copy.deepcopy(rbt.sp_global)
        f1_g_sp = copy.deepcopy(robots[1].sp_global)
        f2_g_sp = copy.deepcopy(robots[2].sp_global)
        f3_g_sp = copy.deepcopy(robots[3].sp_global)

        coord_record.append(
            [l_sp, f1_sp, f2_sp, f3_sp, obst_1, obst_2, obst_3,
                l_g_sp, f1_g_sp, f2_g_sp, f3_g_sp])

        if params.all_sp_record_for_replay is True:

            if params.already_written_allsp is False:

                for drone_index in range(0, 4):
                    if coord_record[-1][drone_index][0] >= \
                            params.info_trap_boundary_x \
                            and coord_record[-1][drone_index][1] >= \
                            params.info_trap_boundary_y:

                        if drone_index == 0:
                            params.info_trapped_drone = "L"
                        elif drone_index == 1:
                            params.info_trapped_drone = "f1"
                        elif drone_index == 2:
                            params.info_trapped_drone = "f2"
                        elif drone_index == 3:
                            params.info_trapped_drone = "f3"

                if params.mode_randomtesting is True:
                    '''
                    when random testing, it starts right before the obstacle,
                    it has no history.
                    '''
                    print(f"[dbg] what the fuck?")
                    for writing_index in range(1, 2):
                        writeFile(ALLSP_PN, ret_log(
                            coord_record, writing_index, params, args, simul_tick))

                    # TODO: in random testing, no need to proceed further after crash checked.
                    # print("[log] BREAK because crash is recorded and no need to go further.")
                    # break

                else:
                    print(f"[dbg] who change randomtesting mode??")

                    for writing_index in range(1, 5):
                        print(f"[dbg] writing_index = {writing_index}")
                        writeFile(ALLSP_PN, ret_log(
                            coord_record, writing_index, params, args, simul_tick)
                        )

                    params.already_written_allsp = True

            if params.mode_stop_before_obs is True:
                print(
                    "[log] break because params.mode_record_trajectory_replay is False")
                # This is deprecated. In real, not used so far.
                break

        if param_setting == 'record':
            if params.crash is True:

                writeFile(RES, ret_crash_log(
                    params, args, simul_tick, randometest_order)
                )
                res_suc_fail = "Failed (crashed)"
                writeFile(RES, res_suc_fail)
                params.crash = False

        if params.crash_check_for_random_testing is True:

            # we give 4 tick for stabilization

            if simul_tick >= p_tick + 4:
                checker_dist_obs(robots, simul_tick, params, OBSTACLES)

            if params.crash == True:

                writeFile(RES, ret_crash_log(
                    params, args, simul_tick, randometest_order)
                )
                params.crash_check_for_random_testing = False
                if params.mode_record_trajectory_replay is False:
                    break

        if param_setting == 'randomtest' and simul_tick > 100:
            min_x_robots = min(
                rbt.sp[0], robots[1].sp[0], robots[2].sp[0], robots[3].sp[0])
            if OBSTACLES[-2][0][0] <= min_x_robots - 0.1:
                print("[log] all difficulties has gone... let's go to the next round")
                break

        # loop should go on.
        simul_tick += 1

        """
        This is natural stop condition for the loop = simulation
        I added go_or_nogo and several restrictions in the if predicate.
        """
        if param_setting != 'randomtest':
            fliped_coin.append(sp_ind)

        new_condition = 1.0 * 1.5
        condition_1 = norm(rbt.sp_global - centroid) < 4 * params.max_sp_dist
        condition_2 = norm(rbt.sp - rbt.sp_global) < new_condition

        if condition_1:
            go_or_nogo = True
        else:

            '''
            TODO: log this and show later when it happened.
            '''
            # print("[log] loop condition is too strict!")

            go_or_nogo = False

        if sp_ind < traj_global.shape[0] - 1 and\
                go_or_nogo:
            sp_ind += 1

        """
        This is end condition by setting time limit is 500 ticks.
        """

        if simul_tick >= MAX_TICK:
            elapsed_time = time.time() - start_time
            msg = "Mission failed: randomseed [%d] modified_value [%s] x[%f] from [%d] \
simul_tick [%d] with [%f] in zone_[%d] speedX [%f] obsSize [%f]" % \
                  (p_seed, p_purpose, p_coef, p_tick,
                   simul_tick, elapsed_time, zone_idx, temp_coef_atk_vel, params.obst_size_bit)

            writeFile(RES, msg)

            res_suc_fail = "Failed (Cannot reach goal)"
            writeFile(RES, res_suc_fail)

            break

    '''
    for contribution score
    '''

    if params.record_contribution_score is True:
        print('Contribution score is being writing')
        write_influence(robots, arg_target_index=1, params=params)
        write_influence(robots, arg_target_index=2, params=params)
        write_influence(robots, arg_target_index=3, params=params)

        print("=== Summary ===")
        # print("1. Current seed pool size: "+str(end_seed_pool_idx)

        file1 = open(OUTPUT_DIR + "/result.log", "r+")

        msg = "distance_with_obs_dia: 0[%f], 4[%f]" % (
            robots[0].distance_with_obs_dia[-1], robots[3].distance_with_obs_dia[-1])
        logging.debug(msg)

    if params.visualize or "DEBUG" in os.environ:
        plt.close('all')

    # TODO: to log files
    for obst_i in range(0, len(COMPLEX_OBS_T_1)):
        print(f"Obstacle: {COMPLEX_OBS_T_1[obst_i][0]}")
