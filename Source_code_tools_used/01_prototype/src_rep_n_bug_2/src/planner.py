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
from fuzz_one import *
from obstacle import *
# from mutation_obstacle import *

import mutation.mutation_obstacle as mutation_obstacle
import obstacle
import metric.complexity_score as complexity_score

import test.test as test

import global_vari
from termcolor import colored
# print(colored('hello', 'red'), colored('world', 'green'))


def TGswarm(vis):

    # referfilename = 'randomtesting/pure_recog_time.csv'  # FIXED
    # referFile = np.array(pd.read_csv(referfilename, sep=','))

    index_seed = 0
    index_tick = 1

    # TODO: set this right
    starting_point = XY_START  # np.array([1.2, 1.25])  # [-1.2, 1.25] when 1.0

    des1 = copy.deepcopy(starting_point)
    des2, des3, des4 = formation(
        4, leader_des=des1, v=np.array([1.0, 0.0]),
        l=0.5)

    xy_start.append(starting_point)
    drone_01.append(des1)
    drone_02.append(des2)
    drone_03.append(des3)
    drone_04.append(des4)

    # for seed_idx in range(rand_start, rand_end):  # 3001

    # p_target_set = [1,2,3] # fixed in this mode 1 means f1, 2 means f2, 3 means f3
    p_target = 1  # TODO: remove
    p_seed = 1
    p_coef = None  # TODO: remove
    p_purpose = None  # TODO: remove
    p_idx = None
    p_tick = None

    '''robots_starting_point'''

    variation_p_coef = [1.0]  # [1.0, 1.6, 1.7, 1.8, 1.9, 2.0]#[1.5] #
    variation_p_idx = [0]
    variation_p_purpose = ['i']

    ''' Case 01: empty <- standard '''

    obstacle_type = COMPLEX_OBS_T_1
    # NEW_OBSTACLE_SET
    OBSTACLE_SET_PREVIOUS = COMPLEX_OBS_T_1

    COMPLEXITY_SCORE = 0
    COMPLEXITY_SCORE_PREVIOUS = 0

    mutation_operator_previous = "Inserting"  # Initial value

    for x in variation_p_coef:
        for y in variation_p_idx:
            for z in variation_p_purpose:
                while True:  # fuzzing loop including mutation

                    p_coef = x
                    p_idx = y
                    if p_idx == 0:
                        p_tick = 1

                    p_purpose = z

                    args = [p_target, p_seed, p_coef,
                            p_purpose, p_tick, p_idx]

                    # 1st trial
                    ts = time.gmtime()
                    timestamp_msg = time.strftime("%Y-%m-%d %H:%M:%S", ts)
                    writeFile(RES, f"Start at {timestamp_msg}")

                    """
                    0. Initialize global variables (for score)
                    - TRAJ_ROBOT_1, 2, 3, 4
                    """
                    global_vari.init_traj_robot_1()
                    global_vari.init_traj_robot_2()
                    global_vari.init_traj_robot_3()
                    global_vari.init_traj_robot_4()

                    '''
                    1. Run simulation:
                    Execution the mission
                    '''

                    fuzz_one(args, obstacle_type, vis,
                             param_setting='record')

                    # TODO: Utilities should be here
                    p_seed = p_seed + 1

                    '''
                    2. Complexity score
                    - a. function for "DCC", 
                    - b. function for "swarmsize", "accuracy", "coherence"
                    '''

                    COMPLEXITY_SCORE = complexity_score.measure_complexity_score(
                        p_seed, "swarmsize", global_vari.get_traj_robot_1(), global_vari.get_traj_robot_2(),
                        global_vari.get_traj_robot_3(), global_vari.get_traj_robot_4())

                    isImproved = COMPLEXITY_SCORE > COMPLEXITY_SCORE_PREVIOUS
                    if isImproved:
                        print(
                            f"[report] complexity is improved: from {round(COMPLEXITY_SCORE_PREVIOUS, 6)} to {round(COMPLEXITY_SCORE, 6)}")
                        COMPLEXITY_SCORE_PREVIOUS = COMPLEXITY_SCORE

                        """
                        NOTE: Update the obs_prev
                        """
                        OBSTACLE_SET_PREVIOUS = obstacle.get_obstacle_set()  # current obstacle set

                    '''
                    3. Mutation
                    if complexity score is improved, 
                        mutate (repeat what we mutate), 
                        update COMPLEXITY_SCORE_PREVIOUS
                    else 
                        revert obstacle (set obstacle to prev_obstacle)
                    
                    '''

                    '''
                    MUTATE
                    
                    # MUTATION_OPERATORS = ["Inserting", "Deleting", "Moving",
                    #                       "Changing_increasing", "Changing_decreasing", "Changing_rotating"]
                    '''
                    # TODO
                    # SEQUENCE_PREV
                    # SEQUENCE_NOW

                    mutation_operator_now = \
                        mutation_obstacle.what_is_next_mutation_operator(isImproved,
                                                                         mutation_operator_previous, global_vari.get_mutation_operators())

                    mutation_type = mutation_operator_now

                    if isImproved == True:

                        """
                        Select mutation operators here
                        1. In order of "Inserting" -> "Deletting" -> "Moving" -> ...
                        """

                        print(colored(
                            f"[report] Improved, Do the same thing: prev[{mutation_operator_previous}] = now[{mutation_type}].", "green"))
                        NEW_OBSTACLE_SET = mutation_obstacle.mutate_obstacle(COMPLEX_OBS_T_1, MAP_BOUNDS_METER_new_x,
                                                                             MAP_BOUNDS_METER_new_y, mutation_type)
                        obstacle.set_obstacle_set(NEW_OBSTACLE_SET)
                        mutation_operator_previous = mutation_operator_now

                    else:
                        """
                        Revert obstacle
                        """

                        print(colored(
                            f"[report] no improvement, change mutation type: prev[{mutation_operator_previous}] -> now[{mutation_type}].", "red"))

                        NEW_OBSTACLE_SET = mutation_obstacle.mutate_obstacle(OBSTACLE_SET_PREVIOUS, MAP_BOUNDS_METER_new_x,
                                                                             MAP_BOUNDS_METER_new_y, mutation_type)
                        obstacle.set_obstacle_set(NEW_OBSTACLE_SET)

                    '''
                    4. Update obstacle set
                    Put mutated obstacle into set
                    '''


arg_01 = 0
metrics = Metrics()

if __name__ == '__main__':
    signal.signal(
        signal.SIGINT, exit_gracefully(signal.getsignal(signal.SIGINT)))

    '''make directory for the output'''
    mkdirs(OUTPUT_DIR)
    mkdirs(SEEDPOOL_DIR)

    parser = argparse.ArgumentParser()
    subparser = parser.add_subparsers(title='sub-parsers')
    tester = subparser.add_parser(
        'tgswarm', help='Tester', add_help=False
    )
    # tester.add_argument(
    #     "-k", "--kind", dest="kind", type=str,
    #     default=None, help="Normal test or replay", required=True
    # )
    tester.add_argument(
        "-n", "--num_exp", dest="number_exp", type=str,
        default=None, help="Dry-run all cases", required=False
    )
    tester.add_argument(
        "-rs", "--rand_start", dest="rand_start", type=str,
        default=None, help="Starting random seed number", required=False
    )
    tester.add_argument(
        "-re", "--rand_end", dest="rand_end", type=str,
        default=None, help="Last random seed number", required=False
    )
    tester.add_argument(
        "-vis", "--visualize", dest="vis", type=str,
        default=None, help="Visualization of map", required=False
    )
    tester.set_defaults(action='tgswarm')
    args = parser.parse_args()

    test.pp()

    if args.action == "tgswarm":
        # kind = args.kind
        # rand_start = int(args.rand_start)
        # rand_end = int(args.rand_end)
        vis = args.vis

        TGswarm(vis)
