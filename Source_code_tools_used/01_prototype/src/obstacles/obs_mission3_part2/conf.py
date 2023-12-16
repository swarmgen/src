#!/usr/bin/env python

import os
import copy
import logging
import numpy as np
from obstacle import *

'''
logging mode
'''
logging.basicConfig(level=logging.DEBUG)

'''
I/O configuration
- file path
'''

OUTPUT_DIR = "output"
REPLAY_DIR = "replay" #TODO check and remove
SEEDPOOL_DIR = "seedpool"

CONTR_PN = os.path.join(OUTPUT_DIR, "contribution.log")
ALLSP_PN = os.path.join(OUTPUT_DIR, "all_sp.log")
CRASH_PN = os.path.join(OUTPUT_DIR, "crash.log")
COORD_PN = os.path.join(OUTPUT_DIR, "coordinates.log")
CRASH_FOR_RT_PN = os.path.join(OUTPUT_DIR, "crash_for_rt.log")
DIST_PN = os.path.join(OUTPUT_DIR, "distance.log")
TESTCASE_PN = os.path.join("test", "testcases") #TODO check and remove
ATTACK_RES = os.path.join(
    OUTPUT_DIR, "EXP_103_str3_vel_1.5_1.0_obsSize_2.0_0.5.log") #TODO check and remove
RES = os.path.join(OUTPUT_DIR, "result.log") #TODO check and RENAME!
SIM_RESULT  = os.path.join(OUTPUT_DIR, "final_sim_result.log") #TODO check and RENAME!
ATTACK_SPOT = os.path.join(OUTPUT_DIR, "attack_spot.log") #TODO check and remove
TIMESTAMP = os.path.join(OUTPUT_DIR, "timestamp.log")
FINAL_REPORT = os.path.join(OUTPUT_DIR, "report.log")
'''
Crash threshold = safety distance
'''
CRASH_THRESHOLD = 0.3  # 0.1 -> 0.3


'''
General configuration for simulation
'''
REPORT = True #when true, sim ouputs the exp report.

VISUALIZE_RRT = True
VISUALIZE = True

DCC_CAL = True #if this is on, will be written as well.

RECORD_COORD = False #if this is on, it records all coordinates of swarm

EFFECT_WIND = True
WIND_DIRECTION = "east" #
WIND_POWER = 0.01 #0.04 is drone's v
# WIND_AREA_X = np.array([-2.0, 2.0]) #x_boundary #False # fixed, can be true by the condition
# WIND_AREA_Y = np.array([-2.5, 2.5])

WIND_AREA_X = np.array([-10.0, 10.0]) #x_boundary #False # fixed, can be true by the condition
WIND_AREA_Y = np.array([-10.0, 10.0])


SPECIAL_MOVE_OBS_IN_SIM = False #true: after rtt, move designated obs to somewhere
# TODO: why did I make this...?? for what?

MAX_TICK = 700  # original 500
NUM_EXPERIMENTS = 10

NUM_ATTACKER = 1


'''
Environment configuration
'''
MAP_SIZE = 1000  # original is 500
MAP_BOUNDS_METER = 5.0  # original is 2.5

MAP_SIZE_new_x = 1000#2000#400#2000
MAP_SIZE_new_y = 1000#2000#400
MAP_BOUNDS_METER_new_x = 5.0#2.0#10.0
MAP_BOUNDS_METER_new_y = 5.0#10.0#2.0

# for backup TODO: remove this after a while

INHERITED = False #when it is true, coordinates are set manually in planner.py

XY_START = np.array([-1.5, 0.0]) #for p1
XY_GOAL = np.array([2.3, 4.0]) #for p1

# XY_START = np.array([8.91361334-6.5-1.0, 1.0153345-0.5+8.0 ]) #for p2
# XY_GOAL = np.array([-1.0, -4.0]) #for p2

# XY_START = np.array([-1.0, 3.0]) # for vertical map for p2
# XY_GOAL = np.array([1.0, -8.0]) # for vertical map for p2



'''
Attack configuration
1. target
2. strategy
3. Spawn position for each attacker
    - zone_idx in old version: (deprecated later)
    - Current version has one attacker.
4. Spawn timing for each attacker
    - Current version has one attacker.
'''


ATTACK_TARGET_MODE = 'random'

# ATTACK_TARGET = 0


ATTACK_STRATEGY = 'd' # 'a', 'b', 'c', 'd', 'e'
ATTACK_DISTANCE = 0.4

'''
Feedback configuration
- Perturbation configuration
'''
COEF_LENGTH_BIG = 0.8
REGEN_BOUNDARY = 5.0

# mode that calculates the diff between dccs
DISTANCE_MODE = 'ncc' #'normal' 

if DISTANCE_MODE == 'ncc':
    # 4.0 for 'normal'
    # 0.87 for 'ncc'
    FUZZ_THRESHOLD = 0.8
    
elif DISTANCE_MODE == 'normal':
    FUZZ_THRESHOLD = 4.0






'''
TODO
???
'''
# when contribution score check? how far from obs?
CSCORE_WRITE_THRESHOLD = 0.8



'''
Deprecated
TODO: check and remove
'''
TESTFUNC = "test_crash"

# constant param: coef name
COEF = {}
COEF["r"] = "params.repulsive_coef"
COEF["a"] = "params.attractive_coef"
COEF["i"] = "params.influence_radius"
COEF["d"] = "params.interrobots_dist"
COEF["v"] = "params.drone_vel"
COEF["b"] = "params.w_bound"
