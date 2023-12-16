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

def ret_init_log(seed, mod_val, x, y):
    return "randomseed [%d] modified_value [%s] x[%f] from [%d]" \
        % (seed, mod_val, x, y)


def print_coef(tick, purpose, param):
    msg = "[%d] %s: %f" % (tick, COEF[purpose], param)
    print(msg)


def print_hdr(tick, p_tick):
    msg = "[%d] modification starts at: %d" % (tick, p_tick)
    print(msg)


def _ret_log(record, idx):
    _msg = str(-idx)

    for x in range(11):
        _msg += " " + str(record[-idx][x][0]) + " " + str(record[-idx][x][1])
    return _msg


def ret_log(record, idx, param, args, simul_tick):
    msg = "%d randomseed [%d] modified_value [%s] x[%f] \
from [%d] simul_tick [%d] " \
          % (args[5], args[1], args[3], args[2], args[4], simul_tick)
    msg += _ret_log(record, idx)
    msg += " " + str(param.crash)
    msg += " " + str(param.info_crashed_drone)
    msg += " " + str(param.info_crashed_obs)
    msg += " " + str(param.info_crashed_time)
    msg += " " + str(param.info_trapped_drone)
    return msg


def ret_crash_log(param, args, simul_tick, randometest_order):
    msg = "%d randomseed [%d] modified_value [%s] x[%f] \
from [%d] simul_tick [%d] " \
          % (args[5], args[1], args[3], args[2], args[4], simul_tick)
    msg += " " + str(param.crash)
    msg += " " + str(param.info_crashed_drone)
    msg += " " + str(param.info_crashed_obs)
    msg += " " + str(param.info_crashed_time)
    msg += " " + str(param.info_trapped_drone)
    msg += " " + str(param.info_crashed_dist)
    msg += " " + str(randometest_order)
    return msg    