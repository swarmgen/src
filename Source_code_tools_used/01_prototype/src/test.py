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


# temp_pop_up_from_spawning_pool = np.loadtxt(open(OUTPUT_DIR + "/result.log", "rb"),
#                                             delimiter=" ", skiprows=1)

# pop_up_from_spawning_pool = temp_pop_up_from_spawning_pool[-1]

# print(pop_up_from_spawning_pool)

# file1 = open(OUTPUT_DIR + "/result.log", "r+")
# last_mat = file1.readlines()
# print(last_mat[-1])

# if(last_mat[-1] == 'XXX\n'):
#     print("OK")
# else:
#     print("not OK")


print(np.zeros((5, 10)))