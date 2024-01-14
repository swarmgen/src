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


def pp():
    # print(np.zeros((5, 10)))
    print(f"Test import.")


my_list = ["d", "c", "b", "a"]

my_list.pop(1)

print(my_list)
