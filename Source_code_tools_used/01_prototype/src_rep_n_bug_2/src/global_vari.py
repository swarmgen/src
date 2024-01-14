

TRAJ_ROBOT_1 = []
TRAJ_ROBOT_2 = []
TRAJ_ROBOT_3 = []
TRAJ_ROBOT_4 = []


MUTATION_OPERATORS = ["Inserting", "Deleting", "Moving",
                      "Changing_increasing", "Changing_decreasing", "Changing_rotating"]

"""
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
getter
setter
adder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
"""
# mutation operator


def get_mutation_operators():
    return MUTATION_OPERATORS


# init


def init_traj_robot_1():
    TRAJ_ROBOT_1 = []


def init_traj_robot_2():
    TRAJ_ROBOT_2 = []


def init_traj_robot_3():
    TRAJ_ROBOT_3 = []


def init_traj_robot_4():
    TRAJ_ROBOT_4 = []

# 1


def set_traj_robot_1(coordinate_traj):
    TRAJ_ROBOT_1 = coordinate_traj


def append_traj_robot_1(coordinate):
    TRAJ_ROBOT_1.append(coordinate)


def get_traj_robot_1():
    return TRAJ_ROBOT_1

# 2


def set_traj_robot_2(coordinate_traj):
    TRAJ_ROBOT_2 = coordinate_traj


def append_traj_robot_2(coordinate):
    TRAJ_ROBOT_2.append(coordinate)


def get_traj_robot_2():
    return TRAJ_ROBOT_2

# 3


def set_traj_robot_3(coordinate_traj):
    TRAJ_ROBOT_3 = coordinate_traj


def append_traj_robot_3(coordinate):
    TRAJ_ROBOT_3.append(coordinate)


def get_traj_robot_3():
    return TRAJ_ROBOT_3

# 4


def set_traj_robot_4(coordinate_traj):
    TRAJ_ROBOT_4 = coordinate_traj


def append_traj_robot_4(coordinate):
    TRAJ_ROBOT_4.append(coordinate)


def get_traj_robot_4():
    return TRAJ_ROBOT_4
