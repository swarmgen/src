
"""
(XX) Attack drone code: below
1. move_obstacles
2. targetting
3. vector transformation
"""

"""
make_target_coor fuction make the target coor following the physical rule.
INPUT: 1) goal (1*2 np.array), 2) vel_atk_drone (float or integer), 3) current_atk_coor (1*2 np.array)
OUTPUT: target_coor

So, control the goal!
"""


def make_target_coor(goal, vel_atk_drone, current_atk_coor):
    """
    goal + current_atk_coor -> direction vector
    direction vector + vel_atk_drone -> target_vector
    target_vector + current_atk_coor -> target_coor
    for example, vel_atk_drone = *** 4.0 m/s ***
    """
    direction_vector = goal - current_atk_coor

    target_coor = current_atk_coor + 0.01 * \
        vel_atk_drone * (direction_vector / norm(direction_vector))

    return target_coor

def spawn_attack_drone(target_coor, zone_idx, params):
    spawning_coor = copy.deepcopy(target_coor)
    zone_coef = 0
    sign_x = np.random.randint(2) * 2 - 1
    sign_y = np.random.randint(2) * 2 - 1
    print("sign_x: " + str(sign_x) + " sign_y: "+str(sign_y))

    if zone_idx == 1:
        zone_coef = 0.2

    elif zone_idx == 2:
        zone_coef = 0.3

    elif zone_idx == 3:
        zone_coef = 0.4

    rnd_x = sign_x * (float(np.random.rand(1)) * 0.1 + zone_coef)  # [0.1)
    rnd_y = sign_y * (float(np.random.rand(1)) * 0.1 + zone_coef)

    spawning_coor[0] += rnd_x
    spawning_coor[1] += rnd_y

    return spawning_coor