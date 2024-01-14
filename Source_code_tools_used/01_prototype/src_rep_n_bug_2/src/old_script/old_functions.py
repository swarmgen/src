def move_special_obs_in_sim(obs_element):
    '''
    obs_element is not whole obs
    Pick one obs in whole obs.
    e.g., OBSTACLE = move_special_obs_in_sim(obs[5]):
    '''

    temp_target_1 = copy.deepcopy(obs_element)
    temp_bit = 999  # big number enough to move away

    obs_element = [[0, temp_bit], [0.01, temp_bit],
                   [0.01, temp_bit+0.01], [0, temp_bit+0.01]]

    return obs_element


def npy_save(folderName, params,
             r1_sp_g, r2_sp_g, r3_sp_g, r4_sp_g,
             r1_sp, r2_sp, r3_sp, r4_sp, obs_sp):

    mkdirs(REPLAY_DIR + "/" + folderName)

    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_1_sp_global_"
        + str(params.rand_seed) + ".npy", r1_sp_g)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_2_sp_global_"
        + str(params.rand_seed) + ".npy", r2_sp_g)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_3_sp_global_"
        + str(params.rand_seed) + ".npy", r3_sp_g)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_4_sp_global_"
        + str(params.rand_seed) + ".npy", r4_sp_g)

    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_1_sp_"
        + str(params.rand_seed) + ".npy", r1_sp)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_2_sp_"
        + str(params.rand_seed) + ".npy", r2_sp)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_3_sp_"
        + str(params.rand_seed) + ".npy", r3_sp)
    np.save(
        REPLAY_DIR + "/" + folderName + "/robot_4_sp_"
        + str(params.rand_seed) + ".npy", r4_sp)

    np.save(
        REPLAY_DIR + "/" + folderName + "/obstacles_"
        + str(params.rand_seed) + ".npy", obs_sp)


def npy_load(folderNamePure, params):

    data_robot1_sp_global = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_1_sp_global_" + str(params.rand_seed) + ".npy")
    data_robot2_sp_global = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_2_sp_global_" + str(params.rand_seed) + ".npy")
    data_robot3_sp_global = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_3_sp_global_" + str(params.rand_seed) + ".npy")
    data_robot4_sp_global = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_4_sp_global_" + str(params.rand_seed) + ".npy")

    data_robot1_sp = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_1_sp_" + str(params.rand_seed) + ".npy")
    data_robot2_sp = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_2_sp_" + str(params.rand_seed) + ".npy")
    data_robot3_sp = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_3_sp_" + str(params.rand_seed) + ".npy")
    data_robot4_sp = np.load(
        REPLAY_DIR + "/" + str(folderNamePure) +
        "/" + str(params.rand_seed)
        + "/robot_4_sp_" + str(params.rand_seed) + ".npy")

    data_obstacles = np.load(
        REPLAY_DIR + "/fixed_obstacles/obstacles_for_1000tick.npy")  # fixed

    return data_robot1_sp_global, data_robot2_sp_global, \
        data_robot3_sp_global, data_robot4_sp_global, \
        data_robot1_sp, data_robot2_sp, \
        data_robot3_sp, data_robot4_sp, \
        data_obstacles


def put_into_seed_pool(robots, arg_target_index, params, seed_pool_idx):

    mkdirs(SEEDPOOL_DIR + "/" + str(seed_pool_idx))

    target_index = arg_target_index  # params.target_index

    # print("target_index = " + str(target_index))
    # print("total simulation time is " +
    #       str(len(robots[target_index].influence_avg)))

    temp_max_relative_s = 0
    """
    Note that the simulation tick is not the absolute tick,
    it is just the order of array.
    """
    if target_index == 0:
        writeFile(SEEDPOOL_DIR + "/" + str(seed_pool_idx) + "/ref_f" + str(target_index)+".csv",
                  "dcc_des dcc_l dcc_f2 dcc_f3 dcc_obs[-1] dcc_obs[-2] dcc_obs[-3] dcc_wall "
                  )

    else:
        writeFile(SEEDPOOL_DIR + "/" + str(seed_pool_idx) + "/ref_f" + str(target_index)+".csv",
                  "dcc_des dcc_l dcc_f2 dcc_f3 dcc_obs[-1] dcc_obs[-2] dcc_obs[-3] dcc_wall "
                  )

    logging.debug("len(robots[target_index].influence_max): [%i], robot[%i]" % (
        len(robots[target_index].influence_max), arg_target_index))
    for tick_index in range(len(robots[target_index].influence_max)):

        if tick_index == len(robots[target_index].influence_avg) - 1:
            break

        '''
        when you need to consider only obs_dia,
        use below code

        if robots[target_index].influence_max[tick_index][5] != 0.00 or robots[target_index].influence_max[tick_index + 1][5] != 0.00 :
        '''

        # everytime
        if tick_index + 1 >= len(robots[target_index].influence_max):
            continue

        local_condition_1 = robots[target_index].influence_max[tick_index][0] != 0.00
        if target_index == 0:
            local_condition_2 = robots[target_index].influence_max[tick_index + 1][1] != 0.00
        else:
            local_condition_2 = robots[target_index].influence_max[tick_index + 1][5] != 0.00

        if local_condition_1 or local_condition_2:
            # print("contribution is now writing...")

            if target_index == 0:
                writeFile(SEEDPOOL_DIR + "/" + str(seed_pool_idx) + "/ref_f" + str(target_index)+".csv",
                          " " + str(robots[target_index].dcc[tick_index][0]) +
                          " " + str(robots[target_index].dcc[tick_index][1]) +
                          " " + str(robots[target_index].dcc[tick_index][2]) +
                          " " + str(robots[target_index].dcc[tick_index][3]) +
                          " " + str(robots[target_index].dcc[tick_index][4])

                          )  # This is because leader only cares obstacles not followers.
            else:
                temp_max_relative_s = max(
                    temp_max_relative_s, robots[target_index].relative_score_with_obs_dia[tick_index])

                writeFile(SEEDPOOL_DIR + "/" + str(seed_pool_idx) + "/ref_f" + str(target_index)+".csv",
                          str(robots[target_index].dcc[tick_index][0]) +
                          " " + str(robots[target_index].dcc[tick_index][1]) +
                          " " + str(robots[target_index].dcc[tick_index][2]) +
                          " " + str(robots[target_index].dcc[tick_index][3]) +
                          " " + str(robots[target_index].dcc[tick_index][4]) +
                          " " + str(robots[target_index].dcc[tick_index][5]) +
                          " " + str(robots[target_index].dcc[tick_index][6]) +
                          " " + str(robots[target_index].dcc[tick_index][7])

                          )

# TODO: remove arrays for record, later
