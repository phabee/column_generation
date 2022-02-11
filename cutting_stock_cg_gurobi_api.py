import logging
import random
import numpy as np
import random
# todo: transfer to gurobi api
# https://www.youtube.com/watch?v=NFaFMGWj1Hc&t=1497s
# calc shadow costs:
# https://stackoverflow.com/questions/49126492/how-to-calculate-the-shadow-price-in-gurobi
import gurobipy as gp

from gurobipy import GRB

# logger configuration
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s', '%Y-%m-%d %H:%M:%S')
file_handler = logging.FileHandler('cutting_stock.log')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(formatter)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)
logger.addHandler(file_handler)


def get_initial_configs(num_configs, rod_size, chunk_sizes):
    configs = []
    for i in range(0, num_configs):
        cum_length = 0
        current_config = [0] * len(chunk_sizes)
        while rod_size - cum_length >= min(chunk_sizes):
            # choose random chunk size
            random_chunk_size_id = random.randint(0, len(chunk_sizes) - 1)
            random_chunk_size = chunk_sizes[random_chunk_size_id]

            # if still containable in rod-remainder, cut
            if rod_size - cum_length >= random_chunk_size:
                current_config[random_chunk_size_id] = current_config[random_chunk_size_id] + 1
                cum_length = cum_length + random_chunk_size
        configs.append(current_config)
    return configs


def build_initial_model(configs, model: gp.Model, rod_size, chunk_sizes, demand, num_rods):
    logger.info('Creating model...')
    all_configs = range(0, len(configs))

    # generate decision variables: X_i tells, how many times config i is used or not
    x = model.addVars(all_configs, name='x', vtype=GRB.CONTINUOUS, lb=0, ub=num_rods)

    # C0: we externally assure that only valid configs are provided, so we don't need to check for every config,
    # whether it complies with the rodlength. :P

    # C1: demand must be fulfilled over all selected configs
    for j in range(0, len(chunk_sizes)):
        model.addConstr(sum(x[i] * configs[i][j] for i in all_configs) >= demand[j])

    # C2: max available rods may not be exceeded
    model.addConstr(sum(x[i] for i in all_configs) <= num_rods)

    # minimize number of rods used
    model.setObjective(sum(x[i] for i in all_configs), GRB.MINIMIZE)
    return x


def convert_1D_var_i(size, VAR, is_int=True):
    """
    Converts the 1d Google OR-Tools solution variable VAR_ij into a numerically usable variable and stores it on the
    it on the OptimizerReuslt object in the member varMember for later access.

    :param size: the dimension of the datafield as int
    :param VAR: Google OR-Tools 1d-solution Variable to be converted to a normal variable
    :param is_int: if the datafield is of type int or float
    :return:
    """
    if is_int:
        var_member = np.zeros(shape=size, dtype=np.int32)
    else:
        var_member = np.zeros(shape=size, dtype=np.float32)
    for i in range(0, size):
        var_member[i] = VAR[i].solution_value()
    return var_member


def solve():
    random.seed(5)
    logger.info('creating model')
    m = gp.Model('cutting stock')

    # problem config
    # maximal number of rods available (upper bound)
    num_rods = 200
    rod_size = 200
    chunk_sizes = [20, 40, 80, 90]
    demand = [21, 17, 4, 8]
    chunk_sizes.sort()

    # define initial cutting configurations (connected to our X_i decision variables!)
    configs = get_initial_configs(num_configs=5, rod_size=rod_size, chunk_sizes=chunk_sizes)
    print("Initial Configs: " + str(configs))

    x = build_initial_model(configs=configs, model=m, rod_size=rod_size, chunk_sizes=chunk_sizes,
                            demand=demand, num_rods=num_rods)

    logger.info('Solving MIP model... ')
    m.optimize()

    for v in m.getVars():
        print('%s %g' % (v.varName, v.x))

    print("shadow costs:" + str(m.getAttr(GRB.Attr.Pi)))
    

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    solve()
