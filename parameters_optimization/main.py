#!/usr/bin/env python3

import numpy as np
from scipy.optimize import minimize
from functools import partial

from data_preprocessing import Bag
from aux_functions import cost_function
from plot_functions import *


# find the parameters r, l+w and T*N that minimize the cost function
bag = Bag(3)

initial_param_vector = np.array([0.07, 0.369, 210])

res = minimize(partial(cost_function, bag),
                initial_param_vector,
                method='Nelder-Mead',
                options={'xatol': 1e-5, 'disp': True},
                )

print('Optimal robot parameters')
print('         r = ' + str(res.x[0]))
print('         l+w = ' + str(res.x[1]))
print('         T*N = ' + str(res.x[2]))


# plots the x,y paths both from the odometry and from the ground pose, with adjustable sliders for r and l+w parameters
# plot_bag_diff(bag, r_min, r_init, r_max, l_w_min, l_w_init, l_w_max):
#
# example:
""" bag = Bag(3)

v_min = np.array([0.05, 0.3, 180])
v_init = np.array([0.07, 0.369, 210])
v_max = np.array([0.09, 0.4, 240])

plot_diff(bag, v_min, v_init, v_max) """


# plots the cost function as a function of parameter r (with slider for l+w)
""" bag = Bag(3)

v_min = np.array([0.05, 0.3, 180])
v_init = np.array([0.07, 0.369, 210])
v_max = np.array([0.09, 0.4, 240])

plot2d_bag_cost_function(bag, v_min, v_init, v_max) """