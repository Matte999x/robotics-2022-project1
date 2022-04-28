from multiprocessing import Pool
from functools import partial

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from aux_functions import cost_function, compute_odometry_positions


# plots the x,y paths both from the odometry and from the ground pose, with adjustable sliders for r and l+w parameters
def plot_diff(bag, min_param_vector, init_param_vector, max_param_vector):
    # compute and plot target path
    target_positions = bag.target_positions
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.25)
    plt.plot(target_positions[0,:], target_positions[2,:], 'k')

    # compute odometry path 
    wheel_speeds_matrix = bag.wheel_speeds_matrix
    time_steps_matrix = bag.time_steps_matrix
    odometry_positions = compute_odometry_positions(wheel_speeds_matrix, time_steps_matrix, target_positions, init_param_vector)

    # create the odometry plot
    odom_plot = plt.plot(odometry_positions[0,:], odometry_positions[2,:], 'r')[0]

    # add sliders for r, l+w and T*N
    ax_r = plt.axes([0.2, 0.15, 0.65, 0.03])
    r_slider = Slider(
        ax=ax_r,
        label='r [m]',
        valmin=min_param_vector[0],
        valmax=max_param_vector[0],
        valinit=init_param_vector[0],
    )
    ax_l_w = plt.axes([0.2, 0.1, 0.65, 0.03])
    l_w_slider = Slider(
        ax=ax_l_w,
        label='l+w [m]',
        valmin=min_param_vector[1],
        valmax=max_param_vector[1],
        valinit=init_param_vector[1],
    )
    ax_T_N = plt.axes([0.2, 0.05, 0.65, 0.03])
    T_N_slider = Slider(
        ax=ax_T_N,
        label='T * N',
        valmin=min_param_vector[2],
        valmax=max_param_vector[2],
        valinit=init_param_vector[2],
    )

    # define the update function for each slider
    def update(val):
        updated_param_vector = np.array([r_slider.val, l_w_slider.val, T_N_slider.val])
        odometry_positions = compute_odometry_positions(wheel_speeds_matrix, time_steps_matrix, target_positions, updated_param_vector)
        odom_plot.set_xdata(odometry_positions[0,:])
        odom_plot.set_ydata(odometry_positions[2,:])
        fig.canvas.draw_idle()

    # register the update function with each slider
    r_slider.on_changed(update)
    l_w_slider.on_changed(update)
    T_N_slider.on_changed(update)

    # show plot
    plt.show()


# plots the cost function as a function of parameter r (slider for l+w and T*N)
def plot2d_bag_cost_function(bag, min_param_vector, init_param_vector, max_param_vector):
    # compute costs
    r_space = np.linspace(min_param_vector[0], max_param_vector[0], 100)
    args = [np.array([r, init_param_vector[1], init_param_vector[2]]) for r in r_space]
    costs = [cost_function(bag, arg) for arg in args]

    # create plot
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.20)
    cost_plot = plt.plot(r_space, costs)[0]

    # add sliders for l+w and T*N
    ax_l_w = plt.axes([0.2, 0.12, 0.65, 0.03])
    l_w_slider = Slider(
        ax=ax_l_w,
        label='l+w [m]',
        valmin=min_param_vector[1],
        valmax=max_param_vector[1],
        valinit=init_param_vector[1],
    )
    ax_T_N = plt.axes([0.2, 0.06, 0.65, 0.03])
    T_N_slider = Slider(
        ax=ax_T_N,
        label='T * N',
        valmin=min_param_vector[2],
        valmax=max_param_vector[2],
        valinit=init_param_vector[2],
    )

    # define the update function for each slider
    def update(val):
        args = [np.array([r, l_w_slider.val, T_N_slider.val]) for r in r_space]

        p = Pool()
        costs = p.map(partial(cost_function, bag), args)

        cost_plot.set_ydata(costs)
        fig.canvas.draw_idle()

    # register the update function with each slider
    l_w_slider.on_changed(update)
    T_N_slider.on_changed(update)

    # show plot
    plt.show()