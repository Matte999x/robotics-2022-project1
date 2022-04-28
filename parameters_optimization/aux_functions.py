import numpy as np
import pandas as pd
import math

from data_preprocessing import Bag


# computes the path of the robot from the wheel data using Runge_Kutta 2nd order
def compute_odometry_positions(wheel_speeds_data_matrix_normalized, time_steps_data_matrix, target_positions_data_matrix, param_vector):
    r = param_vector[0]
    l_w = param_vector[1]
    T_N = param_vector[2]

    wheel_speeds_data_matrix = wheel_speeds_data_matrix_normalized / T_N

    transformation_matrix = (r/4) * np.array([[ 1,      1,       1,      1],
                                              [-1,      1,       1,     -1],
                                              [-1/l_w,  1/l_w,  -1/l_w,  1/l_w]
                                             ])
    velocitiy_matrix = transformation_matrix @ wheel_speeds_data_matrix
 
    odometry_positions = np.zeros((3, wheel_speeds_data_matrix.shape[1]))
    odometry_positions[:,0] = target_positions_data_matrix[:,0]
    for i in range(1, wheel_speeds_data_matrix.shape[1]):
        angle = odometry_positions[2,i-1] + velocitiy_matrix[2,i] * time_steps_data_matrix[i] / 2
        delta_x = (velocitiy_matrix[0,i] * math.cos(angle) - velocitiy_matrix[1,i] * math.sin(angle)) * time_steps_data_matrix[i]
        delta_y = (velocitiy_matrix[0,i] * math.sin(angle) + velocitiy_matrix[1,i] * math.cos(angle)) * time_steps_data_matrix[i]
        delta_theta = velocitiy_matrix[2,i] * time_steps_data_matrix[i]
        odometry_positions[:,i] = odometry_positions[:,i - 1] + np.array([delta_x, delta_y, delta_theta])
    return odometry_positions


# computes the cost function for the bag given the vector of robot parameters
def cost_function(bag, param_vector):
    # compute errors
    wheel_speeds_data_matrix_normalized = bag.wheel_speeds_matrix
    time_steps_data_matrix = bag.time_steps_matrix
    target_positions_data_matrix = bag.target_positions
    odometry_positions = compute_odometry_positions(wheel_speeds_data_matrix_normalized, time_steps_data_matrix, target_positions_data_matrix, param_vector)
    position_errors = target_positions_data_matrix - odometry_positions

    # apply weights (farther points are weighed less than points close to the start of the odometry)
    weights_x = np.logspace(1, 0.001, num=position_errors.shape[1]) # use np.ones(position_errors.shape[1]) to apply the same weight on all points
    weights_y = np.logspace(1, 0.001, num=position_errors.shape[1]) # use np.ones(position_errors.shape[1]) to apply less weight on farthest positions
    weights_theta = np.zeros(position_errors.shape[1]) # not considering error on theta
    weights = np.array([weights_x, weights_y, weights_theta])
    weight_errors = np.multiply(position_errors, weights)

    #return cost function
    return np.sum(np.square(weight_errors))
