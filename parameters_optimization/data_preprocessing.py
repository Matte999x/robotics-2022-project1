import pandas as pd
import numpy as np
from math import pi


def compute_wheel_speeds_matrix(bag_num):
    # import from csv
    wheel_states_df = pd.read_csv ("bag" + str(bag_num) + "/wheel_states.csv")

    # compute time steps
    wheel_states_df['delta_time'] = (wheel_states_df['field.header.stamp'] - wheel_states_df['field.header.stamp'].shift(1)) / 1e9

    # compute wheel speeds
    for wheel_idx in range(4):
        wheel_states_df['wheel' + str(wheel_idx) + '_speed'] = ((wheel_states_df['field.position' + str(wheel_idx)] - wheel_states_df['field.position' + str(wheel_idx)].shift(1)) / wheel_states_df['delta_time']) * 2 * pi

    # return wheel speeds data matrix (from /wheel_states topic)
    return wheel_states_df[['wheel0_speed', 'wheel1_speed', 'wheel2_speed', 'wheel3_speed']].to_numpy().transpose()


def compute_time_steps_matrix(bag_num):
    # import from csv
    wheel_states_df = pd.read_csv ("bag" + str(bag_num) + "/wheel_states.csv")

    # compute time steps
    wheel_states_df['delta_time'] = (wheel_states_df['field.header.stamp'] - wheel_states_df['field.header.stamp'].shift(1)) / 1e9

    # return time steps data matrices (from /wheel_states topic)
    return wheel_states_df['delta_time'].to_numpy().transpose()


def compute_target_positions(bag_num):
    # import from csv
    wheel_states_df = pd.read_csv ("bag" + str(bag_num) + "/wheel_states.csv")
    pose_df = pd.read_csv ("bag" + str(bag_num) + "/pose.csv")

    # convert pose quaternion to yaw
    pose_df['yaw'] = 2.0 * np.arctan2(pose_df['field.pose.orientation.z'], pose_df['field.pose.orientation.w'])
    
    # extract synchronized pose (Note: algorithm could be optimized)
    wheel_states_time = wheel_states_df['field.header.stamp'].values.tolist()
    poses_time = pose_df['field.header.stamp'].values.tolist()
    poses_x = pose_df['field.pose.position.x'].values.tolist()
    poses_y = pose_df['field.pose.position.y'].values.tolist()
    poses_theta = pose_df['yaw'].values.tolist()
    x = []
    y = []
    theta = []
    for i in range(len(wheel_states_time)):
        closest_idx = min(range(len(poses_time)), key=lambda j: abs(poses_time[j]-wheel_states_time[i]))

        second_to_closest_idx =  closest_idx
        if wheel_states_time[i] >= poses_time[closest_idx]:
            second_to_closest_idx = min(closest_idx + 1, len(poses_time) - 1)
        else:
            second_to_closest_idx = max(closest_idx - 1, 0)
        
        if poses_time[closest_idx] == poses_time[second_to_closest_idx]:
            alfa = 0
        else:
            alfa = abs((poses_time[closest_idx] - wheel_states_time[i]) / (poses_time[closest_idx] - poses_time[second_to_closest_idx]))

        x.append(poses_x[closest_idx] * (1 - alfa) + poses_x[second_to_closest_idx] * alfa)
        y.append(poses_y[closest_idx] * (1 - alfa) + poses_y[second_to_closest_idx] * alfa)
        theta.append(poses_theta[closest_idx] * (1 - alfa) + poses_theta[second_to_closest_idx] * alfa)

    # return target positions (real ground poses from /pose topic)
    return np.array([x ,y ,theta])


class Bag():
    def __init__(self, wheel_speeds_matrix, time_steps_matrix, target_positions):
        self.wheel_speeds_matrix = wheel_speeds_matrix
        self.time_steps_matrix = time_steps_matrix
        self.target_positions = target_positions
    
    def __init__(self, bag_num):
        print('Loading bag ' + str(bag_num) + ' ...')
        self.wheel_speeds_matrix = compute_wheel_speeds_matrix(bag_num)
        self.time_steps_matrix = compute_time_steps_matrix(bag_num)
        self.target_positions = compute_target_positions(bag_num)
        print('Done')
