#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix, euler_matrix, \
    euler_from_matrix
from numpy import matrix, array
from os import system


def circle(iter1=50, radius=30):
    alpha = np.linspace(0, 2 * np.pi, iter1)
    list_spiral = []
    r = radius
    x = np.round(r * np.cos(alpha))
    y = np.round(r * np.sin(alpha))
    for i, j in zip(x, y):
        list_spiral.append([i, j])
    return list_spiral, iter1, 2 * np.pi * radius


def line_x(start=-10, end=10, b=-1, iter1=40):
    x = np.linspace(start, end, iter1)
    y = x * 0 + b
    distance = end - start
    return x, y, iter1, np.abs(distance)


def line_y(start=-10, end=10, b=-1, iter1=40):
    y = np.linspace(start, end, iter1)
    x = y * 0 + b
    distance = end - start
    return x, y, iter1, np.abs(distance)


def generate_squre(b=5):
    a = b/2
    list_iter = []
    list_distance = []
    list_square = []
    move_plan = [[-a, a, -a], [-a, a, a], [a, -a, a], [a, -a, -a]]
    for i in range(len(move_plan)):
        current_plan = move_plan[i]
        if i % 2 == 0:
            x, y, iter1, distance = line_x(current_plan[0], current_plan[1], current_plan[2])
        else:
            x, y, iter1, distance = line_y(current_plan[0], current_plan[1], current_plan[2])
        list_distance.append(distance)
        list_iter.append(iter1)
        for n, m in zip(x, y):
            list_square.append([n, m])
    dist_sum = sum(list_distance)
    iter_sum = sum(list_iter)
    return list_square, iter_sum, dist_sum


def count_rospy_rate(v=2, iterations_move=100, s=5):
    distance_per_iter = s / iterations_move
    f = v / distance_per_iter
    return f


if __name__ == '__main__':

    rospy.init_node('target_control')

    landmark_pose = Pose()
    landmark_pose.position.x = 0.5
    landmark_pose.orientation.x = landmark_pose.orientation.y = landmark_pose.orientation.z = landmark_pose.orientation.w = 1

    # sdf = rospy.get_param('target_sdf')
    # tx = rospy.get_param('tx')
    # ty = rospy.get_param('ty')
    # tz = rospy.get_param('tz')
    sdf = '/home/bartosz/catkin_ws/src/calibration_gazebo/sdf/landmark.sdf'
    tx, ty, tz = '0', '0', '2'
    cmd_line = 'rosrun gazebo_ros spawn_model -model target -sdf -file ' + sdf + ' -x ' + tx + ' -y ' + ty + ' -z ' + tz + ' -R 0 -P 0 -Y 0 '
    system(cmd_line)

    pose_update = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = 'target'
    # mode = rospy.get_param('flight_mode')

    center_point = [0, 0, 5]
    mode = 3
    if mode == 1:
        points_trase, counter_iterations, distance = circle(200, 30)
    else:
        points_trase, counter_iterations, distance = generate_squre(5)
    pos = 0
    position = Pose()
    position.position.z = center_point[2]
    velocity = 2
    freq_rate = count_rospy_rate(velocity, counter_iterations, distance)
    rate = rospy.Rate(freq_rate)
    while not rospy.is_shutdown():
        current_xy = points_trase[pos]
        desired_xy = [current_xy[0] + center_point[0], current_xy[1] + center_point[1]]
        position.position.x = desired_xy[0]
        position.position.y = desired_xy[1]
        pos += 1
        if pos >= len(points_trase):
            pos = 0

        state.pose = position
        pose_update.call(state)
        rate.sleep()
