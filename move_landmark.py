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


def circle(step=50, radius=30):
    alpha = np.linspace(0, 2 * np.pi, step)
    list_spiral = []
    r = radius
    x = np.round(r * np.cos(alpha))
    y = np.round(r * np.sin(alpha))
    for i, j in zip(x, y):
        list_spiral.append([i, j])
    return list_spiral


def line_x(start=-10, end=10, b=-1):
    x = np.linspace(start, end, 100)
    y = x * 0 + b
    return x, y


def line_y(start=-10, end=10, b=-1):
    y = np.linspace(start, end, 100)
    x = y * 0 + b
    return x, y


def generate_squre(a=5):
    list_x = []
    list_y = []
    x, y = line_x(-a, a, -a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_y(-a, a, a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_x(a, -a, a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_y(a, -a, -a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    list_square = []
    for i, j in zip(list_x, list_y):
        list_square.append([i, j])

    return list_square


def Homogeneous(pose):
    R = matrix(quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
    t = matrix(translation_matrix([pose.position.x, pose.position.y, pose.position.z]))
    return t * R


class Listener:
    def __init__(self, dt):
        self.dt = dt
        self.cam_link = ''
        self.sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_callback)
        self.pose = Pose()
        self.pose.orientation.w = 1
        self.pose.orientation.y = -1
        self.pose.position.z = 0
        self.msg_ok = False
        self.linear = [0, 0, 0]
        self.angular = [0, 0, 0]

    def link_callback(self, msg):
        self.msg_ok = True
        # try to find a link called camera or something
        for i, name in enumerate(msg.name):
            if 'camera' in name.split('::')[1]:
                # register this link as camera
                self.cam_link = name
                self.pose = msg.pose[i]
                break


if __name__ == '__main__':

    rospy.init_node('calib_bridge')
    dt = 0.1
    rate = rospy.Rate(24)

    # try to get camera link to spawn landmark
    listener = Listener(dt)

    count = 0
    while not rospy.is_shutdown() and not listener.msg_ok and count < 100:
        print(listener.msg_ok)
        count += 1
        rate.sleep()

    landmark_pose = Pose()
    landmark_pose.position.x = 0.5
    landmark_pose.orientation.x = landmark_pose.orientation.y = landmark_pose.orientation.z = landmark_pose.orientation.w = 1
    M = Homogeneous(listener.pose) * Homogeneous(landmark_pose)

    # get XYZ - RPY
    label = ['x', 'y', 'z', 'R', 'P', 'Y']
    values = list(array(M[:3, 3]).flatten()) + list(euler_from_matrix(M))

    sdf = rospy.get_param('landmark_file')
    cmd_line = 'rosrun gazebo_ros spawn_model -model landmark -sdf -file ' + sdf
    for i in range(6):
        cmd_line += ' -' + label[i] + ' ' + str(values[i])
    system(cmd_line)

    pose_update = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = 'landmark'
    mode = rospy.get_param('flight_mode')

    center_point = [0, 0, 5]

    if mode == 1:
        points_trase = circle(200, 30)
    else:
        points_trase = generate_squre(5)
    pos = 0
    position = Pose()
    position.position.z = center_point[2]
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
