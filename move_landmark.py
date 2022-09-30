#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point
from mavros_msgs.msg import GlobalPositionTarget
from os import system
import math
import argparse



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
    dist = end - start
    return x, y, iter1, np.abs(dist)


def line_y(start=-10, end=10, b=-1, iter1=40):
    y = np.linspace(start, end, iter1)
    x = y * 0 + b
    dist = end - start
    return x, y, iter1, np.abs(dist)


def generate_line(a=10, iterations=40):
    move_plan = [[0, a, 0, iterations], [a, 0, 0, iterations]]
    list_iter_distance = []
    list_square = []
    for i in range(len(move_plan)):
        current_plan = move_plan[i]
        x, y, iter1, dist = line_x(current_plan[0], current_plan[1], current_plan[2], current_plan[3])
        list_iter_distance.append([iter1, dist])
        for n, m in zip(x, y):
            list_square.append([n, m])
    iter_sum, dist_sum = 0, 0
    for i in list_iter_distance:
        iter_sum += i[0]
        dist_sum += i[1]
    return list_square, iter_sum, dist_sum


def generate_squre(b=5):
    a = b / 2
    list_iter_distance = []
    list_square = []
    move_plan = [[-a, a, -a], [-a, a, a], [a, -a, a], [a, -a, -a]]
    for i in range(len(move_plan)):
        current_plan = move_plan[i]
        if i % 2 == 0:
            x, y, iter1, dist = line_x(current_plan[0], current_plan[1], current_plan[2])
        else:
            x, y, iter1, dist = line_y(current_plan[0], current_plan[1], current_plan[2])
        list_iter_distance.append([iter1, dist])
        for n, m in zip(x, y):
            list_square.append([n, m])
    iter_sum, dist_sum = 0, 0
    for i in list_iter_distance:
        iter_sum += i[0]
        dist_sum += i[1]

    return list_square, iter_sum, dist_sum


def count_rospy_rate(v=2, iterations_move=100, s=5):
    if v == 0 or iterations_move == 0 or s == 0:
        frequency = 1
    else:
        distance_per_iter = s / iterations_move
        frequency = v / distance_per_iter
    return frequency


def local_to_global(lat_now=47.397742, lon_now=8.5455934, altitude_now=535.3129164476948, v_n=0, v_e=0, h=0):
    """
    :param lat_now: latitude of drone on start point (when it is unarmed)
    :param lon_now: longitude of drone on start point (when it is unarmed)
    :param altitude_now: altitude of drone on start point (when it is unarmed)
    :param v_n: vector in x (local x position)
    :param v_e: vector in y (local y position)
    :param h: vector of height in local position
    :return:
    """

    const_radious_of_earth = 6371000
    lat_now_rad = math.radians(lat_now)
    lon_now_rad = math.radians(lon_now)
    lat_res = math.degrees(lat_now_rad + v_n / const_radious_of_earth)
    lon_res = math.degrees(lon_now_rad + v_e / const_radious_of_earth * np.cos(lat_now_rad))
    altitude_next = altitude_now + h
    return lat_res, lon_res, altitude_next

def publisher(radar_local_x, radar_local_y, radar_local_z):
    target_position = Point()
    target_position_pub = rospy.Publisher(
        '/radar/local_target_position' + str(index), Point, queue_size=1)

    target_global_position = GlobalPositionTarget()
    target_global_position_pub = rospy.Publisher(
        'radar/global_target_position' + str(index), GlobalPositionTarget, queue_size=1
    )

    target_position.x = radar_local_x
    target_position.y = radar_local_y
    target_position.z = radar_local_z
    target_global_position.latitude, target_global_position.longitude, target_global_position.altitude = \
        local_to_global(v_n=radar_local_x, v_e=radar_local_y, h=radar_local_z)

    target_position_pub.publish(target_position)
    target_global_position_pub.publish(target_global_position)


parser = argparse.ArgumentParser(description='Values of spawn drone, velocity and mode')
parser.add_argument('-x', '--x', type=float, metavar='', default=0, help='target co-ordinates of x')
parser.add_argument('-y', '--y', type=float, metavar='', default=0, help='target co-ordinates of y')
parser.add_argument('-z', '--z', type=float, metavar='', default=2, help='target co-ordinates of z')
parser.add_argument('-v', '--velocity', type=float, metavar='', default=2, help='velocity of moving target in m/s')
parser.add_argument('-s', '--sdffile', type=str, metavar='',
                    default='/home/bartosz/catkin_ws/src/velocityraptor/models/no_gravity_iris_easy/no_gravity_iris_easy.sdf',
                    help='model')
parser.add_argument('-m', '--mode', type=float, metavar='', default=2,
                    help='mode of moving target [1 - circle, 2 - square, 3 - line, else: state mode]')

args, unknown = parser.parse_known_args()

class Subsriber:
    def __init__(self):
        self.actual_target_names = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_processes)
    def callback_processes(self, data):
        self.actual_target_names = data.name
    def loginfo(self):
        return self.actual_target_names



if __name__ == '__main__':
    #here is init node with random name, bc if you run a lot of processes, they cannt name the same
    rospy.init_node('target_control' + str(np.random.randint(1000)), anonymous=False)

    landmark_pose = Pose()
    landmark_pose.position.x = 0.5
    landmark_pose.orientation.x = landmark_pose.orientation.y \
        = landmark_pose.orientation.z = landmark_pose.orientation.w = 1

    index = 1
    target_name = "target" + str(index)
    a = Subsriber()

    lista = []
    for i in range(5):
        b = a.loginfo()
        rospy.loginfo(b)
        lista.append(b)

    for elem in lista:
        if elem is None:
            continue
        else:
            if elem[-1][0:-1] == 'target' and int(elem[-1][-1]) >= 1:
                index = int(elem[-1][-1]) + 1
                target_name = "target" + str(index)
            else:
                break
            break

    sdf = args.sdffile
    tx, ty, tz = str(args.x), str(args.y), str(args.z)
    cmd_line = 'rosrun gazebo_ros spawn_model' + ' -model ' + target_name + ' -sdf -file ' + sdf + ' -x ' + tx + ' -y ' + ty + ' -z ' + \
               tz + ' -R 0 -P 0 -Y 0 '
    system(cmd_line)


    pose_update = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = target_name

    center_point = [0, 0, 5]

    mode = float(args.mode)
    velocity = float(args.velocity)

    if mode == 1:
        points_trase, counter_iterations, distance = circle(200, 30)
    elif mode == 2:
        points_trase, counter_iterations, distance = generate_squre(5)
    elif mode == 3:
        points_trase, counter_iterations, distance = generate_line(10)
    else:
        state_position = [[3, 3]]
        points_trase, counter_iterations, distance = state_position, 0, 0

    pos = 0
    position = Pose()
    position.position.z = center_point[2]

    freq_rate = count_rospy_rate(velocity, counter_iterations, distance)
    rate = rospy.Rate(freq_rate)


    while not rospy.is_shutdown():

        current_xy = points_trase[pos]
        desired_xy = [current_xy[0] + center_point[0], current_xy[1] + center_point[1]]
        position.position.x = desired_xy[0]
        position.position.y = desired_xy[1]
        publisher(position.position.x, position.position.y, position.position.z)
        pos += 1
        if pos >= len(points_trase):
            pos = 0

        state.pose = position
        pose_update.call(state)
        rate.sleep()
