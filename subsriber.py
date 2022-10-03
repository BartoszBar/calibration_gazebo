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
import rosnode

if __name__ == '__main__':
    list_str = ['/landmark_bridge', '/rosout', '/gazebo_gui', '/gazebo', '/target_control2', '/target_control3',
                '/target_control5']
    index_node = 1
    filtered_nodes = list(filter(lambda x: '/target_control' in x, list_str))

    if '/landmark_bridge' in list_str and len(filtered_nodes) == 0:
        index_node += 1
    else:
        list_target_numbers = filtered_nodes
        list_numbers = []
        for word in list_target_numbers:
            word_number = word.replace('/target_control', '')
            list_numbers.append(int(word_number))
        index_node = max(list_numbers) + 1
