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


class Subsriber:
    def __init__(self):
        self.actual_target_names = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_processes)

    def callback_processes(self, data):
        self.actual_target_names = data.name

    def loginfo(self):
        return self.actual_target_names


if __name__ == '__main__':
    rospy.init_node('target_control' + str(np.random.randint(1000)), anonymous=False)

    a = Subsriber()
    lista = []
    for i in range(5):
        b = a.loginfo()
        rospy.loginfo(b)
        lista.append(b)
    print(lista)
    new_index = 1
    for elem in lista:
        if elem is None:
            continue
        else:
            if elem[-1][0:-1] == 'target' and int(elem[-1][-1]) >= 1:
                new_index = int(elem[-1][-1]) + 1
                print(new_index)
            else:
                break
            print(3)
            break
    print(new_index)





