import rospy
import numpy
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Vector3, Point, Quaternion
from tf.transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix, euler_matrix, \
    euler_from_matrix
from numpy import matrix, array
from os import system


class MovingDrone:
    def __init__(self):
        self.velocities = Twist()
        self.orientation = Quaternion()
        self.position = Point()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_position)
        rospy.Subscriber('/calibration_gazebo/landmark_vel', Twist, self.callback_velocity)
        self.twist = Twist()
        self.pub = rospy.Publisher('/calibration_gazebo/landmark_vel', Twist, queue_size=1)

    def callback_velocity(self, data):
        self.velocities = data

    def callback_position(self, data):
        self.position = data.pose[2].position
        self.orientation = data.pose[2].orientation

    def does(self):
        rospy.loginfo(str(self.velocities) + "aaaa" + str(self.position))

    def talker(self):
        rate = rospy.Rate(30)  # 10hz
        while not rospy.is_shutdown():
            self.twist.linear.y = 0
            self.twist.linear.x = 0
            self.twist.angular.x = 0
            self.pub.publish(self.twist)
            rate.sleep()


# def callback_velocity(data):
#     rospy.loginfo("Velocity " + str(data))
#
#
# def callback_position(data):
#     rospy.loginfo("Position " + str(data.pose[2].position))


# def publisher():
#     rospy.Subscriber('/gazebo/model_states', ModelStates, callback_position)
#     rospy.Subscriber('/calibration_gazebo/landmark_vel', Twist, callback_velocity)

# def talker():
#     pub = rospy.Publisher('/calibration_gazebo/landmark_vel', Twist, queue_size=1)
#     rate = rospy.Rate(30)  # 10hz
#     while not rospy.is_shutdown():
#         twist = Twist()
#         twist.linear.y = 0
#         twist.linear.x = 0
#         twist.angular.x = 0
#         pub.publish(twist)
#         rate.sleep()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    a = MovingDrone()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        a.does()
        rate.sleep()
