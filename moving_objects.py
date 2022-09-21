import rospy
import numpy
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix, euler_matrix, \
    euler_from_matrix
from numpy import matrix, array
from os import system



def callback_velocity(data):
    rospy.loginfo("Velocity " + str(data))
    # return data


def callback_position(data):
    rospy.loginfo("Position " + str(data.pose[2].position))
    # return data.pose[2].position


def publisher():
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_position)
    rospy.Subscriber('/calibration_gazebo/landmark_vel', Twist, callback_velocity)


def talker():
    pub = rospy.Publisher('/calibration_gazebo/landmark_vel', Twist, queue_size=1)
    rate = rospy.Rate(30)  # 10hz
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.y = 0
        twist.linear.x = 0
        twist.angular.x = 0
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=False)
    publisher()
    talker()
