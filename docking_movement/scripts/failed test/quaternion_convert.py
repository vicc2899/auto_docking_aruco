#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose_callback(data: PoseStamped):
    angle=Vector3()
    orientation_q=data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    rospy.loginfo("Pitch: %s",pitch)
    rospy.loginfo("y: %s\n",data.pose.position.x)

if __name__=='__main__':
    rospy.init_node('quartenion_convert')
    rospy.Subscriber("/aruco_single/pose", PoseStamped,pose_callback)
    euler_pub=rospy.Publisher("/euler_ang",Vector3,queue_size=10)
    rospy.loginfo("quarternion conversion node running")
    rospy.spin()