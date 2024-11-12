#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry
"""
@date: 2023/05
@auther: 陈笑阳
"""

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    base_link_pub = rospy.Publisher('cxy_base_link', Pose, queue_size=1)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            # 等待并获取 /map 到 /base_link 的tf变换
            transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            # 将获取到的位姿数据输出到控制台
            # now_pose = Pose()
            now_pose = Pose()
            now_pose.position.x = transform.transform.translation.x
            now_pose.position.y = transform.transform.translation.y
            now_pose.position.z = transform.transform.translation.z
            now_pose.orientation.x = transform.transform.rotation.x
            now_pose.orientation.y = transform.transform.rotation.y
            now_pose.orientation.z = transform.transform.rotation.z
            now_pose.orientation.w = transform.transform.rotation.w

            

            # orientation = Odometry()
            # orientation.pose.pose.position.x = transform.transform.translation.x
            # orientation.pose.pose.position.y = transform.transform.translation.y
            # orientation.pose.pose.position.z = transform.transform.translation.z
            # orientation.pose.pose.orientation.x = transform.transform.rotation.x
            # orientation.pose.pose.orientation.y = transform.transform.rotation.y
            # orientation.pose.pose.orientation.z = transform.transform.rotation.z
            # orientation.pose.pose.orientation.w = transform.transform.rotation.w
            base_link_pub.publish(now_pose)

            # print(f"Current position (x, y, z): {transform.transform.translation.x:.2f}, "
            #       f"{transform.transform.translation.y:.2f}, {transform.transform.translation.z:.2f}")
            # print(f"Current orientation (qx, qy, qz, qw): {transform.transform.rotation.x:.2f}, "
            #       f"{transform.transform.rotation.y:.2f}, {transform.transform.rotation.z:.2f}, "
            #       f"{transform.transform.rotation.w:.2f}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rospy.logwarn('Failed to get transform')
            pass
        rate.sleep()


