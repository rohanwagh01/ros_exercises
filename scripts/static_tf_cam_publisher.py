#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg


def get_static_tf(parent, child, translation_x):
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child

    static_transformStamped.transform.translation.x = translation_x
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0

    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    return static_transformStamped


if __name__ == '__main__':

    rospy.init_node('static_tf_cam_publisher')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    left = get_static_tf('base_link_gt', 'left_cam', -0.05)
    right = get_static_tf('base_link_gt', 'right_cam', 0.05)
    broadcaster.sendTransform([left, right])

    rospy.spin()
