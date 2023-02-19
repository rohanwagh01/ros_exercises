#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ros_exercises.msg import OpenSpace
import math

def callback(data):
    publish_topic = rospy.get_param('/open_space_publisher/publish_topic', 'open_space')
    pub = rospy.Publisher(publish_topic, OpenSpace, queue_size=10)
    scans = data.ranges
    current_angle = data.angle_min
    angle_increment = data.angle_increment
    largest_distance = 0
    largest_angle = None
    for distance in scans:
        if distance > largest_distance:
            largest_distance = distance
            largest_angle = current_angle
        current_angle += angle_increment
    rospy.loginfo(largest_distance)
    rospy.loginfo(largest_angle)

    target = OpenSpace()
    target.angle = largest_angle
    target.distance = largest_distance
    pub.publish(target)

def open_space_publisher():

    rospy.init_node('open_space_publisher', anonymous=False)

    subscribe_topic = rospy.get_param('/open_space_publisher/subscribe_topic', 'fake_scan')
    rospy.Subscriber(subscribe_topic, LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
