#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

def callback(data):
    pub_distance = rospy.Publisher('open_space/distance', Float32, queue_size=10)
    pub_angle = rospy.Publisher('open_space/angle', Float32, queue_size=10)
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
    pub_angle.publish(largest_angle)
    pub_distance.publish(largest_distance)

def open_space_publisher():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('open_space_publisher', anonymous=False)

    rospy.Subscriber("fake_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    open_space_publisher()
