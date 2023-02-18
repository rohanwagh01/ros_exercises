#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import random
import math

def fake_scan_publisher():
	pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
	rospy.init_node('fake_scan_publisher', anonymous=False)
	rate = rospy.Rate(20) # publish at 20hz
	#now each time, make a new laser scan message that is f-f-f-fake
	while not rospy.is_shutdown():
		rand_scan  = LaserScan()
		rand_scan.header.stamp = rospy.Time.now()
		rand_scan.header.frame_id = "base_link"
		rand_scan.angle_min = -2.0/3.0*math.pi
		rand_scan.angle_max = 2.0/3.0*math.pi
		rand_scan.angle_increment = (1.0/300.0)*math.pi
		rand_scan.scan_time = (1.0/20.0)/401.0
		rand_scan.range_min = 1.0
		rand_scan.range_max = 10.0

    	#now make ranges
		ranges = []

		for item in range(401):
			ranges.append(random.random()*9+1.0)
		rand_scan.ranges = ranges
		#now publish the fake scan
		pub.publish(rand_scan)
		rospy.loginfo(rand_scan)
		rate.sleep()

if __name__ == '__main__':
	fake_scan_publisher()
