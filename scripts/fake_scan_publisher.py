#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import random
import math

def fake_scan_publisher():
	#get the topic
	publish_topic_name = rospy.get_param('/fake_scan_publisher/topic', 'fake_scan')
	pub = rospy.Publisher(publish_topic_name, LaserScan, queue_size=10)

	rospy.init_node('fake_scan_publisher', anonymous=False)

	#get the rate
	publish_rate = rospy.get_param('/fake_scan_publisher/rate', 20.0)
	rate = rospy.Rate(publish_rate) # publish at 20hz

	#now each time, make a new laser scan message that is f-f-f-fake
	while not rospy.is_shutdown():
		rand_scan  = LaserScan()
		rand_scan.header.stamp = rospy.Time.now()
		rand_scan.header.frame_id = "base_link"
		rand_scan.angle_min = rospy.get_param('/fake_scan_publisher/angle_min', -2.0/3.0*math.pi)
		rand_scan.angle_max = rospy.get_param('/fake_scan_publisher/angle_max', 2.0/3.0*math.pi)
		rand_scan.angle_increment = rospy.get_param('/fake_scan_publisher/angle_increment', (1.0/300.0)*math.pi)
		#calculate the number of scans in one pass
		num_scans = (rand_scan.angle_max - rand_scan.angle_min)/rand_scan.angle_increment
		rand_scan.scan_time = (1.0/publish_rate)/num_scans
		rand_scan.range_min = rospy.get_param('/fake_scan_publisher/range_min', 1.0)
		rand_scan.range_max = rospy.get_param('/fake_scan_publisher/range_max', 10.0)

    	#now make ranges
		ranges = []

		for item in range(int(num_scans)):
			ranges.append(random.random()*(rand_scan.range_max - rand_scan.range_min) + rand_scan.range_min)
		rand_scan.ranges = ranges
		#now publish the fake scan
		pub.publish(rand_scan)
		rospy.loginfo(rand_scan)
		rate.sleep()

if __name__ == '__main__':
	fake_scan_publisher()
