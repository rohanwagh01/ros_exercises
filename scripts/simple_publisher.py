#!/usr/bin/env python
#Publisher Node: Produces a random number

import rospy
from std_msgs.msg import Float32
import random

def random_number_publisher():
	pub = rospy.Publisher('my_random_float', Float32, queue_size=10)
	rospy.init_node('simple_publisher', anonymous=False)
	rate = rospy.Rate(20) # publish at 20hz
	while not rospy.is_shutdown():
		rand_num = random.random()*10
		rospy.loginfo(rand_num)
		pub.publish(rand_num)
		rate.sleep()


if __name__ == '__main__':
	try:
		random_number_publisher()
	except rospy.ROSInterruptException:
		pass
