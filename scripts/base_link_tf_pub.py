#!/usr/bin/env python
import rospy

# Because of transformations
import tf.transformations as tft
from tf.listener import TransformerROS
import tf2_ros
import geometry_msgs.msg
import numpy as np


def broadcast_tf(matrix, parent, child):
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = parent
	t.child_frame_id = child
	t.transform.translation.x = matrix[0,3]
	t.transform.translation.y = matrix[1,3]
	t.transform.translation.z = matrix[2,3]
	q = tft.quaternion_from_matrix(matrix)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	br.sendTransform(t)

def base_link_tf_pub():
	rospy.init_node('base_link_tf_pub', anonymous=False)
	rate = rospy.Rate(20) # publish at 20hz
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	while not rospy.is_shutdown():
		try:
			world_to_left = tfBuffer.lookup_transform('world', 'left_cam', rospy.Time())
			trans = world_to_left.transform.translation
			rot = world_to_left.transform.rotation
			#create 4x4 tf for world to base
			quaternion = (rot.x, rot.y, rot.z, rot.w)
			translation = (trans.x, trans.y, trans.z)
			transformer = TransformerROS()
			matrix = transformer.fromTranslationRotation(translation, quaternion)
			#test_matrix = tf_conversions.posemath.toMatrix(tf_conversions.posemath.fromTf(world_to_base)
			#now use the 4x4 transform that goes from robot to left cam
			#no rotation, just a 0.05m movement in either directions
			left_to_base = np.array([[1, 0, 0, 0.05],
						 [0, 1, 0, 0],
						 [0, 0, 1, 0],
						 [0, 0, 0, 1]])

			#now compute the transform to left camera
			world_to_base = np.matmul(left_to_base, matrix)
			broadcast_tf(world_to_base, 'world', 'base_link_gt_2')
		except Exception as e:
			rospy.loginfo(e)
			pass
		rate.sleep()

if __name__ == '__main__':
	try:
		base_link_tf_pub()
	except rospy.ROSInterruptException:
		pass
