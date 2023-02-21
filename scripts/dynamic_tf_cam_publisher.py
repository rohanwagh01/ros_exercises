#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions
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

def dynamic_tf_cam_publisher():
	rospy.init_node('dynamic_tf_cam_publisher', anonymous=False)
	rate = rospy.Rate(20) # publish at 20hz
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	while not rospy.is_shutdown():
		try:
			world_to_base = tfBuffer.lookup_transform('world', 'base_link_gt', rospy.Time())
			trans = world_to_base.transform.translation
			rot = world_to_base.transform.rotation
			#create 4x4 tf for world to base
			quaternion = (rot.x, rot.y, rot.z, rot.w)
			translation = (trans.x, trans.y, trans.z)
			transformer = TransformerROS()
			matrix = transformer.fromTranslationRotation(translation, quaternion)
			#test_matrix = tf_conversions.posemath.toMatrix(tf_conversions.posemath.fromTf(world_to_base)
			#now use the 4x4 transform that goes from robot to left cam
			#no rotation, just a 0.05m movement in either directions
			base_to_left = np.array([[1, 0, 0, -0.05],
						 [0, 1, 0, 0],
						 [0, 0, 1, 0],
						 [0, 0, 0, 1]])

			left_to_right = np.array([[1, 0, 0, 0.1],
                                                 [0, 1, 0, 0],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]])
			#now compute the transform to left camera
			world_to_left = np.matmul(base_to_left, matrix)
			world_to_right = np.matmul(left_to_right, world_to_left)
			broadcast_tf(world_to_left, 'world', 'left_cam')
			broadcast_tf(left_to_right, 'left_cam', 'right_cam')
		except Exception as e:
			rospy.loginfo(e)
			pass
		rate.sleep()

if __name__ == '__main__':
	try:
		dynamic_tf_cam_publisher()
	except rospy.ROSInterruptException:
		pass
