#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros

# Initialize publishers
marker_pub = None
tf_broadcaster = None

def pose_callback(msg):
    # Publish transform between world and tracker
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = msg.header.frame_id
    # transform.child_frame_id = "tracker"

    # Set translation and rotation from pose
    transform.transform.translation.x = msg.pose.position.x
    transform.transform.translation.y = msg.pose.position.y
    transform.transform.translation.z = msg.pose.position.z
    transform.transform.rotation = msg.pose.orientation
    tf_broadcaster.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('pose_to_tf_broadcaster')
    rospy.loginfo("pose_to_tf_broadcaster node started")    
    
    # Initialize tf broadcaster and marker publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # Subscribe to the survive/pose topic
    rospy.loginfo("Subscribing to /survive/pose")
    rospy.Subscriber('/survive/pose', PoseStamped, pose_callback)
    
    rospy.spin()
