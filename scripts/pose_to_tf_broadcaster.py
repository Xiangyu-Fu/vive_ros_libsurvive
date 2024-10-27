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
    # transform.child_frame_id = msg.header.frame_id
    transform.child_frame_id = "tracker"

    # Set translation and rotation from pose
    transform.transform.translation.x = msg.pose.position.x
    transform.transform.translation.y = msg.pose.position.y
    transform.transform.translation.z = msg.pose.position.z
    transform.transform.rotation = msg.pose.orientation
    tf_broadcaster.sendTransform(transform)

    # Publish a marker to represent the object
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "tracked_objects"
    marker.id = hash(msg.header.frame_id)  # Unique ID based on object name
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    # Set marker pose
    marker.pose.position = msg.pose.position
    marker.pose.orientation = msg.pose.orientation

    # Set marker scale and color
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Fully opaque

    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('pose_to_tf_broadcaster')
    rospy.loginfo("pose_to_tf_broadcaster node started")    
    
    # Initialize tf broadcaster and marker publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    
    # Subscribe to the survive/pose topic
    rospy.loginfo("Subscribing to /survive/pose")
    rospy.Subscriber('/survive/pose', PoseStamped, pose_callback)
    
    rospy.spin()
