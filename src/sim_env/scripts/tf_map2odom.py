#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped


class TFMap2Odom:
    def __init__(self) -> None:
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.localization_topic = rospy.get_param("~localization_topic")

        self.tf_pub = tf2_ros.TransformBroadcaster()
        
        # Storage for the latest pose data
        self.latest_pose = None
        
        # dynamic: Subscribe to localization topic (e.g., AMCL or SLAM)
        self.localization_sub = rospy.Subscriber(
            self.localization_topic,
            PoseWithCovarianceStamped,
            self.localization_callback
        )
        rospy.loginfo(f"Subscribed to localization topic: {self.localization_topic}")
        
        # static
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        rospy.loginfo(f"TFMap2Odom node started. Publishing map -> {self.odom_frame_id}.")
        
    def localization_callback(self, msg):
        self.latest_pose = msg.pose.pose
        
    def timer_callback(self, event) -> None:
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = self.odom_frame_id
        
        if self.latest_pose is None:
            return
        
        odom.pose.pose.position.x = self.latest_pose.position.x
        odom.pose.pose.position.y = self.latest_pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.latest_pose.orientation

        self.tf = TransformStamped(
            header=Header(frame_id=odom.header.frame_id, stamp=odom.header.stamp),
            child_frame_id=odom.child_frame_id,
            transform=Transform(translation=odom.pose.pose.position, rotation=odom.pose.pose.orientation),
        )  
        
        # Publish the transform
        self.tf.header.stamp = rospy.Time.now()
        self.tf_pub.sendTransform(self.tf)


# Start the node
if __name__ == "__main__":
    rospy.init_node("tf_map2odom_node")

    node = TFMap2Odom()

    rospy.spin()
