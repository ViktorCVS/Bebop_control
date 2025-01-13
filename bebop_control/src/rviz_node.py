#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node('simple_arrow_publisher')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Define marker 1
        marker1 = Marker()
        marker1.header.frame_id = "world"  # Change to your frame_id
        marker1.header.stamp = rospy.Time.now()
        marker1.ns = "simple_arrows"
        marker1.id = 0
        marker1.type = Marker.ARROW
        marker1.action = Marker.ADD
        marker1.pose.position = Point(0, 0, 0)
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 0.1  # Arrow head width
        marker1.scale.y = 0.5  # Arrow head length
        marker1.scale.z = 0.05  # Arrow shaft diameter
        marker1.color.r = 1.0  # Red
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        marker1.color.a = 1.0  # Alpha

        # Define marker 2
        marker2 = Marker()
        marker2.header.frame_id = "world"  # Change to your frame_id
        marker2.header.stamp = rospy.Time.now()
        marker2.ns = "simple_arrows"
        marker2.id = 1
        marker2.type = Marker.ARROW
        marker2.action = Marker.ADD
        marker2.pose.position = Point(0, 0, 0)
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 0.5  # Arrow head width
        marker2.scale.y = 0.06  # Arrow head length
        marker2.scale.z = 0.06  # Arrow shaft diameter
        marker2.color.r = 0.0  # Green
        marker2.color.g = 1.0
        marker2.color.b = 0.0
        marker2.color.a = 1.0  # Alpha

        # Publish markers
        pub.publish(marker1)
        pub.publish(marker2)

        rate.sleep()

if __name__ == '__main__':
    main()
