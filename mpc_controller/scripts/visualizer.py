#!/usr/bin/env python3

import csv
import rospy
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class WaypointPublisher:
    def __init__(self):
        self.x = None
        self.x_pos = None
        self.marker = self.marker_publisher()
        self.path_msg = self.csv_to_path()
        self.pub = rospy.Publisher('waypoints', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('marker', Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber('/gem/base_footprint/odom', Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10) # 10 Hz

    def marker_publisher(self):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "base_footprint"  # Set the marker frame to "odom"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Set the initial position of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        return marker

    def csv_to_path(self):
        # read the CSV file and extract the waypoints
        waypoints = []
        
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/straight.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
            
            for point in path_points:
                x= float(point[0])
                y= float(point[1])
                z= float(point[2])
                waypoints.append((x, y, z))

        # convert the waypoints to a Path message
        path_msg = Path()
        path_msg.header.frame_id = "base_footprint"  # Assign a frame ID here
        for waypoint in waypoints:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = waypoint[0]
            pose_msg.pose.position.y = waypoint[1]
            pose_msg.pose.position.z = waypoint[2]
            path_msg.poses.append(pose_msg)

        return path_msg

    def odom_callback(self, msg):
        self.x_pos =msg.pose.pose.position.x


    def run(self):
        while not rospy.is_shutdown():
            if self.x_pos is not None:
                self.marker.pose.position.x = self.x_pos
            self.pub.publish(self.path_msg)
            self.marker_pub.publish(self.marker)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')
    waypoint_publisher = WaypointPublisher()
    waypoint_publisher.run()

