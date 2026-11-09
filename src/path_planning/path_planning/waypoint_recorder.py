#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
import numpy as np
import pandas as pd
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class WaypointsLogger(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('waypoints_logger')

        # Configure the output file path
        self.filename = ""  # Define file path for saving waypoints

        # Initialize the DataFrame to store waypoints
        self.waypoints_df = pd.DataFrame(columns=['x', 'y', 'w'])

        # Last recorded position (used to compare movements)
        self.last_position = None
        self.min_distance = 0.05  # Minimum distance to register a new waypoint (in meters)

        # Define the odometry topic to listen to
        self.odom_topic = ""  # Define the odometry topic here
        self.create_subscription(Odometry, self.odom_topic, self.save_waypoint, 10)

    def save_waypoint(self, data):
        """
        Callback function to save x, y, and w (orientation) from the robot.
        You should implement logic to save the waypoint under certain conditions.
        """
        pass  # Implement saving the waypoint here

    def has_moved_significantly(self, x, y):
        """
        Check if the robot has moved significantly (i.e., beyond the minimum distance).
        Implement logic to calculate the distance between the current and last position.
        """
        pass  # Implement distance check here

    def save_to_file(self):
        """
        Save the waypoints DataFrame to a CSV file.
        Implement logic to save the waypoints to a file when the node shuts down.
        """
        pass  # Implement saving to CSV here

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()

    try:
        # Run the main ROS loop to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure to save the waypoints when shutting down
        node.save_to_file()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
