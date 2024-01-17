import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import TrackedObjects
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
import csv
import cv2
import numpy as np
from pypcd import pypcd
from pathlib import Path
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TrackedObjectsToCSV(Node):

    def __init__(self):
        super().__init__('tracked_objects_to_csv')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            TrackedObjects,
            '/tracked_objects',
            self.listener_callback,
            qos_profile=qos_profile
        )
        self.csv_file_path = ""
        self.initialize_csv()
        self.sequence_number = -1

    def initialize_csv(self):
        with open(self.csv_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            header = ['timestamp', 'sequence_number', 'object_id', 'object_class', 'x_position', 'y_position', 'z_position', 
                      'x_dimension', 'y_dimension', 'z_dimension', 'quaternion_x', 'quaternion_y', 'quaternion_z', 'quaternion_w',
                      'velocity_x', 'velocity_y', 'velocity_z']
            writer.writerow(header)

    def listener_callback(self, msg):
        #print("listener_callback")
        self.sequence_number += 1
        with open(self.csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for obj in msg.objects:
                row = [
                    float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9, 
                    self.sequence_number,
                    bytes(obj.object_id.uuid).hex(),
                    obj.classification[0].label if obj.classification else 'Unknown',
                    obj.kinematics.pose_with_covariance.pose.position.x,
                    obj.kinematics.pose_with_covariance.pose.position.y,
                    obj.kinematics.pose_with_covariance.pose.position.z,
                    obj.shape.dimensions.x,
                    obj.shape.dimensions.y,
                    obj.shape.dimensions.z,
                    obj.kinematics.pose_with_covariance.pose.orientation.x,
                    obj.kinematics.pose_with_covariance.pose.orientation.y,
                    obj.kinematics.pose_with_covariance.pose.orientation.z,
                    obj.kinematics.pose_with_covariance.pose.orientation.w,
                    obj.kinematics.twist_with_covariance.twist.linear.x,
                    obj.kinematics.twist_with_covariance.twist.linear.y,
                    obj.kinematics.twist_with_covariance.twist.linear.z
                ]
                writer.writerow(row)

          

def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectsToCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
