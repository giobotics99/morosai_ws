#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

class TofIntensityFilterNode(Node):
    def __init__(self):
        super().__init__('tof_intensity_filter_node')
        
        # Declare ROS 2 parameter for terminal configuration
        # self.declare_parameter('intensity_threshold_up', 250)
        self.declare_parameter('intensity_threshold_down', 10)
        
        self.bridge = CvBridge()
        
        # Subscriptions
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/gordon_tof/pointcloud', qos_profile=qos_profile_sensor_data)
        self.intensity_sub = message_filters.Subscriber(self, Image, '/gordon_tof/intensity', qos_profile=qos_profile_sensor_data)
        
        # ExactTime synchronizer is used since the C++ node shares the same stamp across all frame types
        self.sync = message_filters.TimeSynchronizer([self.pc_sub, self.intensity_sub], queue_size=10)
        self.sync.registerCallback(self.sync_callback)
        
        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/gordon_tof/pointcloud_filtered', 10)
        self.intensity_pub = self.create_publisher(Image, '/gordon_tof/intensity_filtered', 10)
        
        self.get_logger().info("ToF Intensity Filter Node initialized and waiting for synchronized data.")

    def sync_callback(self, pc_msg: PointCloud2, intensity_msg: Image):
        # Fetch current threshold parameters
        # threshold_up = self.get_parameter('intensity_threshold_up').value
        threshold_down = self.get_parameter('intensity_threshold_down').value

        # 1. Process Intensity Image
        # Convert from ROS Image to OpenCV Mat (mono8)
        cv_image = self.bridge.imgmsg_to_cv2(intensity_msg, desired_encoding='passthrough')
        
        # Revert the 90-degree clockwise rotation applied in the C++ node to align with the PointCloud
        cv_image_aligned = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # Flatten to match the 1D structure of the PointCloud2 data
        intensity_flat = cv_image_aligned.flatten()
        
        # 2. Extract PointCloud Data
        points = list(pc2.read_points(pc_msg, skip_nans=False))
        
        if len(points) != len(intensity_flat):
            self.get_logger().warn(
                f"Data size mismatch: PointCloud contains {len(points)} points, "
                f"but Intensity image contains {len(intensity_flat)} pixels."
            )
            return

        # 3. Apply Intensity Filter to PointCloud
        # Retain only points where the intensity is less than or equal to the threshold
        filtered_points = [
            pt for i, pt in enumerate(points) 
            # if threshold_down <= intensity_flat[i] <= threshold_up
            if intensity_flat[i] >= threshold_down
        ]

        # 4. Generate and Publish Filtered Intensity Image (Visual Debug)
        # Sfruttiamo NumPy per applicare la maschera vettorialmente (ottimizzato per real-time)
        filtered_img_aligned = cv_image_aligned.copy()
        # I pixel sopra la soglia vengono oscurati (impostati a 0/nero)
        # filtered_img_aligned[filtered_img_aligned > threshold_up] = 0
        filtered_img_aligned[filtered_img_aligned < threshold_down] = 0
        
        # Ruotiamo di nuovo in senso orario per matchare l'orientamento originale del nodo C++
        filtered_img_pub = cv2.rotate(filtered_img_aligned, cv2.ROTATE_90_CLOCKWISE)
        
        filtered_intensity_msg = self.bridge.cv2_to_imgmsg(filtered_img_pub, encoding='mono8')
        filtered_intensity_msg.header = intensity_msg.header
        self.intensity_pub.publish(filtered_intensity_msg)
        
        # 5. Construct and Publish Filtered PointCloud2
        header = Header()
        header.stamp = pc_msg.header.stamp
        header.frame_id = pc_msg.header.frame_id
        
        # Create a new PointCloud2 message preserving the original fields (x, y, z, rgb)
        filtered_pc_msg = pc2.create_cloud(header, pc_msg.fields, filtered_points)
        
        # Maintain unordered/unorganized state (standard post-filtering)
        filtered_pc_msg.is_dense = True
        
        self.pc_pub.publish(filtered_pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TofIntensityFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()