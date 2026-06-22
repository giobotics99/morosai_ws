import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import struct
import chronoptics.tof as tof
from typing import List

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.cam = tof.TuiCamera("303000c")
        self.cam.setOutputFrameTypes([
            tof.FrameType.BGR, tof.FrameType.RADIAL, tof.FrameType.INTENSITY
        ])
        
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, 'gordon/rgb', 10)
        self.intensity_pub = self.create_publisher(Image, 'gordon/intensity', 10)
        self.depth_pub = self.create_publisher(Image, 'gordon/depth', 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'gordon/pointcloud', 10)
        
        self.cam.start()
        self.timer = self.create_timer(0.1, self.publish_frames)
    
    def get_frame(self, frames: List[tof.Data], frame_type: tof.FrameType):
        for frame in frames:
            if frame.frameType() == frame_type:
                return frame
        return None
    
    def publish_frames(self):
        frames = self.cam.getFrames()
        
        bgr_frame = self.get_frame(frames, tof.FrameType.BGR)
        i_frame = self.get_frame(frames, tof.FrameType.INTENSITY)
        r_frame = self.get_frame(frames, tof.FrameType.RADIAL)
        
        if bgr_frame is not None:
            bgr_array = np.flip(np.array(bgr_frame), axis=2)  # Convert BGR to RGB
            bgr_array = np.rot90(bgr_array, 2)  # Rotazione di 180 gradi
            msg = self.bridge.cv2_to_imgmsg(bgr_array, encoding='rgb8')
            self.rgb_pub.publish(msg)
        
        if i_frame is not None:
            intensity_array = np.array(i_frame).astype(np.uint8)
            intensity_array = np.rot90(intensity_array, 3)  # Ruotare se necessario
            msg = self.bridge.cv2_to_imgmsg(intensity_array, encoding='mono8')
            self.intensity_pub.publish(msg)
        
        if r_frame is not None:
            depth_array = np.array(r_frame).astype(np.uint16)
            depth_array = np.rot90(depth_array, 3)  # Ruota l'immagine di 270 gradi
            msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='16UC1')
            self.depth_pub.publish(msg)
            self.publish_pointcloud(depth_array)
    
    def publish_pointcloud(self, depth_array):
        height, width = depth_array.shape
        
        # non_zero_depth = depth_array[depth_array > 0]
        # if len(non_zero_depth) > 0:
        #     print(f"Un valore di depth valido: {non_zero_depth[0]}")
        # else:
        #     print("Nessun valore valido trovato!")

        # # Intrinsic parameters from the datasheet
        # scale_x = width / 804  # Fattore di scala rispetto alla risoluzione originale
        # scale_y = height / 672

        # fx = 281.48 * scale_x
        # fy = 336.0 * scale_y

        fx = 281.48  # Focal length in pixels (horizontal)
        fy = 336.0   # Focal length in pixels (vertical)

        cx = width / 2  # Principal point x (usually at the center of the image)
        cy = height / 2 # Principal point y (usually at the center of the image)

        print(f"Intrinsic parameters: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='pack', offset=12, datatype=PointField.FLOAT32, count=1),  # Aggiunto campo pack
        ]
        
        points = []
        for v in range(height):
            for u in range(width):
                # Get the depth value (in mm)
                z = float(depth_array[v, u])  # Depth value from the depth image
                
                if z == 0:
                    continue  # Skip points with no depth (invalid points)
                
                # Convert depth to meters
                z = z / 1000.0
                
                # Calculate the corresponding 3D coordinates (X, Y, Z)
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                # Applica una rotazione di 180 gradi attorno all'asse Z (camera capovolta)
                x_rot = -x
                y_rot = z
                z_rot = -y

                # Pack the point and add it to the list
                points.append(struct.pack('ffff', x_rot, y_rot, z_rot, 0.0))  # Aggiunto campo pack
        
        print(f"Generated {len(points)} valid 3D points")

        # Create the PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = "camera_link"
        pointcloud_msg.height = 1  # Set height to 1 for unordered point cloud
        pointcloud_msg.width = len(points)  # Set width to the number of valid points
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16  # 4 floats * 4 bytes each
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
        pointcloud_msg.is_dense = True  # Set to True if there are no invalid points
        pointcloud_msg.data = b''.join(points)
        
        print("Publishing PointCloud2 message...")

        # Publish the PointCloud2 message
        self.pc_pub.publish(pointcloud_msg)

    
    def shutdown(self):
        self.cam.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
