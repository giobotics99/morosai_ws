#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np

class OdomToBaseFootprintBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_basefootprint_broadcaster')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Trasformazioni statiche:
        # base_footprint -> base_link
        self.basefootprint_to_baselink = self.make_transform(
            xyz=(0, 0, 0.04),
            rpy=(0, 0, 0),
            parent='base_footprint',
            child='base_link'
        )

        # base_link -> camera_link
        self.baselink_to_camera = self.make_transform(
            xyz=(0.25, 0.0, 0.286),  # POSIZIONE REALSENSE CAM
            rpy=(0, 0, 0),
            parent='base_link',
            child='camera_link'
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def make_transform(self, xyz, rpy, parent, child):
        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child
        # Cast esplicito a float
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        quat = tf_transformations.quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        return t


    def invert_transform(self, t):
        # Inverte una TransformStamped (posizione + rotazione)
        trans = [t.transform.translation.x,
                 t.transform.translation.y,
                 t.transform.translation.z]
        rot = [t.transform.rotation.x,
               t.transform.rotation.y,
               t.transform.rotation.z,
               t.transform.rotation.w]

        T = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(trans),
            tf_transformations.quaternion_matrix(rot)
        )
        T_inv = np.linalg.inv(T)

        trans_inv = tf_transformations.translation_from_matrix(T_inv)
        rot_inv = tf_transformations.quaternion_from_matrix(T_inv)

        inv = TransformStamped()
        inv.header = t.header
        inv.child_frame_id = t.header.frame_id  # invertito
        inv.header.frame_id = t.child_frame_id  # invertito
        inv.transform.translation.x = trans_inv[0]
        inv.transform.translation.y = trans_inv[1]
        inv.transform.translation.z = trans_inv[2]
        inv.transform.rotation.x = rot_inv[0]
        inv.transform.rotation.y = rot_inv[1]
        inv.transform.rotation.z = rot_inv[2]
        inv.transform.rotation.w = rot_inv[3]
        return inv

    def timer_callback(self):
        try:
            # Prendo la trasformazione odom -> camera_link
            odom_to_camera = self.tf_buffer.lookup_transform('odom', 'camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Non riesco a trovare odom->camera_link: {e}')
            return

        # Inverti baselink->camera_link per ottenere camera_link->baselink
        camera_to_baselink = self.invert_transform(self.baselink_to_camera)

        # Componi: odom->camera_link * camera_link->baselink = odom->baselink
        odom_to_baselink = self.compose_transforms(odom_to_camera, camera_to_baselink)
        
        # Inverti basefootprint->baselink per avere baselink->basefootprint
        baselink_to_basefootprint = self.invert_transform(self.basefootprint_to_baselink)

        # Componi: odom->baselink * baselink->basefootprint = odom->basefootprint
        odom_to_basefootprint = self.compose_transforms(odom_to_baselink, baselink_to_basefootprint)

        # Pubblica odom->base_footprint
        odom_to_basefootprint.header.stamp = self.get_clock().now().to_msg()
        odom_to_basefootprint.header.frame_id = 'odom'
        odom_to_basefootprint.child_frame_id = 'base_footprint'

        self.tf_broadcaster.sendTransform(odom_to_basefootprint)

    def compose_transforms(self, t1, t2):
        # Componi due TransformStamped t1 * t2
        # t1: parent->intermedio
        # t2: intermedio->child
        # risultato: parent->child

        # Converti in matrici 4x4
        trans1 = [t1.transform.translation.x,
                  t1.transform.translation.y,
                  t1.transform.translation.z]
        rot1 = [t1.transform.rotation.x,
                t1.transform.rotation.y,
                t1.transform.rotation.z,
                t1.transform.rotation.w]

        trans2 = [t2.transform.translation.x,
                  t2.transform.translation.y,
                  t2.transform.translation.z]
        rot2 = [t2.transform.rotation.x,
                t2.transform.rotation.y,
                t2.transform.rotation.z,
                t2.transform.rotation.w]

        T1 = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(trans1),
            tf_transformations.quaternion_matrix(rot1)
        )

        T2 = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(trans2),
            tf_transformations.quaternion_matrix(rot2)
        )

        T = np.dot(T1, T2)

        trans = tf_transformations.translation_from_matrix(T)
        rot = tf_transformations.quaternion_from_matrix(T)

        t = TransformStamped()
        t.header.frame_id = t1.header.frame_id
        t.child_frame_id = t2.child_frame_id
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]

        return t


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseFootprintBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
