#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from optical_head.msg import PgvScanData
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class VirtualCorridor(Node):
    def __init__(self):
        super().__init__('virtual_corridor_node_updated')

        # Parametri corridoio
        self.declare_parameter("corridor_width", 1.0)  # metri
        self.declare_parameter("corridor_length", 1.0)  # metri
        self.width = self.get_parameter("corridor_width").value
        self.length = self.get_parameter("corridor_length").value

        # Subscriber PGV
        self.sub = self.create_subscription(
            PgvScanData,
            '/pgv100_scan',
            self.callback,
            10
        )

        # Publisher PointCloud2 del corridoio
        self.pub = self.create_publisher(PointCloud2, '/virtual_corridor_cloud', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: PgvScanData):
        # Usa sempre lo stesso timestamp "now" per evitare extrapolation
        stamp = self.get_clock().now().to_msg()

        if msg.no_pos == 1:
            # Publish empty pointcloud per far sparire il corridoio
            empty = self.create_cloud([], stamp)
            self.pub.publish(empty)
            return

        # Punto locale sul robot
        tag_local = PointStamped()
        tag_local.header.stamp = stamp
        tag_local.header.frame_id = "base_footprint"
        tag_local.point.x = msg.x_pos / 1000.0
        tag_local.point.y = msg.y_pos / 1000.0
        tag_local.point.z = 0.0

        try:
            # Trasforma base_footprint -> odom
            t_odom = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                rclpy.time.Time(),  # usa "latest available"
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tag_odom = do_transform_point(tag_local, t_odom)

            # Trasforma odom -> map
            t_map = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tag_map = do_transform_point(tag_odom, t_map)

        except Exception as e:
            self.get_logger().warn(f"TF non trovata: {e}")
            return

        # Coordinate corridoio
        x_center = tag_map.point.x
        y_center = tag_map.point.y
        points = []

        for dx in np.linspace(-self.length/2, self.length/2, int(self.length / 0.05)):
            yl = y_center - self.width
            yr = y_center + self.width
            xl = x_center - self.length/2 + dx
            xr = x_center - self.length/2 + dx
            points.append([xl, yl, 0.0])
            points.append([xr, yr, 0.0])

        self.pub.publish(self.create_cloud(points, stamp))

    def create_cloud(self, pts, stamp):
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        return pc2.create_cloud(header, fields, pts)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCorridor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
Versione aggiornata del nodo VirtualCorridor per ROS2 con gestione corretta del TF e del timestamp.

Differenze principali rispetto alla versione originale:
1. Trasformazioni TF:
   - La versione originale lavorava solo in frame "base_footprint" e generava la PointCloud2 nello stesso frame.
   - La nuova versione trasforma i punti dal frame "base_footprint" a "odom" e poi a "map", allineando il corridoio virtuale alla mappa globale.
   - Usa sempre l'ultima trasformazione disponibile (`rclpy.time.Time()`) per evitare warning di "extrapolation into the future".

2. Gestione del timestamp:
   - Ora si usa `stamp = self.get_clock().now().to_msg()` per tutti i punti e la cloud, evitando problemi di clock o differenze temporali tra PGV e TF.
   - La versione originale generava il cloud con timestamp del messaggio PGV, rischiando che il TF non fosse disponibile a quel tempo.

3. Empty pointcloud:
   - La nuova versione pubblica una cloud vuota quando `no_pos==1` per far scomparire il corridoio, mentre l originale non lo gestiva.

4. Frame della PointCloud2:
   - Nella nuova versione il cloud viene pubblicato in frame "map", coerente con la mappa globale.
   - Nell originale era pubblicata in "base_footprint", quindi solo locale e non trasformata nella mappa.

In pratica, questa versione è compatibile con un robot reale, utilizza le trasformazioni corrette, evita warning di TF e mantiene la sincronizzazione tra sensori e mappa.
"""

