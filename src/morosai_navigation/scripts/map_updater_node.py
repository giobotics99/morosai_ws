#!/usr/bin/env python3
"""
Nodo ROS2 standalone (da integrare in un pacchetto a scelta dell'utente).

Si iscrive a un topic nav_msgs/msg/OccupancyGrid (default: /map_new), scrive
map.pgm + map.yaml su disco nel formato standard letto da nav2_map_server, e
chiama il servizio /map_server/load_map (nav2_msgs/srv/LoadMap) per far
ricaricare la mappa a Nav2.

Dipendenze: rclpy, nav_msgs, nav2_msgs, numpy.

Parametri (tutti riconfigurabili via launch/CLI):
  map_new_topic     (string, default '/map_new')
  map_directory      (string, default '/tmp/nav2_maps')
  map_name           (string, default 'map')            -> map_name.pgm / map_name.yaml
  load_map_service   (string, default '/map_server/load_map')
  update_period_sec  (double, default 2.0)  -> throttling: max una scrittura+reload ogni N secondi

  clear_costmaps_on_update    (bool, default True)
  global_costmap_clear_service (string, default '/global_costmap/clear_entirely_global_costmap')
  local_costmap_clear_service  (string, default '/local_costmap/clear_entirely_local_costmap')

  reinitialize_amcl_on_update (bool, default False)  -> disruttivo: il robot deve rilocalizzarsi da zero
  amcl_reinit_service          (string, default '/reinitialize_global_localization')
"""

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from std_srvs.srv import Empty

# Soglie standard usate da nav2_map_server per il round-trip pgm<->OccupancyGrid:
# con negate=0, un pixel p viene letto come occ = (255-p)/255 e confrontato con
# queste soglie. Con questi valori: 0->free, 100->occupied, -1->pixel 205->unknown.
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
NEGATE = 0


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def occupancy_grid_to_pgm_bytes(msg: OccupancyGrid) -> bytes:
    width = msg.info.width
    height = msg.info.height
    grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

    # riga 0 dell'immagine (in alto) deve corrispondere alla riga con y massima
    # della griglia (in basso a sinistra e' l'origine per convenzione OccupancyGrid).
    img = np.flipud(grid)

    pixel = np.full(img.shape, 205, dtype=np.uint8)  # default: ignoto (grigio)
    known = img != -1
    values = img[known].astype(np.float64)
    pixel[known] = np.clip(np.round(254.0 - values * 2.54), 0, 254).astype(np.uint8)

    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    return header + pixel.tobytes()


def write_map_files(msg: OccupancyGrid, directory: str, name: str) -> str:
    os.makedirs(directory, exist_ok=True)
    pgm_path = os.path.join(directory, f"{name}.pgm")
    yaml_path = os.path.join(directory, f"{name}.yaml")

    pgm_tmp = pgm_path + ".tmp"
    yaml_tmp = yaml_path + ".tmp"

    with open(pgm_tmp, "wb") as f:
        f.write(occupancy_grid_to_pgm_bytes(msg))

    origin = msg.info.origin
    yaw = quaternion_to_yaw(origin.orientation)
    yaml_content = (
        f"image: {name}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {msg.info.resolution:.6f}\n"
        f"origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, {yaw:.6f}]\n"
        f"negate: {NEGATE}\n"
        f"occupied_thresh: {OCCUPIED_THRESH:.6f}\n"
        f"free_thresh: {FREE_THRESH:.6f}\n"
    )
    with open(yaml_tmp, "w") as f:
        f.write(yaml_content)

    # rename atomico: evita che map_server legga file a meta' scrittura
    os.replace(pgm_tmp, pgm_path)
    os.replace(yaml_tmp, yaml_path)
    return yaml_path


class MapUpdaterNode(Node):
    def __init__(self):
        super().__init__("map_updater_node")

        self.declare_parameter("map_new_topic", "/map_new")
        self.declare_parameter("map_directory", "/tmp/nav2_maps")
        self.declare_parameter("map_name", "map")
        self.declare_parameter("load_map_service", "/map_server/load_map")
        self.declare_parameter("update_period_sec", 2.0)
        self.declare_parameter("clear_costmaps_on_update", True)
        self.declare_parameter("global_costmap_clear_service", "/global_costmap/clear_entirely_global_costmap")
        self.declare_parameter("local_costmap_clear_service", "/local_costmap/clear_entirely_local_costmap")
        self.declare_parameter("reinitialize_amcl_on_update", False)
        self.declare_parameter("amcl_reinit_service", "/reinitialize_global_localization")

        self._map_directory = self.get_parameter("map_directory").value
        self._map_name = self.get_parameter("map_name").value
        self._update_period = float(self.get_parameter("update_period_sec").value)
        self._clear_costmaps = bool(self.get_parameter("clear_costmaps_on_update").value)
        self._reinit_amcl = bool(self.get_parameter("reinitialize_amcl_on_update").value)

        map_new_topic = self.get_parameter("map_new_topic").value
        load_map_service = self.get_parameter("load_map_service").value
        global_costmap_clear_service = self.get_parameter("global_costmap_clear_service").value
        local_costmap_clear_service = self.get_parameter("local_costmap_clear_service").value
        amcl_reinit_service = self.get_parameter("amcl_reinit_service").value

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(OccupancyGrid, map_new_topic, self._on_map_new, qos)
        self._load_map_client = self.create_client(LoadMap, load_map_service)
        self._global_costmap_clear_client = self.create_client(ClearEntireCostmap, global_costmap_clear_service)
        self._local_costmap_clear_client = self.create_client(ClearEntireCostmap, local_costmap_clear_service)
        self._amcl_reinit_client = self.create_client(Empty, amcl_reinit_service)

        self._pending_msg = None
        self._busy = False
        self._last_update_time = 0.0
        self._timer = self.create_timer(0.5, self._process_pending)

        self.get_logger().info(
            f"In ascolto su '{map_new_topic}', scrittura mappe in '{self._map_directory}', "
            f"reload via '{load_map_service}' (periodo min. {self._update_period:.1f}s)"
        )

    def _on_map_new(self, msg: OccupancyGrid):
        self._pending_msg = msg

    def _process_pending(self):
        if self._pending_msg is None or self._busy:
            return
        now = time.monotonic()
        if now - self._last_update_time < self._update_period:
            return

        msg = self._pending_msg
        self._pending_msg = None
        self._busy = True
        self._last_update_time = now

        try:
            yaml_path = write_map_files(msg, self._map_directory, self._map_name)
        except OSError as e:
            self.get_logger().error(f"Scrittura mappa su disco fallita: {e}")
            self._busy = False
            return

        self._call_load_map(yaml_path)

    def _call_load_map(self, yaml_path: str):
        if not self._load_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f"Servizio '{self._load_map_client.srv_name}' non disponibile, mappa scritta ma non ricaricata"
            )
            self._busy = False
            return

        req = LoadMap.Request()
        req.map_url = yaml_path
        future = self._load_map_client.call_async(req)
        future.add_done_callback(self._on_load_map_response)

    def _on_load_map_response(self, future):
        self._busy = False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Chiamata a load_map fallita: {e}")
            return

        if response.result == LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().info("Mappa aggiornata con successo")
            if self._clear_costmaps:
                self._call_empty_style(self._global_costmap_clear_client, ClearEntireCostmap.Request(), "clear global_costmap")
                self._call_empty_style(self._local_costmap_clear_client, ClearEntireCostmap.Request(), "clear local_costmap")
            if self._reinit_amcl:
                self._call_empty_style(self._amcl_reinit_client, Empty.Request(), "reinitialize_global_localization")
        else:
            self.get_logger().error(f"load_map ha restituito errore, codice={response.result}")

    def _call_empty_style(self, client, request, label: str):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Servizio '{client.srv_name}' non disponibile, salto '{label}'")
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._on_empty_style_response(f, label))

    def _on_empty_style_response(self, future, label: str):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"'{label}' fallita: {e}")
        else:
            self.get_logger().info(f"'{label}' completata")


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
