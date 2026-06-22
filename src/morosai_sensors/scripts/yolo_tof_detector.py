#!/usr/bin/env python3

"""
MOROSAI AGV: YOLO Object Detection Node (ToF Camera)

Subscribes to the ToF camera BGR image topic and runs YOLO inference
to detect only the target classes: 'person' and 'box'.

Subscriptions:
  /gordon_tof/bgr            sensor_msgs/Image   (bgr8)

Publications:
  /yolo_detections/image     sensor_msgs/Image   (bgr8, annotated)
  /yolo_detections/results   std_msgs/String     (JSON array of detections)

Each detection in the JSON has:
  {
    "class_name": str,
    "class_id":   int,
    "confidence": float,
    "bbox": [x1, y1, x2, y2]   # pixel coords
  }
"""

import os
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np
from cv_bridge import CvBridge

# Ultralytics YOLO ─────────────────────────────────────────────────────────────
try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError(
        "ultralytics package not found. Install it with:\n"
        "  pip install ultralytics"
    ) from e


# ──────────────────────────────────────────────────────────────────────────────
# Colour palette for bounding boxes (one per target class, BGR)
# ──────────────────────────────────────────────────────────────────────────────
CLASS_COLORS = {
    "person": (0, 120, 255),   # orange-ish
    "box":    (0, 220, 120),   # green-ish
    "chair":  (120, 120, 0),   # blue-ish
}
DEFAULT_COLOR = (180, 180, 180)


class YoloTofDetectorNode(Node):
    """Runs YOLO on every BGR frame from the ToF camera."""

    def __init__(self):
        super().__init__("yolo_tof_detector")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            "weights_path",
            os.path.join(
                os.path.dirname(__file__),   # same dir as this script
                "..", "weights", "yolo26n.pt"
            ),
        )
        self.declare_parameter("image_topic", "/gordon_tof/bgr")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("target_classes", ["person", "box", "chair"])
        self.declare_parameter("device", "cpu")   # "cpu" or "cuda:0"
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("publish_annotated_image", True)

        weights_path      = self.get_parameter("weights_path").value
        image_topic       = self.get_parameter("image_topic").value
        self.conf_thresh  = self.get_parameter("confidence_threshold").value
        self.iou_thresh   = self.get_parameter("iou_threshold").value
        self.target_names = set(self.get_parameter("target_classes").value)
        self.device       = self.get_parameter("device").value
        self.imgsz        = self.get_parameter("imgsz").value
        self.pub_img      = self.get_parameter("publish_annotated_image").value

        # ── Load model ────────────────────────────────────────────────────────
        weights_path = os.path.abspath(weights_path)
        if not os.path.isfile(weights_path):
            self.get_logger().fatal(
                f"Weights file NOT found: {weights_path}\n"
                "Check the 'weights_path' parameter."
            )
            raise FileNotFoundError(weights_path)

        self.get_logger().info(f"Loading YOLO model from: {weights_path}")
        self.model = YOLO(weights_path)
        self.get_logger().info(
            f"Model loaded. Classes: {self.model.names}\n"
            f"Target filter: {sorted(self.target_names)}"
        )

        # Pre-compute the set of class IDs to keep (much faster than name lookup
        # per detection at runtime).
        self.target_ids: set[int] = {
            idx
            for idx, name in self.model.names.items()
            if name.lower() in {c.lower() for c in self.target_names}
        }

        if not self.target_ids:
            self.get_logger().warn(
                f"None of the target classes {sorted(self.target_names)} "
                f"were found in the model's class list: {self.model.names}. "
                "All detections will be published."
            )

        # ── Bridge ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ────────────────────────────────────────────────────────
        self.results_pub = self.create_publisher(
            String, "/yolo_detections/results", 10
        )

        if self.pub_img:
            self.image_pub = self.create_publisher(
                Image, "/yolo_detections/image", 10
            )
        else:
            self.image_pub = None

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # ── Stats & Throttling ────────────────────────────────────────────────
        self._frame_count = 0
        self._last_process_time = 0.0
        self.get_logger().info(
            f"YoloTofDetectorNode ready.\n"
            f"  Image topic      : {image_topic}\n"
            f"  Confidence thresh: {self.conf_thresh}\n"
            f"  IoU thresh       : {self.iou_thresh}\n"
            f"  Target classes   : {sorted(self.target_names)}\n"
            f"  Target class IDs : {sorted(self.target_ids)}\n"
            f"  Device           : {self.device}\n"
            f"  Image size       : {self.imgsz}"
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        """Convert incoming image, run YOLO, publish results."""
        # ── Skip frames if processing too fast (Throttle to ~5 FPS max) ───────
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_process_time) < 0.2:
            return
        
        self._last_process_time = now

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        self._frame_count += 1

        # ── Inference ─────────────────────────────────────────────────────────
        try:
            results = self.model.predict(
                frame_bgr,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        # ── Parse detections ──────────────────────────────────────────────────
        detections = []
        annotated = frame_bgr.copy() if self.pub_img else None

        result = results[0]  # single image → single Results object

        if result.boxes is not None:
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()   # (N, 4)
            confs      = result.boxes.conf.cpu().numpy()   # (N,)
            cls_ids    = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

            for (x1, y1, x2, y2), conf, cls_id in zip(boxes_xyxy, confs, cls_ids):

                # ── Class filter ──────────────────────────────────────────────
                if self.target_ids and cls_id not in self.target_ids:
                    continue

                class_name = self.model.names.get(cls_id, str(cls_id))
                det = {
                    "class_name": class_name,
                    "class_id":   int(cls_id),
                    "confidence": round(float(conf), 4),
                    "bbox":       [int(x1), int(y1), int(x2), int(y2)],
                }
                detections.append(det)

                # ── Draw bounding box ─────────────────────────────────────────
                if annotated is not None:
                    color = CLASS_COLORS.get(class_name.lower(), DEFAULT_COLOR)
                    self._draw_detection(
                        annotated, det, color
                    )

        # ── Publish JSON results ───────────────────────────────────────────────
        results_msg = String()
        results_msg.data = json.dumps(detections)
        self.results_pub.publish(results_msg)

        # ── Publish annotated image ────────────────────────────────────────────
        if self.image_pub is not None and annotated is not None:
            try:
                ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                ann_msg.header = msg.header   # keep original timestamp & frame_id
                self.image_pub.publish(ann_msg)
            except Exception as exc:
                self.get_logger().error(f"Failed to publish annotated image: {exc}")

        # Periodic log
        if len(detections) > 0 or self._frame_count % 50 == 0:
            self.get_logger().info(
                f"[Frame {self._frame_count:05d}] "
                f"{len(detections)} detection(s): "
                + ", ".join(
                    f"{d['class_name']} ({d['confidence']:.2f})"
                    for d in detections
                )
            )

    # ── Drawing helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _draw_detection(image: np.ndarray, det: dict, color: tuple):
        """Draw a bounding box + label on *image* in-place."""
        x1, y1, x2, y2 = det["bbox"]
        label = f"{det['class_name']} {det['confidence']:.2f}"

        # Box
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        # Label background
        font          = cv2.FONT_HERSHEY_SIMPLEX
        font_scale    = 0.55
        thickness     = 1
        (tw, th), bl  = cv2.getTextSize(label, font, font_scale, thickness)
        ly1           = max(y1 - th - bl - 4, 0)
        cv2.rectangle(image, (x1, ly1), (x1 + tw + 4, y1), color, cv2.FILLED)

        # Label text
        text_color = (0, 0, 0) if sum(color) > 400 else (255, 255, 255)
        cv2.putText(
            image, label,
            (x1 + 2, y1 - bl - 2),
            font, font_scale, text_color, thickness, cv2.LINE_AA,
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = YoloTofDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
