#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Dual 2D LiDAR — Small-Offset Calibration + Fused Scan (ROS 2)
#
# - Usa TF dell'URDF come stima iniziale (già quasi corretta).
# - Stima una piccola correzione SE2 che allinea REAR a FRONT nel frame base_link
#   usando un accoppiamento per angoli (binning polare) e Procrustes 2D robusto.
# - Non sovrascrive i TF dell'URDF: applica la correzione ai punti del REAR
#   e pubblica un nuovo frame opzionale "lidar_rear_link_calib".
# - Pubblica sempre un LaserScan fuso e calibrato su /fused_calib_scan (frame: base_link).

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

# ============================== PARAMS ==============================
FRONT_SCAN_TOPIC = '/lidar_front/scan'
REAR_SCAN_TOPIC  = '/lidar_rear/scan'

BASE_FRAME  = 'base_link'
FRONT_FRAME = 'lidar_front_link'
REAR_FRAME  = 'lidar_rear_link'
REAR_CALIB_FRAME = 'lidar_rear_link_calib'

# Raccolta per calibrazione (scene quasi statica)
SAMPLES_TO_COLLECT = 40
MIN_RANGE = 0.15
MAX_RANGE = 20.0
STRIDE_CALIB = 2         # sottocampionamento raggi per la calibrazione
BIN_RES_DEG  = 0.5       # risoluzione angolare per pairing e fusione
OUTLIER_QUANT = 0.85     # scarta la coda peggiore dei residui
# ===================================================================


def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def tf_to_se2(tf: TransformStamped):
    tx = float(tf.transform.translation.x)
    ty = float(tf.transform.translation.y)
    q = tf.transform.rotation
    th = yaw_from_quat(q.x, q.y, q.z, q.w)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, tx],
                     [s,  c, ty],
                     [0,  0,  1 ]], dtype=np.float64)


def se2(dx, dy, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, dx],
                     [s,  c, dy],
                     [0,  0,  1 ]], dtype=np.float64)


def scan_to_xy(scan: LaserScan, max_range: float, min_range: float, stride: int):
    n = len(scan.ranges)
    if n == 0:
        return np.empty((0, 2), dtype=np.float64)
    angles = np.linspace(scan.angle_min, scan.angle_max, n, dtype=np.float64)
    r = np.asarray(scan.ranges, dtype=np.float64)
    mask = np.isfinite(r) & (r > min_range) & (r < max_range)
    angles, r = angles[mask][::stride], r[mask][::stride]
    if r.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    x = r * np.cos(angles)
    y = r * np.sin(angles)
    return np.stack([x, y], axis=1)


def apply_T(T: np.ndarray, P: np.ndarray):
    if P.shape[0] == 0:
        return P
    R = T[:2, :2]
    t = T[:2, 2]
    return (P @ R.T) + t


def polar_bin_pairs(Pa: np.ndarray, Pb: np.ndarray, bin_res_rad: float,
                    rmin: float, rmax: float):
    # binning per angolo in base_link; usa r minimo nel bin
    def to_bins(P):
        if P.shape[0] == 0:
            return {}
        th = np.arctan2(P[:, 1], P[:, 0])
        rr = np.hypot(P[:, 0], P[:, 1])
        mask = (rr > rmin) & (rr < rmax)
        th = th[mask]
        rr = rr[mask]
        idx = np.floor((th + math.pi) / bin_res_rad).astype(np.int64)
        bins = {}
        for i, r in zip(idx, rr):
            if i in bins:
                if r < bins[i]:
                    bins[i] = r
            else:
                bins[i] = r
        return bins

    A = to_bins(Pa)
    B = to_bins(Pb)
    common = sorted(set(A.keys()) & set(B.keys()))
    if not common:
        return np.empty((0, 2)), np.empty((0, 2))
    ths = (np.asarray(common, dtype=np.float64) * bin_res_rad) - math.pi + (0.5 * bin_res_rad)
    ra = np.asarray([A[i] for i in common], dtype=np.float64)
    rb = np.asarray([B[i] for i in common], dtype=np.float64)
    Pa_pairs = np.stack([ra * np.cos(ths), ra * np.sin(ths)], axis=1)
    Pb_pairs = np.stack([rb * np.cos(ths), rb * np.sin(ths)], axis=1)
    return Pa_pairs, Pb_pairs


def procrustes_2d(src: np.ndarray, dst: np.ndarray):
    # ritorna dx, dy, dtheta che mappano src -> dst
    if src.shape[0] < 10 or dst.shape[0] < 10:
        return np.zeros(3), np.inf
    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    X = src - cs
    Y = dst - cd
    H = X.T @ Y
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = cd - (R @ cs)
    th = math.atan2(R[1, 0], R[0, 0])
    e = Y - (X @ R.T)
    rmse = math.sqrt(np.mean(np.sum(e * e, axis=1)))
    return np.array([t[0], t[1], th]), rmse


class DualLidarCalibrator(Node):
    def __init__(self):
        super().__init__('dual_lidar_calibrator')

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Parametri runtime (override)
        self.declare_parameter('bin_res_deg', BIN_RES_DEG)
        self.declare_parameter('samples', SAMPLES_TO_COLLECT)
        self.declare_parameter('manual_dx', 0.0)
        self.declare_parameter('manual_dy', 0.0)
        self.declare_parameter('manual_dyaw_deg', 0.0)

        # Publisher fused
        self.pub_fused = self.create_publisher(LaserScan, '/fused_calib_scan', 10)

        # Subscribers sync
        sub_f = Subscriber(self, LaserScan, FRONT_SCAN_TOPIC)
        sub_r = Subscriber(self, LaserScan, REAR_SCAN_TOPIC)
        self.sync = ApproximateTimeSynchronizer([sub_f, sub_r], queue_size=40, slop=0.15)
        self.sync.registerCallback(self.cb_scans)

        # Stato calibrazione
        self.samples = []  # lista di delta [dx,dy,th]
        self.calibrated = False
        self.Tcorr = np.eye(3)  # correzione in base_link per REAR

        self.get_logger().info("Piccola calibrazione REAR→FRONT. Ruota lentamente in scena statica…")

    def cb_scans(self, scan_f: LaserScan, scan_r: LaserScan):
        # Trasforma punti nel frame base usando TF correnti (URDF)
        Pf = scan_to_xy(scan_f, MAX_RANGE, MIN_RANGE, STRIDE_CALIB)
        Pr = scan_to_xy(scan_r, MAX_RANGE, MIN_RANGE, STRIDE_CALIB)

        # usa i frame dichiarati nei messaggi se disponibili
        front_frame = scan_f.header.frame_id if scan_f.header.frame_id else FRONT_FRAME
        rear_frame  = scan_r.header.frame_id if scan_r.header.frame_id else REAR_FRAME

        # timing accurato: TF al timestamp dei messaggi
        stamp_f = Time.from_msg(scan_f.header.stamp)
        stamp_r = Time.from_msg(scan_r.header.stamp)
        try:
            tf_fb = self.tf_buffer.lookup_transform(BASE_FRAME, front_frame, stamp_f, timeout=Duration(seconds=0.2))
            tf_rb = self.tf_buffer.lookup_transform(BASE_FRAME, rear_frame,  stamp_r, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"TF non disponibile: {e}")
            return
        Tfb = tf_to_se2(tf_fb)
        Trb = tf_to_se2(tf_rb)
        Pf_b = apply_T(Tfb, Pf)
        Pr_b = apply_T(Trb, Pr)

        # Durante calibrazione: stima delta piccolo e accumula
        if not self.calibrated:
            # aggiorna parametri se variati a runtime
            bin_res_deg = float(self.get_parameter('bin_res_deg').get_parameter_value().double_value)
            target_samples = int(self.get_parameter('samples').get_parameter_value().integer_value or SAMPLES_TO_COLLECT)
            bin_res = math.radians(bin_res_deg)
            A, B = polar_bin_pairs(Pr_b, Pf_b, bin_res, MIN_RANGE, MAX_RANGE)
            if A.shape[0] >= 30:
                delta, rmse = procrustes_2d(A, B)
                if np.isfinite(rmse):
                    # trimming outlier e ristima
                    R = se2(delta[0], delta[1], delta[2])
                    A2 = apply_T(R, A)
                    res = np.linalg.norm(B - A2, axis=1)
                    q = np.quantile(res, OUTLIER_QUANT)
                    keep = res <= q
                    if keep.sum() >= 20:
                        delta, rmse = procrustes_2d(A[keep], B[keep])
                        self.samples.append(delta)
                        self.get_logger().info(f"Samples {len(self.samples)}/{target_samples} (rmse~{rmse:.03f})")
            if len(self.samples) >= target_samples:
                self.finish_calibration(Trb)

        # Pubblica sempre fused (applica correzione a REAR se disponibile)
        self.publish_fused(Pf_b, Pr_b, scan_f, scan_r)

    def finish_calibration(self, Trb_current: np.ndarray):
        D = np.vstack(self.samples)
        dx = float(np.median(D[:, 0]))
        dy = float(np.median(D[:, 1]))
        s = float(np.median(np.sin(D[:, 2])))
        c = float(np.median(np.cos(D[:, 2])))
        th = math.atan2(s, c)
        # applica offset manuale se fornito
        mdx = float(self.get_parameter('manual_dx').get_parameter_value().double_value)
        mdy = float(self.get_parameter('manual_dy').get_parameter_value().double_value)
        mdyaw = math.radians(float(self.get_parameter('manual_dyaw_deg').get_parameter_value().double_value))
        self.Tcorr = se2(mdx, mdy, mdyaw) @ se2(dx, dy, th)

        # Pubblica frame opzionale calibrato per rear
        Trb_calib = self.Tcorr @ Trb_current
        xr, yr = Trb_calib[0, 2], Trb_calib[1, 2]
        thr = math.atan2(Trb_calib[1, 0], Trb_calib[0, 0])

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = BASE_FRAME
        tf.child_frame_id = REAR_CALIB_FRAME
        tf.transform.translation.x = float(xr)
        tf.transform.translation.y = float(yr)
        tf.transform.translation.z = 0.0
        cz, sz = math.cos(thr / 2.0), math.sin(thr / 2.0)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = sz
        tf.transform.rotation.w = cz
        self.static_broadcaster.sendTransform(tf)

        self.calibrated = True
        self.get_logger().info("=== Calibrazione completata (correzione REAR nel frame base_link) ===")
        self.get_logger().info(f"delta: dx={dx:.3f} m, dy={dy:.3f} m, yaw={th:.6f} rad")
        self.get_logger().info(f"Nuovo frame pubblicato: {BASE_FRAME} -> {REAR_CALIB_FRAME}")

    def publish_fused(self, Pf_b: np.ndarray, Pr_b: np.ndarray,
                      scan_f: LaserScan, scan_r: LaserScan):
        # applica correzione al rear
        Pr_b_cal = apply_T(self.Tcorr, Pr_b)

        # setup output scan
        angle_min = -math.pi
        angle_max = math.pi
        # consenti modificare risoluzione a runtime
        inc = max(math.radians(float(self.get_parameter('bin_res_deg').get_parameter_value().double_value or BIN_RES_DEG)), 1e-4)
        n_bins = int(math.ceil((angle_max - angle_min) / inc))
        ranges = [float('inf')] * n_bins

        rmin = float(min(getattr(scan_f, 'range_min', MIN_RANGE), getattr(scan_r, 'range_min', MIN_RANGE), MIN_RANGE))
        rmax = float(max(getattr(scan_f, 'range_max', MAX_RANGE), getattr(scan_r, 'range_max', MAX_RANGE), MAX_RANGE))

        def bin_points(P):
            if P.shape[0] == 0:
                return
            th = np.arctan2(P[:, 1], P[:, 0])
            rr = np.hypot(P[:, 0], P[:, 1])
            mask = (rr > rmin) & (rr < rmax)
            th = th[mask]
            rr = rr[mask]
            idx = np.floor((th - angle_min) / inc).astype(np.int64)
            for i, r in zip(idx, rr):
                if 0 <= i < n_bins and r < ranges[i]:
                    ranges[i] = float(r)

        bin_points(Pf_b)
        bin_points(Pr_b_cal)

        out = LaserScan()
        out.header.stamp = scan_f.header.stamp
        out.header.frame_id = BASE_FRAME
        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = inc
        out.time_increment = 0.0
        out.scan_time = max(getattr(scan_f, 'scan_time', 0.0), getattr(scan_r, 'scan_time', 0.0))
        out.range_min = rmin
        out.range_max = rmax
        out.ranges = [r if math.isfinite(r) else float('nan') for r in ranges]
        out.intensities = []
        self.pub_fused.publish(out)


def main():
    rclpy.init()
    node = DualLidarCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
