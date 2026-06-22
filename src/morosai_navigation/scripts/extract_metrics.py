#!/usr/bin/env python3
"""
Offline metrics extractor per rosbag2.
Uso: ros2 run morosai_navigation extract_metrics.py <path_to_bag_folder>
oppure: python3 src/morosai_navigation/scripts/extract_metrics.py <path_to_bag_folder>
"""

import sys
import math
import json
import csv
import numpy as np
from pathlib import Path
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py

# Standard ROS2 messages
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

# Nav2 specific
try:
    from nav2_msgs.msg import BehaviorTreeLog
except ImportError:
    print("[WARN] nav2_msgs non trovato. Segmentazione BT disabilitata.")
    BehaviorTreeLog = None

try:
    from dwb_msgs.msg import LocalPlanEvaluation
except ImportError:
    print("[WARN] dwb_msgs non trovato. Analisi DWB disabilitata.")
    LocalPlanEvaluation = None

# ─────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────

def get_reader(bag_path: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader

def path_length(poses) -> float:
    """Lunghezza di un nav_msgs/Path in metri"""
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i-1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total

def path_length_xy(pts) -> float:
    """Lunghezza da lista di (x,y)"""
    if len(pts) < 2:
        return 0.0
    return sum(math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
               for i in range(1, len(pts)))

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def smoothness(headings) -> float:
    """Variazione media dell'heading (rad) — bassa = path smooth"""
    if len(headings) < 2:
        return 0.0
    diffs = [abs(headings[i] - headings[i-1]) for i in range(1, len(headings))]
    return float(np.mean(diffs))

def avg_deviation_from_plan(traj_xy, plan_poses) -> float:
    """Deviazione media (m) della traiettoria reale dal piano globale"""
    if not traj_xy or not plan_poses:
        return 0.0
    plan_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in plan_poses])
    traj_pts = np.array(traj_xy)
    devs = [np.hypot(plan_pts[:,0]-px, plan_pts[:,1]-py).min()
            for px, py in traj_pts]
    return float(np.mean(devs))

# Deprecated: _agv_deviation was replaced by _agv_local_path

# ─────────────────────────────────────────────
#  Lettura del bag
# ─────────────────────────────────────────────

TYPE_MAP = {
    '/odom':               Odometry,
    '/plan':               NavPath,
    '/unsmoothed_plan':    NavPath,
    '/local_plan':         NavPath,
    '/received_global_plan': NavPath,
    '/merged':             LaserScan,
    '/lidar_front/scan':   LaserScan,
    '/lidar_rear/scan':    LaserScan,
    '/cmd_vel':            Twist,
    '/agv_v':              Float32,
    '/agv_local_path':     String,  # Placeholder for JSON-bridge path
}
if BehaviorTreeLog:
    TYPE_MAP['/behavior_tree_log'] = BehaviorTreeLog
if LocalPlanEvaluation:
    TYPE_MAP['/evaluation'] = LocalPlanEvaluation

def read_bag(bag_path: str) -> dict:
    """Legge il bag e restituisce dizionario topic→lista messaggi con timestamp"""
    try:
        reader = get_reader(bag_path)
    except Exception as e:
        print(f"Errore nell'apertura del bag: {e}")
        sys.exit(1)

    data = defaultdict(list)
    topics_to_read = set(TYPE_MAP.keys())
    
    while reader.has_next():
        topic, raw, ts_ns = reader.read_next()
        if topic not in topics_to_read:
            continue
        msg_type = TYPE_MAP[topic]
        msg = deserialize_message(raw, msg_type)
        data[topic].append((ts_ns, msg))
    
    print(f"Letti {sum(len(v) for v in data.values())} messaggi da {len(data)} topic")
    return data

# ─────────────────────────────────────────────
#  Segmentazione per navigazione (da BT log)
# ─────────────────────────────────────────────

def extract_nav_sessions(bt_msgs) -> list:
    """
    Estrae le sessioni di navigazione dal BehaviorTreeLog.
    Ogni sessione = {start_ns, end_ns, success}
    """
    sessions = []
    current = None

    for ts_ns, msg in bt_msgs:
        for event in msg.event_log:
            node = event.node_name
            status = event.current_status  # RUNNING, SUCCESS, FAILURE, IDLE

            if any(k in node for k in ['NavigateRecovery', 'NavigateToPose',
                                        'Navigate', 'FollowPath']):
                if status == 'RUNNING' and current is None:
                    current = {'start_ns': ts_ns, 'node': node}
                elif status in ('SUCCESS', 'FAILURE') and current is not None:
                    current['end_ns'] = ts_ns
                    current['success'] = (status == 'SUCCESS')
                    sessions.append(current)
                    current = None

    # Fallback: se non troviamo nulla dal BT, usa l'intera bag come sessione unica
    if not sessions:
        print("WARN: nessuna sessione trovata nel BT log → uso intera bag come sessione unica")
        return []
    return sessions

# ─────────────────────────────────────────────
#  Calcolo metriche per sessione
# ─────────────────────────────────────────────

def slice_by_time(msgs, t_start, t_end):
    return [(ts, m) for ts, m in msgs if t_start <= ts <= t_end]

def compute_session_metrics(session: dict, data: dict, session_id: int) -> dict:
    t0, t1 = session['start_ns'], session['end_ns']
    dt_s = (t1 - t0) * 1e-9

    # ── 1. Path length pianificato (ultimo /plan nella sessione)
    plans = slice_by_time(data.get('/plan', []), t0, t1)
    plan_length = path_length(plans[-1][1].poses) if plans else 0.0
    replanning_count = max(0, len(plans) - 1)

    unsmoothed = slice_by_time(data.get('/unsmoothed_plan', []), t0, t1)
    unsmoothed_length = path_length(unsmoothed[-1][1].poses) if unsmoothed else 0.0

    # ── 2. Traiettoria reale da /odom
    odom_msgs = slice_by_time(data.get('/odom', []), t0, t1)
    traj_xy = [(ts, m.pose.pose.position.x, m.pose.pose.position.y)
               for ts, m in odom_msgs]
    actual_length = path_length_xy([(x, y) for _, x, y in traj_xy])

    # Smoothness da heading reale
    headings = [yaw_from_quat(m.pose.pose.orientation) for _, m in odom_msgs]
    smooth = smoothness(headings)

    # Velocità lineare da /odom
    lin_vels = [math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
                for _, m in odom_msgs]
    avg_vel = float(np.mean(lin_vels)) if lin_vels else 0.0
    max_vel = float(np.max(lin_vels)) if lin_vels else 0.0

    # ── 3. Deviazione dal piano globale
    plan_poses = plans[-1][1].poses if plans else []
    dev_computed = avg_deviation_from_plan([(x,y) for _,x,y in traj_xy], plan_poses)

    # Deviation tracking from /agv_deviation is now deprecated in favor of /agv_local_path
    dev_agv_mean = None
    dev_agv_max  = None

    # ── 4. Clearance (distanza minima ostacoli) da /merged o /lidar_*
    scans = data.get('/merged', []) or data.get('/lidar_front/scan', [])
    scan_msgs = slice_by_time(scans, t0, t1)
    clearances = []
    for _, scan in scan_msgs:
        valid = [r for r in scan.ranges
                 if scan.range_min < r < scan.range_max and not math.isinf(r)]
        if valid:
            clearances.append(min(valid))
    min_clearance = float(np.min(clearances)) if clearances else None
    avg_clearance = float(np.mean(clearances)) if clearances else None

    # ── 5. DWB evaluation (costo local planner)
    eval_msgs = slice_by_time(data.get('/evaluation', []), t0, t1)
    dwb_scores = []
    dwb_valid_traj_counts = []
    for _, ev in eval_msgs:
        if hasattr(ev, 'twists') and ev.twists:
            valid = [t for t in ev.twists if not math.isinf(t.total)]
            if valid:
                dwb_scores.append(min(t.total for t in valid))
                dwb_valid_traj_counts.append(len(valid))
    dwb_best_score_mean = float(np.mean(dwb_scores)) if dwb_scores else None
    dwb_valid_traj_mean = float(np.mean(dwb_valid_traj_counts)) if dwb_valid_traj_counts else None

    # ── 6. Velocità aggregata da /agv_v
    agv_v_msgs = slice_by_time(data.get('/agv_v', []), t0, t1)
    agv_v_vals = [m.data for _, m in agv_v_msgs]
    agv_v_mean = float(np.mean(agv_v_vals)) if agv_v_vals else None
    agv_v_max  = float(np.max(agv_v_vals))  if agv_v_vals else None

    return {
        'session_id':            session_id,
        'bt_node':               session.get('node', ''),
        'success':               int(session.get('success', False)),
        'nav_time_s':            round(dt_s, 3),

        # Path quality
        'plan_length_m':         round(plan_length, 4),
        'unsmoothed_length_m':   round(unsmoothed_length, 4),
        'actual_length_m':       round(actual_length, 4),
        'length_ratio':          round(actual_length / plan_length, 4) if plan_length > 0 else None,
        'replanning_count':      replanning_count,

        # Smoothness
        'smoothness_rad':        round(smooth, 5),

        # Deviation
        'dev_computed_m':        round(dev_computed, 5),
        'dev_agv_mean_m':        round(dev_agv_mean, 5) if dev_agv_mean else None,
        'dev_agv_max_m':         round(dev_agv_max,  5) if dev_agv_max  else None,

        # Safety
        'min_clearance_m':       round(min_clearance, 4) if min_clearance else None,
        'avg_clearance_m':       round(avg_clearance, 4) if avg_clearance else None,

        # DWB
        'dwb_best_score_mean':   round(dwb_best_score_mean, 4) if dwb_best_score_mean else None,
        'dwb_valid_traj_mean':   round(dwb_valid_traj_mean, 2) if dwb_valid_traj_mean else None,

        # Velocity
        'avg_vel_ms':            round(avg_vel, 4),
        'max_vel_ms':            round(max_vel, 4),
        'agv_v_mean_ms':         round(agv_v_mean, 4) if agv_v_mean else None,
        'agv_v_max_ms':          round(agv_v_max,  4) if agv_v_max  else None,
    }

# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 extract_metrics.py <bag_folder>")
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f"Apro bag: {bag_path}")

    # Inizializza rclpy per caricare correttamente i tipi di messaggio
    rclpy.init()

    data = read_bag(bag_path)

    # Segmenta le sessioni di navigazione
    sessions = extract_nav_sessions(data.get('/behavior_tree_log', []))
    
    # Se non c'Ã¨ BT log, usa l'intera bag come sessione unica
    if not sessions:
        if data.get('/odom'):
            t0 = data['/odom'][0][0]
            t1 = data['/odom'][-1][0]
            sessions = [{'start_ns': t0, 'end_ns': t1, 'success': True, 'node': 'full_bag'}]
        else:
            print("Nessuna sessione trovata e nessun dato odom presente.")
            sys.exit(1)

    print(f"Trovate {len(sessions)} sessioni di navigazione")

    all_metrics = []
    for i, session in enumerate(sessions):
        m = compute_session_metrics(session, data, i+1)
        all_metrics.append(m)
        print(f"\nSessione {i+1} | t={m['nav_time_s']}s | "
              f"len_plan={m['plan_length_m']}m | len_actual={m['actual_length_m']}m | "
              f"clearance_min={m['min_clearance_m']}m | "
              f"dev={m['dev_computed_m']}m | {'âœ“' if m['success'] else 'âœ˜'}")

    if not all_metrics:
        print("Nessuna metrica calcolata.")
        sys.exit(0)

    # ── Output CSV
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = f'nav_metrics_{ts}.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=all_metrics[0].keys())
        writer.writeheader()
        writer.writerows(all_metrics)
    print(f"\nCSV salvato: {csv_path}")

    # ── Output JSON
    json_path = f'nav_metrics_{ts}.json'
    with open(json_path, 'w') as f:
        duration = 0
        if data.get('/odom'):
             duration = (data['/odom'][-1][0] - data['/odom'][0][0]) * 1e-9

        json.dump({
            'bag': bag_path,
            'duration_s': duration,
            'sessions': all_metrics,
            'summary': {
                'success_rate':       float(np.mean([m['success'] for m in all_metrics])),
                'avg_nav_time_s':     float(np.mean([m['nav_time_s'] for m in all_metrics])),
                'avg_plan_length_m':  float(np.mean([m['plan_length_m'] for m in all_metrics])),
                'avg_actual_length_m':float(np.mean([m['actual_length_m'] for m in all_metrics])),
                'avg_deviation_m':    float(np.mean([m['dev_computed_m'] for m in all_metrics])),
                'avg_min_clearance_m':float(np.mean([m['min_clearance_m'] for m in all_metrics
                                               if m['min_clearance_m'] is not None])),
                'avg_smoothness_rad': float(np.mean([m['smoothness_rad'] for m in all_metrics])),
                'total_replanning':   int(sum(m['replanning_count'] for m in all_metrics)),
            }
        }, f, indent=2)
    print(f"JSON salvato: {json_path}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
