#!/usr/bin/env python3
"""
ROS2 Bag Analysis Node - Pro Version
Fixed: Non analizza finché non ha ricevuto dati e attende la fine della bag.
"""

import math
import csv
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# Tentativo import DWB
try:
    from dwb_msgs.msg import LocalPlanEvaluation
    HAS_DWB = True
except ImportError:
    HAS_DWB = False

class BagAnalysisNode(Node):
    def __init__(self):
        super().__init__('bag_analysis_node')

        # --- Parametri ---
        self.declare_parameter('auto_shutdown_delay', 3.0) # Secondi di silenzio prima di chiudere
        self.auto_shutdown_delay = self.get_parameter('auto_shutdown_delay').value

        # --- Data Storage ---
        self.odom_trajectory = []
        self.global_plans = []
        self.local_plans = []
        self.cmd_vel_data = []
        self.evaluations = []
        self.start_time = None
        self._last_msg_wall_time = self.get_clock().now().nanoseconds / 1e9
        self._has_received_data = False
        self._analysis_triggered = False

        # --- QoS ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.create_subscription(Path, '/plan', self.plan_cb, qos)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, qos)
        if HAS_DWB:
            self.create_subscription(LocalPlanEvaluation, '/evaluation', self.evaluation_cb, qos)

        # Watchdog: controlla ogni secondo se la bag è finita
        self.timer = self.create_timer(1.0, self.watchdog_check)

        print("\n" + "="*60)
        print("  ANALISI AVVIATA: In attesa dei dati dalla Rosbag...")
        print("  L'analisi partirà AUTOMATICAMENTE quando la bag finisce.")
        print("  (Oppure premi Ctrl+C per forzare la generazione ora)")
        print("="*60 + "\n")

    def _update_timestamp(self):
        self._last_msg_wall_time = self.get_clock().now().nanoseconds / 1e9
        if not self._has_received_data:
            print("  [!] Ricezione dati iniziata...")
            self._has_received_data = True

    def odom_cb(self, msg):
        self._update_timestamp()
        self.odom_trajectory.append(msg.pose.pose.position)

    def plan_cb(self, msg):
        self._update_timestamp()
        self.global_plans.append(msg.poses)

    def local_plan_cb(self, msg):
        self._update_timestamp()
        self.local_plans.append(msg.poses)

    def cmd_vel_cb(self, msg):
        self._update_timestamp()
        self.cmd_vel_data.append(msg)

    def evaluation_cb(self, msg):
        self._update_timestamp()
        self.evaluations.append(msg)

    def watchdog_check(self):
        if not self._has_received_data:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        silence_duration = now - self._last_msg_wall_time

        if silence_duration > self.auto_shutdown_delay:
            print(f"\n[INFO] Fine bag rilevata ({silence_duration:.1f}s di silenzio).")
            self.run_final_analysis()

    def run_final_analysis(self):
        if self._analysis_triggered:
            return
        self._analysis_triggered = True
        
        print("\n" + "="*60)
        print("  ELABORAZIONE REPORT FINALE...")
        print("="*60)
        
        # Statistiche rapide
        print(f"  Punti Odom:    {len(self.odom_trajectory)}")
        print(f"  Messaggi Vel:  {len(self.cmd_vel_data)}")
        print(f"  Valutazioni DWB: {len(self.evaluations)}")
        
        if len(self.odom_trajectory) < 2:
            print("\n[ERRORE] Troppi pochi dati per generare grafici!")
        else:
            self.plot_results()
        
        print("\n[✔] Analisi completata. Chiudi la finestra del grafico per uscire.")
        plt.show()
        # Non usiamo sys.exit qui per permettere a plt.show() di bloccare
        # Il nodo verrà terminato alla chiusura del grafico o con Ctrl+C

    def plot_results(self):
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_facecolor('#1e1e2e')
        
        # Plot Traiettoria
        x = [p.x for p in self.odom_trajectory]
        y = [p.y for p in self.odom_trajectory]
        ax.plot(x, y, color='#89b4fa', label='Traiettoria Robot', linewidth=2)
        
        # Plot Global Plan (ultimo ricevuto)
        if self.global_plans:
            gx = [p.pose.position.x for p in self.global_plans[-1]]
            gy = [p.pose.position.y for p in self.global_plans[-1]]
            ax.plot(gx, gy, '--', color='#a6e3a1', label='Piano Globale', alpha=0.7)

        ax.set_title("Percorso Analizzato dalla Bag", color='white')
        ax.legend()
        ax.axis('equal')
        plt.grid(True, alpha=0.2)

def main(args=None):
    rclpy.init(args=args)
    node = BagAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Se premi Ctrl+C, forza l'analisi prima di uscire
        if not node._analysis_triggered:
            node.run_final_analysis()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()