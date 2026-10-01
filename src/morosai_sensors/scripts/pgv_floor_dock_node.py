#!/usr/bin/env python3

from enum import Enum

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from optical_head.msg import PgvScanData
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DockState(Enum):
    # Stati principali della macchina di docking:
    # IDLE = attesa di un tag valido, ALIGN = heading e centraggio su y,
    # APPROACH = avanzamento nella direzione positiva di x.
    IDLE = 0
    ALIGN = 1
    APPROACH = 2
    DOCKED = 3


class PgvFloorDockNode(Node):
    """Interrompe Nav2 e muove il robot usando i dati del PGV sul pavimento."""

    def __init__(self):
        super().__init__('pgv_floor_dock_node')

        # Topic del sensore PGV e topic su cui inviare i comandi di velocita.
        # /cmd_vel_raw e' l'ingresso del collision monitor nel setup del robot.
        self.declare_parameter('pgv_topic', '/pgv100_scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')

        # Servizio standard di Nav2 per cancellare gli obiettivi attivi.
        self.declare_parameter('cancel_service', '/navigate_to_pose/_action/cancel_goal')
        # Comando scheduler: true abilita/riarma, false disabilita il docking.
        self.declare_parameter('dock_command_topic', '/agv_dock')

        # Il docking parte quando x e' nell'intervallo [200, 800] mm.
        self.declare_parameter('x_start_mm', 4800.0)
        self.declare_parameter('x_stop_mm', 2300.0)
        # Direzione docking: True per retromarcia (linear.x < 0), False per marcia avanti.
        self.declare_parameter('reverse_docking', True)

        # Tolleranza di centraggio laterale e limiti di velocita.
        self.declare_parameter('y_tolerance_mm', 15.0)
        self.declare_parameter('max_forward_speed', 0.15)
        self.declare_parameter('max_lateral_speed', 0.04)
        self.declare_parameter('angle_tolerance_deg', 1.0)
        self.declare_parameter('angle_kp', 0.01)
        self.declare_parameter('max_angular_speed', 0.10)
        self.declare_parameter('angle_sign', 1.0)

        # Guadagno proporzionale per correggere y:
        # velocita_y = lateral_sign * lateral_kp * y_pos.
        self.declare_parameter('lateral_kp', 0.002)
        self.declare_parameter('lateral_sign', -1.0)

        # Sicurezza: se il sensore non aggiorna il dato entro questo tempo,
        # il robot viene fermato perche' la posizione non e' piu' affidabile.
        self.declare_parameter('scan_timeout', 0.30)
        self.declare_parameter('control_rate', 20.0)

        pgv_topic = self.get_parameter('pgv_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        cancel_service = self.get_parameter('cancel_service').value
        dock_command_topic = self.get_parameter('dock_command_topic').value
        self.x_start_mm = float(self.get_parameter('x_start_mm').value)
        self.x_stop_mm = float(self.get_parameter('x_stop_mm').value)
        self.reverse_docking = bool(self.get_parameter('reverse_docking').value)
        self.y_tolerance_mm = float(self.get_parameter('y_tolerance_mm').value)
        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.angle_tolerance_deg = float(
            self.get_parameter('angle_tolerance_deg').value
        )
        self.angle_kp = float(self.get_parameter('angle_kp').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.angle_sign = float(self.get_parameter('angle_sign').value)
        self.lateral_kp = float(self.get_parameter('lateral_kp').value)
        self.lateral_sign = float(self.get_parameter('lateral_sign').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)

        # Evita una configurazione senza intervallo di avanzamento valido.
        # if self.x_stop_mm <= self.x_start_mm:
        #     raise ValueError('x_stop_mm must be greater than x_start_mm')

        if self.x_start_mm <= self.x_stop_mm:
            raise ValueError('x_stop_mm must be SMALLER than x_start_mm')

        # Il docking deve essere esplicitamente abilitato dallo scheduler.
        # Cosi' il semplice passaggio sopra un tag non avvia il robot.
        self.dock_enabled = False
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.cancel_requested = False

        # Publisher dei comandi e dello stato diagnostico del docking.
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '/pgv_floor_dock/status', 10)

        # Il comando arriva dallo scheduler/MQTT come flag Bool:
        # true = nuova missione di docking, false = stop/disabilitazione.
        self.dock_command_sub = self.create_subscription(
            Bool, dock_command_topic, self.dock_command_callback, 10
        )

        # Ogni messaggio PGV aggiorna l'ultima posizione conosciuta del tag.
        self.scan_sub = self.create_subscription(
            PgvScanData, pgv_topic, self.scan_callback, 10
        )

        # Client del servizio che cancella gli obiettivi Nav2 attivi.
        self.cancel_client = self.create_client(CancelGoal, cancel_service)

        # Il controllo gira periodicamente e pubblica il comando corretto
        # in base allo stato corrente e all'ultimo messaggio PGV ricevuto.
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('control_rate').value),
            self.control_loop,
        )

        self.get_logger().info(
            f'PGV floor docking active for x={self.x_start_mm:.0f}..'
            f'{self.x_stop_mm:.0f} mm on {pgv_topic}'
        )

    def scan_callback(self, msg: PgvScanData):
        # Salva sempre l'ultimo dato e l'istante in cui e' stato ricevuto.
        # Il control_loop usera' questi valori per verificare che il dato sia
        # recente prima di muovere il robot.
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

        # Una volta iniziato il docking, il callback si limita ad aggiornare
        # la misura: la macchina a stati viene gestita dal control_loop.
        if not self.dock_enabled or self.state != DockState.IDLE:
            return

        # Un tag e' valido solo se rilevato, nell'intervallo x richiesto e
        # senza il bit di errore segnalato dal sensore PGV.
        if not self.is_start_tag(msg):
            return

        # Primo passaggio: ferma Nav2 e corregge contemporaneamente heading e y.
        self.state = DockState.ALIGN
        self.cancel_requested = False
        self.publish_status('TAG_DETECTED_CANCELING_NAV')
        self.request_nav_cancel()
        self.get_logger().info(
            f'PGV tag detected at x={msg.x_pos:.1f} mm, y={msg.y_pos:.1f} mm; '
            'alignment to 0/180 deg and y=0'
        )

    def dock_command_callback(self, msg: Bool):
        """Abilita o disabilita il docking tramite il flag dello scheduler."""
        if not msg.data:
            # false/0: stop immediato e nessuna reazione ai tag PGV.
            self.dock_enabled = False
            self.stop_robot()
            self.state = DockState.IDLE
            self.latest_scan = None
            self.latest_scan_time = None
            self.cancel_requested = False
            self.publish_status('DISABLED')
            return

        if self.dock_enabled and self.state != DockState.DOCKED:
            return

        # true/1: abilita una nuova missione. Il reset dei dati precedenti
        # impedisce che una vecchia scansione faccia partire il docking.
        self.dock_enabled = True
        self.state = DockState.IDLE
        self.latest_scan = None
        self.latest_scan_time = None
        self.cancel_requested = False
        self.publish_status('ARMED')
        self.get_logger().info('PGV docking armed; waiting for a new tag')

    def is_start_tag(self, msg: PgvScanData):
        # Questa condizione viene usata solo per avviare un nuovo docking:
        # la posizione deve essere valida (no_pos == 0) e nell'intervallo iniziale x=200..800 mm.
        return (
            msg.no_pos == 0
            # and self.x_start_mm <= msg.x_pos <= self.x_stop_mm
            and self.x_stop_mm <= msg.x_pos <= self.x_start_mm
            and msg.error is False
        )

    def is_valid_tag(self, msg: PgvScanData):
        # Anche durante il docking x non deve mai superare x_stop: oltre questo
        # limite il robot sarebbe gia' oltre la posizione massima consentita.
        # Il control_loop ferma il robot prima di poter pubblicare altri comandi.
        return (
            msg.no_pos == 0
            # and msg.x_pos >= self.x_start_mm
            # and msg.x_pos <= self.x_stop_mm
            and msg.x_pos >= self.x_stop_mm
            and msg.x_pos <= self.x_start_mm
            and msg.error is False
        )

    def request_nav_cancel(self):
        # Evita di inviare piu' richieste per lo stesso evento di rilevamento.
        if self.cancel_requested:
            return
        self.cancel_requested = True

        # Se Nav2 non e' ancora pronto, il docking continua comunque.
        # Il comando successivo su /cmd_vel_raw verra' comunque prodotto da
        # questo nodo e il collision monitor gestira' l'uscita verso i motori.
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn(
                'Nav2 cancel service is not ready; continuing PGV docking'
            )
            return

        request = CancelGoal.Request()
        # Una richiesta vuota (goal_info tutto a zero) chiede a Nav2 di
        # cancellare tutti gli obiettivi attivi, non uno specifico goal.
        self.cancel_client.call_async(request).add_done_callback(
            self.cancel_done_callback
        )

    def cancel_done_callback(self, future):
        # Callback eseguito quando Nav2 risponde alla richiesta di cancellazione.
        try:
            response = future.result()
            self.get_logger().info(
                f'Nav2 cancel response: return_code={response.return_code}'
            )
        except Exception as exc:
            self.get_logger().error(f'Nav2 cancel request failed: {exc}')

    def control_loop(self):
        # In IDLE non viene pubblicato alcun comando di movimento.
        if not self.dock_enabled or self.state in (DockState.IDLE, DockState.DOCKED):
            return

        # Se manca una misura, per sicurezza il robot resta fermo.
        if self.latest_scan is None or self.latest_scan_time is None:
            self.stop_robot()
            return

        age = (
            self.get_clock().now() - self.latest_scan_time
        ).nanoseconds / 1e9
        # Una misura vecchia o non piu' valida interrompe il docking.
        if age > self.scan_timeout or not self.is_valid_tag(self.latest_scan):
            self.stop_robot()
            self.publish_status('FAILED_TAG_LOST')
            self.get_logger().warn('PGV tag lost or scan invalid; docking stopped')
            # Dopo una perdita del tag o un superamento di x_stop il nodo si
            # disarma: un nuovo comando true deve autorizzare un altro tentativo.
            self.dock_enabled = False
            self.state = DockState.IDLE
            return

        scan = self.latest_scan
        if self.state == DockState.ALIGN:
            # Mantiene il tag sotto osservazione correggendo heading e y insieme.
            if (
                abs(scan.y_pos) <= self.y_tolerance_mm
                and self.is_heading_aligned(scan.angle)
            ):
                self.state = DockState.APPROACH
                self.publish_status('APPROACHING')
                self.get_logger().info('PGV heading and lateral alignment complete')
            else:
                self.publish_alignment(scan.angle, scan.y_pos)
                return

        if self.state == DockState.APPROACH:
            # Quando x raggiunge 800 mm il docking e' completato.
            # if scan.x_pos >= self.x_stop_mm:
            if scan.x_pos <= self.x_stop_mm:
                self.stop_robot()
                self.publish_status('SUCCESS')
                self.get_logger().info(
                    f'PGV docking complete at x={scan.x_pos:.1f} mm'
                )
                # Disabilita automaticamente il nodo: il tag puo' rimanere
                # sotto il sensore senza riavviare il docking.
                self.dock_enabled = False
                self.state = DockState.DOCKED
                return

            self.publish_approach(scan.x_pos)

    def angle_error(self, angle_deg):
        # PGV increases clockwise. Select the nearer of the two valid headings.
        angle_to_zero = (0.0 - angle_deg + 180.0) % 360.0 - 180.0
        angle_to_180 = (180.0 - angle_deg + 180.0) % 360.0 - 180.0
        return angle_to_zero if abs(angle_to_zero) <= abs(angle_to_180) else angle_to_180

    def is_heading_aligned(self, angle_deg):
        return abs(self.angle_error(angle_deg)) <= self.angle_tolerance_deg

    def publish_alignment(self, angle_deg, y_pos_mm):
        twist = Twist()
        angular_speed = self.angle_sign * self.angle_kp * self.angle_error(angle_deg)
        twist.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_speed),
        )
        lateral_speed = self.lateral_sign * self.lateral_kp * y_pos_mm
        twist.linear.y = max(
            -self.max_lateral_speed,
            min(self.max_lateral_speed, lateral_speed),
        )
        self.cmd_pub.publish(twist)

    def publish_approach(self, x_pos_mm):
        # Calcola quanto e' avanzato il robot nell'intervallo x.
        # progress=0 a x_start e progress=1 a x_stop.
        # progress = (x_pos_mm - self.x_start_mm) / (
        #     self.x_stop_mm - self.x_start_mm
        # )
        progress = (self.x_start_mm - x_pos_mm) / (
            self.x_start_mm - self.x_stop_mm
        )

        # La velocita' diminuisce linearmente con l'aumentare di x:
        # velocita' massima all'inizio e quasi zero vicino al punto finale.
        forward_speed = self.max_forward_speed * (1.0 - progress)
        speed = max(0.0, min(self.max_forward_speed, forward_speed))
        twist = Twist()
        # Se reverse_docking e' True, il robot indietreggia in retromarcia (linear.x negativo)
        twist.linear.x = -speed if self.reverse_docking else speed
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        # Twist vuoto = tutte le velocita' lineari e angolari a zero.
        self.cmd_pub.publish(Twist())

    def publish_status(self, status):
        # Pubblica lo stato per log, supervisori o strumenti di diagnostica.
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Arresto di sicurezza anche durante lo spegnimento del nodo.
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    # Inizializza ROS 2, crea il nodo e lascia l'esecutore gestire callback
    # del sensore, timer e risposta del servizio Nav2.
    rclpy.init(args=args)
    node = PgvFloorDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# ros2 topic pub --once /agv_dock std_msgs/msg/Bool "{data: true}"