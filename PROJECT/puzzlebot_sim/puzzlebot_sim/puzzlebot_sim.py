import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class PuzzlebotSim(Node):
    """
    Nodo ROS2 - Simulador cinematico del Puzzlebot (Mini Challenge 2, Part 2).

    Suscripciones:
      - /cmd_vel  (Twist)    -> velocidad lineal v y angular omega
      - /odom     (Odometry) -> pose integrada por el nodo localisation
                               (se usa para actualizar JointStates en RViz)

    Publicaciones:
      - /pose_sim     (PoseStamped) -> posicion integrada del robot
      - /wr           (Float32)     -> velocidad angular rueda derecha [rad/s]
      - /wl           (Float32)     -> velocidad angular rueda izquierda [rad/s]
      - /joint_states (JointState)  -> angulos acumulados ruedas (para RViz)

    Nota: El TF odom -> base_footprint lo publica el nodo 'localisation'.
          El TF map  -> odom          lo publica el nodo 'localisation'.
          El robot_state_publisher maneja los TFs internos del URDF.
    """

    UPDATE_RATE_HZ = 20

    # Parametros fisicos del Puzzlebot
    WHEEL_RADIUS = 0.05   # r [m]
    WHEEL_BASE   = 0.19   # l [m]

    def __init__(self):
        super().__init__('puzzlebot_kinematic_model')

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim',     10)
        self.wr_pub   = self.create_publisher(Float32,     'wr',           10)
        self.wl_pub   = self.create_publisher(Float32,     'wl',           10)
        self.js_pub   = self.create_publisher(JointState,  'joint_states', 10)

        # --- Subscribers ---
        self.create_subscription(Twist,    'cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Odometry, 'odom',    self._odom_cb,    10)

        # --- Velocidades recibidas por cmd_vel ---
        self.v     = 0.0
        self.omega = 0.0

        # --- Pose propia (integrada desde cmd_vel, para pose_sim) ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # --- Angulos acumulados de ruedas (para JointStates / RViz) ---
        self.wheel_r_angle = 0.0
        self.wheel_l_angle = 0.0

        # Paso de tiempo
        self.dt = 1.0 / self.UPDATE_RATE_HZ

        # Timer principal
        self.create_timer(self.dt, self._update_cb)

        self.get_logger().info(
            'PuzzlebotSim inicializado. '
            f'r={self.WHEEL_RADIUS} m  l={self.WHEEL_BASE} m'
        )

    # ------------------------------------------------------------------
    # SUBSCRIBERS
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """Recibe velocidades lineales y angulares del controlador."""
        self.v     = msg.linear.x
        self.omega = msg.angular.z

    def _odom_cb(self, msg: Odometry):
        """
        Recibe odometria del nodo localisation.
        Actualiza angulos de rueda acumulados para JointStates en RViz,
        usando las velocidades de twist del mensaje.
        """
        v_odom     = msg.twist.twist.linear.x
        omega_odom = msg.twist.twist.angular.z

        wr = (2.0 * v_odom + omega_odom * self.WHEEL_BASE) / (2.0 * self.WHEEL_RADIUS)
        wl = (2.0 * v_odom - omega_odom * self.WHEEL_BASE) / (2.0 * self.WHEEL_RADIUS)

        self.wheel_r_angle += wr * self.dt
        self.wheel_l_angle += wl * self.dt

    # ------------------------------------------------------------------
    # CALLBACK PRINCIPAL (20 Hz)
    # ------------------------------------------------------------------

    def _update_cb(self):
        """Integra cinematica, calcula velocidades de rueda y publica."""

        # 1. Integrar modelo cinematico (Euler explicito)
        self.x     += self.v * math.cos(self.theta) * self.dt
        self.y     += self.v * math.sin(self.theta) * self.dt
        self.theta += self.omega * self.dt

        # Normalizar theta en [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 2. Calcular velocidades angulares de ruedas
        wr = (2.0 * self.v + self.omega * self.WHEEL_BASE) / (2.0 * self.WHEEL_RADIUS)
        wl = (2.0 * self.v - self.omega * self.WHEEL_BASE) / (2.0 * self.WHEEL_RADIUS)

        # 3. Publicar
        now = self.get_clock().now().to_msg()
        self._publish_pose(now)
        self._publish_wheel_speeds(wr, wl)
        self._publish_joint_states(now)

    # ------------------------------------------------------------------
    # PUBLISHERS
    # ------------------------------------------------------------------

    def _publish_pose(self, stamp):
        """Publica la pose integrada en /pose_sim (PoseStamped)."""
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0

        # Cuaternion manual desde yaw (sin librerias externas)
        half = self.theta / 2.0
        msg.pose.orientation.w = math.cos(half)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(half)

        self.pose_pub.publish(msg)

    def _publish_wheel_speeds(self, wr: float, wl: float):
        """Publica velocidades angulares de las ruedas en /wr y /wl."""
        msg_r      = Float32()
        msg_r.data = float(wr)
        self.wr_pub.publish(msg_r)

        msg_l      = Float32()
        msg_l.data = float(wl)
        self.wl_pub.publish(msg_l)

    def _publish_joint_states(self, stamp):
        """Publica los angulos acumulados de las ruedas para RViz."""
        js              = JointState()
        js.header.stamp = stamp
        js.name         = ['wheel_r_joint', 'wheel_l_joint']
        js.position     = [self.wheel_r_angle, self.wheel_l_angle]
        js.velocity     = [0.0, 0.0]
        js.effort       = []
        self.js_pub.publish(js)


# ----------------------------------------------------------------------
# ENTRYPOINT
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()