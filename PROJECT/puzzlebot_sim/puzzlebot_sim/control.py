import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class Control(Node):
    """
    Nodo ROS2 - Controlador go-to-goal (Mini Challenge 2, Extra).

    Suscripciones:
      - /odom       (Odometry) -> pose actual del robot
      - /set_point  (Point)    -> waypoint actual enviado por set_point_generator

    Publicaciones:
      - /cmd_vel    (Twist)    -> comandos de velocidad al simulador
      - /next_point (Bool)     -> flag True cuando el robot llega al waypoint

    Parametros ROS2:
      - linear_vel  (float) : velocidad lineal [m/s]        (default: 0.15)
      - angular_vel (float) : velocidad angular max [rad/s] (default: 0.5)
      - goal_tol    (float) : tolerancia de llegada [m]     (default: 0.05)
      - angle_tol   (float) : tolerancia de angulo [rad]    (default: 0.05)

    Logica de control (go-to-goal por estado):
      Estado 0 - ROTATE: gira hasta apuntar al set_point
      Estado 1 - MOVE  : avanza hasta llegar al set_point
    """

    UPDATE_RATE_HZ = 20

    STATE_ROTATE = 0
    STATE_MOVE   = 1

    def __init__(self):
        super().__init__('control')

        # --- Parametros ---
        self.declare_parameter('linear_vel',  0.15)
        self.declare_parameter('angular_vel', 0.5)
        self.declare_parameter('goal_tol',    0.05)
        self.declare_parameter('angle_tol',   0.05)

        self.v_lin     = self.get_parameter('linear_vel').value
        self.v_ang     = self.get_parameter('angular_vel').value
        self.goal_tol  = self.get_parameter('goal_tol').value
        self.angle_tol = self.get_parameter('angle_tol').value

        # --- Pose actual del robot ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.odom_received = False

        # --- Setpoint actual (viene del generador) ---
        self.goal_x         = None
        self.goal_y         = None
        self.goal_received  = False

        # --- Estado del controlador ---
        self.state           = self.STATE_ROTATE
        self.goal_reached    = False   # True mientras espera nuevo setpoint
        self.flag_sent       = False   # evita mandar next_point multiple veces

        # --- Publishers ---
        self.cmd_pub        = self.create_publisher(Twist, 'cmd_vel',    10)
        self.next_point_pub = self.create_publisher(Bool,  'next_point', 10)

        # --- Subscribers ---
        self.create_subscription(Odometry, 'odom',      self._odom_cb,      10)
        self.create_subscription(Point,    'set_point', self._set_point_cb, 10)

        # --- Timer ---
        self.dt = 1.0 / self.UPDATE_RATE_HZ
        self.create_timer(self.dt, self._update_cb)

        self.get_logger().info(
            f'Control inicializado. '
            f'v={self.v_lin} m/s  w={self.v_ang} rad/s  '
            f'goal_tol={self.goal_tol} m  angle_tol={self.angle_tol} rad'
        )

    # ------------------------------------------------------------------
    # SUBSCRIBERS
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        """Lee la pose actual del robot desde /odom."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        self.theta = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
        self.odom_received = True

    def _set_point_cb(self, msg: Point):
        """
        Recibe el waypoint actual del generador.
        Si es un punto nuevo, reinicia el estado del controlador.
        """
        new_x = msg.x
        new_y = msg.y

        # Detectar si es un waypoint diferente al actual
        if self.goal_x != new_x or self.goal_y != new_y:
            self.goal_x        = new_x
            self.goal_y        = new_y
            self.goal_received = True
            self.goal_reached  = False
            self.flag_sent     = False
            self.state         = self.STATE_ROTATE
            self.get_logger().info(f'Nuevo setpoint: ({new_x:.2f}, {new_y:.2f})')

    # ------------------------------------------------------------------
    # CALLBACK PRINCIPAL (20 Hz)
    # ------------------------------------------------------------------

    def _update_cb(self):
        """Maquina de estados go-to-goal."""

        if not self.odom_received or not self.goal_received:
            return

        # Si ya llego y mando el flag, esperar nuevo setpoint
        if self.goal_reached:
            self._publish_stop()
            return

        # --- Calcular errores ---
        dx            = self.goal_x - self.x
        dy            = self.goal_y - self.y
        dist          = math.sqrt(dx * dx + dy * dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error   = self._normalize_angle(angle_to_goal - self.theta)

        # --- Maquina de estados ---
        if self.state == self.STATE_ROTATE:
            self._state_rotate(angle_error)
        elif self.state == self.STATE_MOVE:
            self._state_move(dist, angle_error)

    # ------------------------------------------------------------------
    # ESTADOS
    # ------------------------------------------------------------------

    def _state_rotate(self, angle_error: float):
        """Gira el robot hasta apuntar al setpoint."""
        if abs(angle_error) > self.angle_tol:
            cmd = Twist()
            cmd.angular.z = self.v_ang if angle_error > 0.0 else -self.v_ang
            self.cmd_pub.publish(cmd)
        else:
            self._publish_stop()
            self.state = self.STATE_MOVE
            self.get_logger().info(
                f'Rotacion OK -> avanzando a ({self.goal_x:.2f}, {self.goal_y:.2f})'
            )

    def _state_move(self, dist: float, angle_error: float):
        """Avanza hacia el setpoint con correccion angular."""
        if dist > self.goal_tol:
            cmd = Twist()
            cmd.linear.x  = self.v_lin
            cmd.angular.z = 0.5 * angle_error
            self.cmd_pub.publish(cmd)
        else:
            # Waypoint alcanzado
            self._publish_stop()
            self.goal_reached = True
            self.get_logger().info(
                f'Llegada a ({self.goal_x:.2f}, {self.goal_y:.2f})  '
                f'pos=({self.x:.3f}, {self.y:.3f})'
            )
            # Publicar flag al generador
            if not self.flag_sent:
                flag = Bool()
                flag.data = True
                self.next_point_pub.publish(flag)
                self.flag_sent = True

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


# ----------------------------------------------------------------------
# ENTRYPOINT
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Control()
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