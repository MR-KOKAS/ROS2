import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


# ----------------------------------------------------------------------
# GENERADORES DE WAYPOINTS
# ----------------------------------------------------------------------

def generate_square(side: float):
    waypoints = [
        ( side,   0.0),
        ( side,  side),
        (  0.0,  side),
        (  0.0,   0.0),
    ]
    waypoints.append(waypoints[0])
    return waypoints


def generate_pentagon(radius: float):
    waypoints = []
    for i in range(1, 6):
        angle = math.pi / 2.0 + 2.0 * math.pi * i / 5.0
        waypoints.append((radius * math.cos(angle), radius * math.sin(angle)))
    waypoints.append(waypoints[0])
    return waypoints


def generate_circle(radius: float, num_points: int = 36):
    waypoints = []
    for i in range(1, num_points + 1):
        angle = 2.0 * math.pi * i / num_points
        waypoints.append((radius * math.cos(angle), radius * math.sin(angle)))
    return waypoints


# ----------------------------------------------------------------------
# NODO SET POINT GENERATOR
# ----------------------------------------------------------------------

class SetPointGenerator(Node):
    """
    Nodo ROS2 - Generador de setpoints (Mini Challenge 2, Extra).

    Publica el waypoint actual al controlador y espera su flag de llegada
    para avanzar al siguiente punto.

    Suscripciones:
      - /next_point (Bool) -> flag del controlador indicando que llego al goal

    Publicaciones:
      - /set_point (Point) -> waypoint actual para el controlador

    Parametros ROS2:
      - trajectory  (string) : 'square' | 'pentagon' | 'circle'  (default: 'square')
      - side        (float)  : lado del cuadrado [m]              (default: 1.0)
      - radius      (float)  : radio del pentagono/circulo [m]    (default: 1.0)
      - circle_pts  (int)    : puntos del circulo                 (default: 36)
    """

    def __init__(self):
        super().__init__('set_point_generator')

        # --- Parametros ---
        self.declare_parameter('trajectory', 'square')
        self.declare_parameter('side',        1.0)
        self.declare_parameter('radius',      1.0)
        self.declare_parameter('circle_pts',  36)

        trajectory = self.get_parameter('trajectory').value
        side       = self.get_parameter('side').value
        radius     = self.get_parameter('radius').value
        circle_pts = self.get_parameter('circle_pts').value

        # --- Generar waypoints ---
        if trajectory == 'pentagon':
            self.waypoints = generate_pentagon(radius)
            self.get_logger().info(f'Trayectoria: PENTAGONO  radio={radius} m  ({len(self.waypoints)} waypoints)')
        elif trajectory == 'circle':
            self.waypoints = generate_circle(radius, circle_pts)
            self.get_logger().info(f'Trayectoria: CIRCULO  radio={radius} m  puntos={circle_pts}')
        else:
            self.waypoints = generate_square(side)
            self.get_logger().info(f'Trayectoria: CUADRADO  lado={side} m  ({len(self.waypoints)} waypoints)')

        # --- Estado ---
        self.wp_index = 0
        self.done     = False

        # --- Publisher y Subscriber ---
        self.set_point_pub = self.create_publisher(Point, 'set_point', 10)
        self.create_subscription(Bool, 'next_point', self._next_point_cb, 10)

        # Publicar primer waypoint al arrancar (a 10 Hz hasta que el controlador lo reciba)
        self.create_timer(0.1, self._publish_current)

        self.get_logger().info('SetPointGenerator inicializado.')

    # ------------------------------------------------------------------
    # CALLBACK: next_point
    # ------------------------------------------------------------------

    def _next_point_cb(self, msg: Bool):
        """Avanza al siguiente waypoint cuando el controlador manda True."""
        if not msg.data or self.done:
            return

        self.wp_index += 1

        if self.wp_index >= len(self.waypoints):
            self.done = True
            self.get_logger().info('Trayectoria completada.')
        else:
            gx, gy = self.waypoints[self.wp_index]
            self.get_logger().info(f'Siguiente WP {self.wp_index}: ({gx:.2f}, {gy:.2f})')

    # ------------------------------------------------------------------
    # TIMER: publicar set_point actual
    # ------------------------------------------------------------------

    def _publish_current(self):
        """Publica continuamente el waypoint actual."""
        if self.done:
            return

        gx, gy = self.waypoints[self.wp_index]
        msg = Point()
        msg.x = float(gx)
        msg.y = float(gy)
        msg.z = 0.0
        self.set_point_pub.publish(msg)


# ----------------------------------------------------------------------
# ENTRYPOINT
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SetPointGenerator()
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