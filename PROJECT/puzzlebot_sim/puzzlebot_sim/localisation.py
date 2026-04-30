import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class Localisation(Node):
    """
    Nodo ROS2 - Localizacion por Dead Reckoning (Mini Challenge 2, Part 2).

    Suscripciones:
      - /wr  (Float32)  -> velocidad angular rueda derecha [rad/s]
      - /wl  (Float32)  -> velocidad angular rueda izquierda [rad/s]

    Publicaciones:
      - /odom (Odometry)  -> odometria integrada del robot
      - TF dinamico: odom -> base_footprint

    Modelo cinematico (Euler explicito):
      x_{k+1}     = x_k     + v * cos(theta_k) * dt
      y_{k+1}     = y_k     + v * sin(theta_k) * dt
      theta_{k+1} = theta_k + omega * dt

    Donde:
      v     = r * (wr + wl) / 2
      omega = r * (wr - wl) / l
    """

    UPDATE_RATE_HZ = 20

    # Parametros fisicos del Puzzlebot
    WHEEL_RADIUS = 0.05   # r [m]
    WHEEL_BASE   = 0.19   # l [m]

    def __init__(self):
        super().__init__('localisation')

        # --- Broadcasters TF ---
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster  = StaticTransformBroadcaster(self)

        # --- Publisher ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # --- Subscribers ---
        self.create_subscription(Float32, 'wr', self._wr_cb, 10)
        self.create_subscription(Float32, 'wl', self._wl_cb, 10)

        # --- Velocidades de rueda actuales ---
        self.wr = 0.0
        self.wl = 0.0

        # --- Estado del robot ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Paso de tiempo
        self.dt = 1.0 / self.UPDATE_RATE_HZ

        # TF estatico map -> odom
        self._publish_static_map_to_odom()

        # Timer principal
        self.create_timer(self.dt, self._update_cb)

        self.get_logger().info(
            'Localisation (dead reckoning) inicializado. '
            f'r={self.WHEEL_RADIUS} m  l={self.WHEEL_BASE} m'
        )

    # ------------------------------------------------------------------
    # SUBSCRIBERS
    # ------------------------------------------------------------------

    def _wr_cb(self, msg: Float32):
        self.wr = msg.data

    def _wl_cb(self, msg: Float32):
        self.wl = msg.data

    # ------------------------------------------------------------------
    # TF ESTATICO: map -> odom
    # ------------------------------------------------------------------

    def _publish_static_map_to_odom(self):
        tf_static = TransformStamped()
        tf_static.header.stamp    = RclpyTime(seconds=0).to_msg()
        tf_static.header.frame_id = 'map'
        tf_static.child_frame_id  = 'odom'

        tf_static.transform.translation.x = 0.0
        tf_static.transform.translation.y = 0.0
        tf_static.transform.translation.z = 0.0
        tf_static.transform.rotation.x    = 0.0
        tf_static.transform.rotation.y    = 0.0
        tf_static.transform.rotation.z    = 0.0
        tf_static.transform.rotation.w    = 1.0

        self.static_tf_broadcaster.sendTransform(tf_static)
        self.get_logger().info('TF estatico map -> odom publicado.')

    # ------------------------------------------------------------------
    # CALLBACK PRINCIPAL (20 Hz)
    # ------------------------------------------------------------------

    def _update_cb(self):
        """Integra odometria con Euler explicito y publica /odom y TF."""

        # 1. Calcular v y omega desde velocidades de rueda
        v     = self.WHEEL_RADIUS * (self.wr + self.wl) / 2.0
        omega = self.WHEEL_RADIUS * (self.wr - self.wl) / self.WHEEL_BASE

        # 2. Integrar modelo cinematico (Euler explicito)
        #    x_{k+1}     = x_k     + v * cos(theta_k) * dt
        #    y_{k+1}     = y_k     + v * sin(theta_k) * dt
        #    theta_{k+1} = theta_k + omega * dt
        self.x     += v * math.cos(self.theta) * self.dt
        self.y     += v * math.sin(self.theta) * self.dt
        self.theta += omega * self.dt

        # Normalizar theta en [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 3. Construir cuaternion manualmente (sin librerias externas)
        #    Para yaw: q = (cos(theta/2), 0, 0, sin(theta/2))  -> (w, x, y, z)
        half = self.theta / 2.0
        qw   =  math.cos(half)
        qx   =  0.0
        qy   =  0.0
        qz   =  math.sin(half)

        now = self.get_clock().now().to_msg()

        # 4. Publicar Odometry
        self._publish_odom(now, v, omega, qw, qx, qy, qz)

        # 5. Publicar TF odom -> base_footprint
        self._publish_tf(now, qw, qx, qy, qz)

    # ------------------------------------------------------------------
    # PUBLISHERS
    # ------------------------------------------------------------------

    def _publish_odom(self, stamp, v, omega, qw, qx, qy, qz):
        """Publica mensaje Odometry en /odom."""
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_footprint'

        # Pose
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz

        # Twist (velocidades en frame del robot)
        msg.twist.twist.linear.x  = v
        msg.twist.twist.angular.z = omega

        # Covarianza no requerida — se deja en ceros por defecto

        self.odom_pub.publish(msg)

    def _publish_tf(self, stamp, qw, qx, qy, qz):
        """Publica TF dinamico odom -> base_footprint."""
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_footprint'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w    = qw
        tf.transform.rotation.x    = qx
        tf.transform.rotation.y    = qy
        tf.transform.rotation.z    = qz

        self.dynamic_tf_broadcaster.sendTransform(tf)


# ----------------------------------------------------------------------
# ENTRYPOINT
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
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