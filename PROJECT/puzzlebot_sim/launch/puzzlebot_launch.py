import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file del paquete puzzlebot_sim - Mini Challenge 2 Extra.

    Nodos lanzados:
      1. robot_state_publisher  -> TFs internos del URDF
      2. puzzlebot_sim          -> modelo cinematico
      3. localisation           -> dead reckoning
      4. set_point_generator    -> genera waypoints y los pasa al controlador
      5. control                -> go-to-goal, publica next_point al llegar
      6. rviz2                  -> visualizacion 3D
      7. rqt_graph              -> grafico de nodos y topicos

    Uso desde terminal:
      ros2 launch puzzlebot_sim puzzlebot_launch.py trajectory:=square   side:=1.0
      ros2 launch puzzlebot_sim puzzlebot_launch.py trajectory:=pentagon radius:=1.0
      ros2 launch puzzlebot_sim puzzlebot_launch.py trajectory:=circle   radius:=1.0
    """

    pkg_share = get_package_share_directory('puzzlebot_sim')

    urdf_path        = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'puzzlebot_rviz.rviz')

    with open(urdf_path, 'r') as urdf_file:
        robot_description_str = urdf_file.read()

    # --- Argumentos de lanzamiento ---
    trajectory_arg  = DeclareLaunchArgument('trajectory',  default_value='square', description="'square' | 'pentagon' | 'circle'")
    side_arg        = DeclareLaunchArgument('side',        default_value='1.0',    description='Lado del cuadrado [m]')
    radius_arg      = DeclareLaunchArgument('radius',      default_value='1.0',    description='Radio del pentagono o circulo [m]')
    circle_pts_arg  = DeclareLaunchArgument('circle_pts',  default_value='36',     description='Puntos del circulo (suavidad)')
    linear_vel_arg  = DeclareLaunchArgument('linear_vel',  default_value='0.15',   description='Velocidad lineal [m/s]')
    angular_vel_arg = DeclareLaunchArgument('angular_vel', default_value='0.5',    description='Velocidad angular maxima [rad/s]')
    goal_tol_arg    = DeclareLaunchArgument('goal_tol',    default_value='0.05',   description='Tolerancia de llegada [m]')
    angle_tol_arg   = DeclareLaunchArgument('angle_tol',   default_value='0.05',   description='Tolerancia de angulo [rad]')

    return LaunchDescription([

        # Declarar argumentos
        trajectory_arg,
        side_arg,
        radius_arg,
        circle_pts_arg,
        linear_vel_arg,
        angular_vel_arg,
        goal_tol_arg,
        angle_tol_arg,

        # ------------------------------------------------------------------
        # robot_state_publisher
        # ------------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_str,
                'use_sim_time': False,
            }]
        ),

        # ------------------------------------------------------------------
        # puzzlebot_sim  (modelo cinematico)
        # ------------------------------------------------------------------
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_kinematic_model',
            parameters=[{'use_sim_time': False}]
        ),

        # ------------------------------------------------------------------
        # localisation  (dead reckoning)
        # ------------------------------------------------------------------
        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            parameters=[{'use_sim_time': False}]
        ),

        # ------------------------------------------------------------------
        # set_point_generator
        # Publica:   /set_point  (Point)
        # Suscribe:  /next_point (Bool)
        # ------------------------------------------------------------------
        Node(
            package='puzzlebot_sim',
            executable='set_point_generator',
            name='set_point_generator',
            parameters=[{
                'use_sim_time': False,
                'trajectory':   LaunchConfiguration('trajectory'),
                'side':         LaunchConfiguration('side'),
                'radius':       LaunchConfiguration('radius'),
                'circle_pts':   LaunchConfiguration('circle_pts'),
            }]
        ),

        # ------------------------------------------------------------------
        # control  (go-to-goal)
        # Suscribe:  /odom, /set_point
        # Publica:   /cmd_vel, /next_point
        # ------------------------------------------------------------------
        Node(
            package='puzzlebot_sim',
            executable='control',
            name='control',
            parameters=[{
                'use_sim_time':  False,
                'linear_vel':    LaunchConfiguration('linear_vel'),
                'angular_vel':   LaunchConfiguration('angular_vel'),
                'goal_tol':      LaunchConfiguration('goal_tol'),
                'angle_tol':     LaunchConfiguration('angle_tol'),
            }]
        ),

        # ------------------------------------------------------------------
        # RViz2
        # ------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': False}]
        ),

        # ------------------------------------------------------------------
        # rqt_graph
        # ------------------------------------------------------------------
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph'
        ),

    ])