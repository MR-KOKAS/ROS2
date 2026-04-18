# Importaciones necesarias para definir y ejecutar archivos de lanzamiento (launch) en ROS 2
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Función que configura los nodos y lanzamientos incluidos
def launch_setup(context, *args, **kwargs):
    # Obtiene el valor del argumento 'map_name' (nombre del mapa) desde el archivo de lanzamiento
    map_name = LaunchConfiguration('map_name').perform(context)
    
    # Obtiene el path del paquete principal que contiene configuraciones de navegación
    base_path = get_package_share_directory('puzzlebot_navigation2')
    
    # Ruta del archivo de configuración de RViz para visualizar el mapeo
    rviz_file = os.path.join(base_path, 'rviz', "slam.rviz")
    
    # Obtiene las rutas de los paquetes nav2_bringup y slam_toolbox
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Diccionario con argumentos comunes, como si se usa el tiempo simulado
    args = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # Si el mapa se llama 'puzzlebot', se agrega la configuración de SLAM
    if map_name.lower() == 'puzzlebot':
        args['slam_params_file'] = LaunchConfiguration('slam_params_file')

    # Devuelve una lista de acciones que se lanzarán
    return [
        # Declaración del argumento 'use_sim_time' con valor por defecto 'true'
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Declaración del argumento 'slam_params_file' con un archivo YAML por defecto
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(base_path, 'config', 'slam_toolbox.yaml'),
            description='Full path to the Slam Toolbox configuration file'
        ),

        # Inclusión del lanzamiento principal de navegación de Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),

        # Inclusión del lanzamiento del SLAM Toolbox en modo 'online_async'
        # Este modo realiza mapeo en tiempo real con optimización asíncrona (ideal para SBCs)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments=args.items()
        ),

        # Nodo de RViz para visualizar el proceso de SLAM
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ]

# Función principal que genera la descripción del lanzamiento
def generate_launch_description():
    return LaunchDescription([
        # Se declaran argumentos iniciales del archivo de lanzamiento
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='hexagonal'),
        # Se utiliza OpaqueFunction para ejecutar dinámicamente la función 'launch_setup'
        OpaqueFunction(function=launch_setup)
    ])
