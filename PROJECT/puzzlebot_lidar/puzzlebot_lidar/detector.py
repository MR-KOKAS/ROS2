import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
import math
import signal  
import sys 

# ros2 launch turtlebot3_gazebo empty_world.launch.py 
# ros2 run rviz2 rviz2
# ros2 run puzzlebot_lidar detector 


class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('laser_scan_subscriber') 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
         # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 
        self.lidar = LaserScan()  # Data from lidar will be stored here. 
        self.d_safety = 0.3
        self.kv = 0.4
        self.kw = 0.4
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!")

    def get_closest_object(self):
        ranges = self.lidar.ranges

        if len(ranges) == 0:
            return None, None

        filtered = [r if math.isfinite(r) else float('inf') for r in ranges]
        closest_range = min(filtered)

        if closest_range == float('inf'):
            return None, None

        closest_index = filtered.index(closest_range)
        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment
        theta_closest = math.atan2(math.sin(theta_closest), math.cos(theta_closest))

        return closest_range, theta_closest 

    def safety_distance_controller(self, d_closest, theta_closest):
        msg = Twist()

        d_diff = d_closest - self.d_safety

        msg.linear.x  = self.kv * d_diff
        msg.angular.z = self.kw * theta_closest

        self.pub.publish(msg)

        print("---------- Closest Object ----------")
        print(f"closest_range:   {d_closest:.4f} m")
        print(f"theta_closest:   {theta_closest:.4f} rad  ({math.degrees(theta_closest):.2f} deg)")
        print(f"d_diff:          {d_diff:.4f} m")
        print(f"v:               {msg.linear.x:.4f} m/s")
        print(f"w:               {msg.angular.z:.4f} rad/s")
        print("====================================")

    def timer_callback(self):
        d_closest, theta_closest = self.get_closest_object()

        if d_closest is not None:
            self.safety_distance_controller(d_closest, theta_closest)
        else:
            # Sin objeto detectado: detener el robot
            self.pub.publish(Twist())
            self.get_logger().warn("No object detected — robot stopped.")

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar = lidar_msg  

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program

def main(args=None): 
    rclpy.init(args=args) 
    m_p = LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()