import rclpy 

from rclpy.node import Node 

from sensor_msgs.msg import LaserScan 

 

class LaserScanSub(Node): 

    def __init__(self): 

        super().__init__('laser_scan') 

        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 

        self.lidar = LaserScan() # Data from lidar will be stored here. 

        timer_period = 1.0 

        self.timer = self.create_timer(timer_period, self.timer_callback) 

        self.get_logger().info("Node initialized!!!") 

     

    def timer_callback(self): 
        # Calculations 
        """
        sensor_msgs.msg.LaserScan(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), angle_min=0.0, angle_max=0.0, angle_increment=0.0, time_increment=0.0, scan_time=0.0, range_min=0.0, range_max=0.0, ranges=[], intensities=[])

        """
        print(f"Angle_ mim: {self.lidar.angle_min} rad ")
        print(f"Angle_ max: {self.lidar.angle_max} rad ")

        print(f"Range:  {self.lidar.range_min} m")
        print(f"Range: {self.lidar.range_max} m")
        

        print(f"Name {self.lidar.header.frame_id}")

        print(self.lidar.ranges[0])
        print(self.lidar.intensities[-1])

 

    def lidar_cb(self, lidar_msg): 

        ## This function receives the ROS LaserScan message 

        self.lidar =  lidar_msg  

 

def main(args=None): 

    rclpy.init(args=args) 

    m_p=LaserScanSub() 

    rclpy.spin(m_p) 

    m_p.destroy_node() 

    rclpy.shutdown() 

     

if __name__ == '__main__': 

    main() 