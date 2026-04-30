import rclpy 

from rclpy.node import Node 

from geometry_msgs.msg import Twist 

import signal  

import sys 

 

# A simple node example 

class MyNodeClass(Node):  

    def __init__(self):  

        super().__init__('my_node') 

        ###########  YOUR CODE  ################ 

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) 

        # Handle shutdown gracefully 

        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

         

        ########### MORE CODE ################ 

        timer_period = 0.5  

        self.create_timer(timer_period, self.main_timer_cb) 

        self.get_logger().info("Node initialized!!") 

  

    def main_timer_cb(self): 

        print("Hello from the main timer callback") 

 

    def shutdown_function(self, signum, frame): 

        # Handle shutdown gracefully 

        # This function will be called when Ctrl+C is pressed 

        # It will stop the robot and shutdown the node 

        self.get_logger().info("Shutting down. Stopping robot...") 

        stop_twist = Twist()  # All zeros to stop the robot 

        self.pub_cmd_vel.publish(stop_twist) # publish it to stop the robot before shutting down 

        rclpy.shutdown() # Shutdown the node 

        sys.exit(0) # Exit the program 

         

def main(args=None): 

    rclpy.init(args=args) 

    my_node=MyNodeClass() 

    rclpy.spin(my_node) 

    my_node.destroy_node() 

    rclpy.shutdown() 

     

if __name__ == '__main__': 

    main() 