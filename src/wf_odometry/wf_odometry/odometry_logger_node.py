import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class OdometryLoggerNode(Node):
    def __init__(self):
        super().__init__('odometry_logger_node')
        
        # Create a subscription to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10) # QoS default is fine for odometry
        
        # --- File Setup ---
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_filename = f'robot_path_log_{timestamp}.csv'
        self.log_file = open(self.log_filename, 'w')
        self.log_file.write("timestamp_sec,x_m,y_m\n") # Write header
        self.get_logger().info(f"Logging odometry data to {self.log_filename}")

    def odom_callback(self, msg):
        # Extract the pose data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract the timestamp
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Write to the file
        self.log_file.write(f"{time_sec:.6f},{x:.4f},{y:.4f}\n")

    def destroy_node(self):
        # Ensure the file is closed when the node shuts down
        if self.log_file:
            self.log_file.close()
            self.get_logger().info("Odometry logging stopped and file closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()