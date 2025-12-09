import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class OdometryLoggerNode(Node):
    def __init__(self):
        super().__init__('odometry_logger_node')
        
        odom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a subscription to the /odom topic using the specialized QOS profile
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos_profile # Use the QOS Profile here
        )

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # 1. Define the relative log directory
        log_dir = './odom_logs'
        
        # 2. Ensure the directory exists (creates it if not, handles existing one gracefully)
        try:
            os.makedirs(log_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"Could not create log directory {log_dir}: {e}")
            # If directory creation fails, you should probably stop
            raise e
        
        # 3. Construct the full file path safely
        filename = f'log_{timestamp}.csv'
        self.log_filename = os.path.join(log_dir, filename) 
        
        # 4. Open the file
        try:
            # Change 'w' (text mode) to 'wb' (write binary mode) and keep buffering=0
            self.log_file = open(self.log_filename, 'wb', buffering=0) 
            
            # NOTE: Must encode strings to bytes for binary mode
            self.log_file.write("timestamp_sec,x_m,y_m\n".encode('utf-8')) 
            
            self.get_logger().info(f"Logging odometry data to {self.log_filename}")
        except Exception as e:
            self.get_logger().error(f"FATAL ERROR: Could not open log file at {self.log_filename}. Error: {e}")
            raise e

    def odom_callback(self, msg):
        # Data Extraction (Confirmed Correct)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Construct the string line
        data_string = f"{time_sec:.6f},{x:.4f},{y:.4f}\n"
        
        # Write and Flush (Must encode the string to bytes before writing)
        self.log_file.write(data_string.encode('utf-8'))
        
        # Flush is still correct, though often redundant with buffering=0
        self.log_file.flush()

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