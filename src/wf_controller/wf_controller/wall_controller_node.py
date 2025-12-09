# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import Twist
# import numpy as np 

# class WallControllerNode(Node):
#     def __init__(self):
#         super().__init__('wall_controller_node')
#         # ... (Subscription and Publisher setup)
        
#         # --- TUNING PARAMETERS (REDUCED) ---
#         self.kd = 0.001           # Adjusted significantly lower
#         self.ktheta = 0.0005      # Adjusted significantly lower
        
#         self.v_nominal_high = 0.30 
#         self.v_nominal_low = 0.10   

#         # --- SAFETY THRESHOLDS ---
#         self.DISTANCE_TOLERANCE = 50 
#         self.ANGLE_TOLERANCE = 10 
        
#         # --- Minimum Guaranteed Linear Velocity (REDUCED) ---
#         self.V_MIN_GUARANTEED = 0.05 # Reduced minimum speed
    

#     def callback(self, msg):
#         distance_delta = msg.data[0] 
#         angle_error = msg.data[1]    

#         # 1. Calculate Angular Velocity (Omega)
#         omega = self.kd * distance_delta + self.ktheta * angle_error
        
#         # Optional: Clamp Angular Velocity (prevent violent turns)
#         max_omega = 0.5 # radians/second
#         omega = np.clip(omega, -max_omega, max_omega)

#         # --- 2. Safety and Mode Switching Logic (Modified) ---
        
#         is_tracking_well = (
#             abs(distance_delta) < self.DISTANCE_TOLERANCE and 
#             abs(angle_error) < self.ANGLE_TOLERANCE
#         )

#         if is_tracking_well:
#             # Mode 1: Cruising Speed
#             v = self.v_nominal_high
#         else:
#             # Mode 2: Correction Speed
#             v = self.v_nominal_low
        
#         # --- CRITICAL FIX 1: Ensure Minimum Forward Movement ---
#         # The linear velocity must be at least V_MIN_GUARANTEED
#         v = max(v, self.V_MIN_GUARANTEED) 

#         # --- CRITICAL FIX 2: Velocity Suppression for Extreme Turns ---
#         # If the robot is commanded to turn very sharply, slow down the linear speed.
#         # This prevents the robot from driving into the wall while turning.
        
#         # Calculate a suppression factor based on omega. Example: 1.0 - abs(omega) / max_omega
#         # When omega is 0, factor is 1. When omega is max_omega, factor is 0.
#         omega_ratio = abs(omega) / max_omega 
        
#         # Suppress V further if omega is large, but never below V_MIN
#         v_suppressed = v * (1.0 - 0.75 * omega_ratio) # 0.75 is a scaling factor
        
#         v = max(v_suppressed, self.V_MIN_GUARANTEED)

#         # --- 3. Publish Twist Command ---
#         twist = Twist()
#         twist.linear.x = float(v)
#         twist.angular.z = float(omega)
#         self.publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WallControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# wall_controller_node_tb4.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class WallControllerNode(Node):
    def __init__(self):
        super().__init__('wall_controller_node_tb4')

        # Tunable parameters (declare to allow runtime changes)
        self.declare_parameter('kd', 0.002)            # distance gain
        self.declare_parameter('ktheta', 0.0015)       # angle gain
        self.declare_parameter('v_nominal_high', 0.25)
        self.declare_parameter('v_nominal_low', 0.12)
        self.declare_parameter('v_min_guaranteed', 0.05)
        self.declare_parameter('max_omega', 0.6)
        self.declare_parameter('distance_tolerance', 40.0)
        self.declare_parameter('angle_tolerance', 8.0)

        self.kd = self.get_parameter('kd').value
        self.ktheta = self.get_parameter('ktheta').value
        self.v_nominal_high = self.get_parameter('v_nominal_high').value
        self.v_nominal_low = self.get_parameter('v_nominal_low').value
        self.V_MIN_GUARANTEED = self.get_parameter('v_min_guaranteed').value
        self.max_omega = self.get_parameter('max_omega').value
        self.DISTANCE_TOLERANCE = self.get_parameter('distance_tolerance').value
        self.ANGLE_TOLERANCE = self.get_parameter('angle_tolerance').value

        # Pub/Sub
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/wall_tracking',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('WallControllerNode (TB4) started.')

    def callback(self, msg: Float64MultiArray):
        # Expecting [distance_error, angle_error, side_flag]
        if len(msg.data) < 3:
            self.get_logger().warn('wall_tracking message malformed (expected 3 values).')
            return

        distance_delta = float(msg.data[0])
        angle_error = float(msg.data[1])
        side_flag = float(msg.data[2])  # -1 for left, +1 for right

        # Compute angular velocity.
        # We choose sign convention: positive omega -> rotate COUNTERCLOCKWISE (left turn).
        # For right-wall following, we need to invert some terms so that positive distance_error
        # results in turning toward the wall. Use side_flag to flip.
        omega = (self.kd * distance_delta + self.ktheta * angle_error) * (-side_flag)

        # Clamp omega
        omega = float(np.clip(omega, -self.max_omega, self.max_omega))

        # Speed mode selection
        is_tracking_well = (abs(distance_delta) < self.DISTANCE_TOLERANCE and abs(angle_error) < self.ANGLE_TOLERANCE)
        v = self.v_nominal_high if is_tracking_well else self.v_nominal_low
        v = max(v, self.V_MIN_GUARANTEED)

        # Suppress linear speed for aggressive turns
        omega_ratio = min(1.0, abs(omega) / self.max_omega)
        v_suppressed = v * (1.0 - 0.75 * omega_ratio)
        v = max(v_suppressed, self.V_MIN_GUARANTEED)

        # Build and publish Twist
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega)
        self.publisher.publish(twist)

        # Optional debug
        self.get_logger().debug(f'cmd -> v={v:.3f}, omega={omega:.3f}; dist_err={distance_delta:.1f}, ang_err={angle_error:.1f}, side={side_flag}')

def main(args=None):
    rclpy.init(args=args)
    node = WallControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
