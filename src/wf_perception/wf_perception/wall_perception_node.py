# 

# wall_perception_node_tb4.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from std_msgs.msg import Float64MultiArray
import os

# Expected TB4 frame names (from your echo)
FRAMES = [
    'ir_intensity_side_left',
    'ir_intensity_left',
    'ir_intensity_front_left',
    'ir_intensity_front_center_left',
    'ir_intensity_front_center_right',
    'ir_intensity_front_right',
    'ir_intensity_right'
]

class WallPerceptionNode(Node):
    def __init__(self):
        super().__init__('wall_perception_node_tb4')

        # Parameters (tune these)
        self.declare_parameter('target_intensity', 100.0)   # desired intensity ~ distance target
        self.declare_parameter('init_threshold', 30.0)     # minimum intensity to consider a wall present
        self.declare_parameter('left_weight', 1.0)         # relative weight for left cluster
        self.declare_parameter('right_weight', 1.0)        # relative weight for right cluster
        self.declare_parameter('front_weight', 1.0)        # weight for front-center sensors in angle calc

        self.target_intensity = self.get_parameter('target_intensity').value
        self.init_threshold = self.get_parameter('init_threshold').value

        # Subscribers / Publishers
        self.subscription = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.ir_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Float64MultiArray, '/wall_tracking', 10)

        self.side_determined = False
        self.side = None  # 'left' or 'right'

        self.get_logger().info('WallPerceptionNode (TB4) started.')

    def _readings_to_dict(self, msg):
        return {r.header.frame_id: r.value for r in msg.readings}

    def _choose_side(self, readings):
        # Define clusters (these are proxies based on TB4 front fan)
        left_cluster = [
            'ir_intensity_side_left',
            'ir_intensity_left',
            'ir_intensity_front_left'
        ]
        right_cluster = [
            'ir_intensity_right',
            'ir_intensity_front_right',
            'ir_intensity_front_center_right'
        ]

        left_sum = sum(readings.get(k, 0) for k in left_cluster)
        right_sum = sum(readings.get(k, 0) for k in right_cluster)

        # If both low, return None
        if left_sum < self.init_threshold and right_sum < self.init_threshold:
            return None

        return 'left' if left_sum >= right_sum else 'right'

    def _compute_proxies(self, readings, side):
        # Virtual "rear" proxy (stable lateral distance) uses side + immediate neighbor
        if side == 'left':
            rear_keys = ['ir_intensity_side_left', 'ir_intensity_left']
            front_keys = ['ir_intensity_front_left', 'ir_intensity_front_center_left']
        else:  # right
            rear_keys = ['ir_intensity_right', 'ir_intensity_front_right']  # treat front_right as nearer-right
            front_keys = ['ir_intensity_front_center_right', 'ir_intensity_front_right']

        # average, guard against missing entries
        rear_vals = [readings.get(k, 0) for k in rear_keys]
        front_vals = [readings.get(k, 0) for k in front_keys]

        # If rear_vals all zero but front provides signal, still use front as proxy
        rear_proxy = sum(rear_vals) / (len([v for v in rear_vals if v is not None]) or 1)
        front_proxy = sum(front_vals) / (len([v for v in front_vals if v is not None]) or 1)

        return rear_proxy, front_proxy

    def ir_callback(self, msg):
        readings = self._readings_to_dict(msg)

        # Quick check: if none of the expected frames are present, warn once
        if not any(f in readings for f in FRAMES):
            self.get_logger().warn('No expected IR frames present in message.')
            return

        # Determine side if not yet determined
        if not self.side_determined:
            side_guess = self._choose_side(readings)
            if side_guess is None:
                # No wall detected yet
                out = Float64MultiArray()
                out.data = [0.0, 0.0, 0.0]
                self.publisher.publish(out)
                self.get_logger().debug('No wall detected yet (waiting).')
                return
            self.side = side_guess
            self.side_determined = True
            self.get_logger().info(f'Wall side determined: {self.side}')

        # If determined, compute proxies
        rear_proxy, front_proxy = self._compute_proxies(readings, self.side)

        # distance_error: positive => robot too far (needs to move toward wall)
        distance_error = float(self.target_intensity - rear_proxy)

        # angle_error: positive => front is stronger than rear => angled TOWARD the wall
        angle_error = float(front_proxy - rear_proxy)

        # side_flag: -1 for left, +1 for right (used by controller to flip sign consistently)
        side_flag = -1.0 if self.side == 'left' else 1.0

        out = Float64MultiArray()
        out.data = [distance_error, angle_error, side_flag]
        self.publisher.publish(out)

        # Debug logging (low-rate to avoid flooding; can be elevated while tuning)
        self.get_logger().debug(f'Published dist_err={distance_error:.2f}, ang_err={angle_error:.2f}, side={self.side}')

def main(args=None):
    rclpy.init(args=args)
    node = WallPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
