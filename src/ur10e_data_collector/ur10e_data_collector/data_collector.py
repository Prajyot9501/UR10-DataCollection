import rclpy
import csv
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class UR10eDataCollector(Node):
    def __init__(self):
        super().__init__('ur10e_data_collector')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.file = open('/tmp/ur10e_joint_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['Timestamp', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        self.get_logger().info('UR10e Data Collector Node started')
        self.get_logger().info('Writing data to /tmp/ur10e_joint_data.csv')
        
    def joint_state_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec
        joint_positions = [f"{p:.4f}" for p in msg.position]
        self.writer.writerow([timestamp] + joint_positions)
        self.get_logger().info(f"Logged: {joint_positions}")
        
    def destroy_node(self):
        if hasattr(self, 'file') and self.file is not None:
            self.file.close()
            self.get_logger().info('Closed data logging file')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UR10eDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UR10e Data Collector Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()