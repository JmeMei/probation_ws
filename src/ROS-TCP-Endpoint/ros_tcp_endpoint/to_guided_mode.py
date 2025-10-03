import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode

class ModeClient(Node):
    def __init__(self):
        super().__init__('mode')
        self.declare_parameter('mode', 'GUIDED')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

    def call_change_mode(self, mode):
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MAVROS set_mode service...")
        
        # Set mode
        mode_req = SetMode.Request()
        mode_req.base_mode = 0
        mode_req.custom_mode = mode
        self.get_logger().info(f'Setting mode: {mode}')
        mode_future = self.mode_cli.call_async(mode_req)

        self.get_logger().info('Mode change request sent successfully')

def main():
    rclpy.init()
    node = ModeClient()
    mode = node.get_parameter('mode').value
    node.call_change_mode(mode)
    rclpy.spin_once(node)  # Process the service calls
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            

