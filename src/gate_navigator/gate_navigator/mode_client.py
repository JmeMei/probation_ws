import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool

class ModeClient(Node):
    def __init__(self):
        super().__init__('mode_client')
        self.declare_parameter('mode', 'GUIDED')
        self.declare_parameter('arm', True)
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.timer = self.create_timer(0.5, self.tick)
        self.done = False

    def tick(self):
        if self.done:
            return
        if not self.mode_cli.service_is_ready() or not self.arm_cli.service_is_ready():
            self.get_logger().info('Waiting for MAVROS services...')
            return
        mode = self.get_parameter('mode').value
        arm_flag = self.get_parameter('arm').value

        mode_req = SetMode.Request()
        mode_req.base_mode = 0
        mode_req.custom_mode = mode
        self.get_logger().info(f'Setting mode: {mode}')
        self.mode_cli.call_async(mode_req)

        if arm_flag:
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.get_logger().info('Arming vehicle')
            self.arm_cli.call_async(arm_req)

        self.done = True
        self.get_logger().info('Requests sent (mode / arm). You can Ctrl+C or let it spin.')

def main():
    rclpy.init()
    node = ModeClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
