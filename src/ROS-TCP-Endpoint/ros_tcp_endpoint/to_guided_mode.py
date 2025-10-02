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

    def idk(self):
            while not self.client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for MAVROS services...")
        

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


#####################
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class ModeClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, request=request))

    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))

def main(args=None):
    rclpy.init(args=args)
    node = ModeClient()
    node.call_add_two_ints(2, 7)
    node.call_add_two_ints(1, 4)
    node.call_add_two_ints(10, 20)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
