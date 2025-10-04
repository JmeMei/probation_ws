import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode

class ModeClient(Node):
    MAV_MODE_GUIDED_ARMED = 216
    
    def __init__(self):
        super().__init__('mode')
        #self.declare_parameter('mode', 'GUIDED')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def call_change_mode(self, mode):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MAVROS set_mode service...")
        
        # Set mode
        mode_req = SetMode.Request()
        mode_req.base_mode = self.MAV_MODE_GUIDED_ARMED  # Use the constant
        mode_req.custom_mode = mode
        self.get_logger().info(f'Setting mode: {mode}')
        mode_future = self.mode_client.call_async(mode_req)

        # Wait for the response and handle it
        rclpy.spin_until_future_complete(self, mode_future)

        if mode_future.result() is not None:
            response = mode_future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode change to {mode} successful!')
            else:
                self.get_logger().error(f'Mode change to {mode} failed!')
        else:
            self.get_logger().error('Service call failed')

def main():
    rclpy.init()
    node = ModeClient()
    mode = node.get_parameter('mode').value
    node.call_change_mode(mode)
    rclpy.spin_once(node)  # so that you don't have to ctrl-c
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
 Commands that I have used to check the mode:
- ros2 service list # check available services

- ros2 service type /mavros/set_mode # check service type

- ros2 interface show mavros_msgs/srv/SetMode

 # set FCU mode
#
# Known custom modes listed here:
# http://wiki.ros.org/mavros/CustomModes

# basic modes from MAV_MODE
uint8 MAV_MODE_PREFLIGHT                = 0
uint8 MAV_MODE_STABILIZE_DISARMED       = 80
uint8 MAV_MODE_STABILIZE_ARMED          = 208
uint8 MAV_MODE_MANUAL_DISARMED          = 64
uint8 MAV_MODE_MANUAL_ARMED             = 192
uint8 MAV_MODE_GUIDED_DISARMED          = 88
uint8 MAV_MODE_GUIDED_ARMED             = 216
uint8 MAV_MODE_AUTO_DISARMED            = 92
uint8 MAV_MODE_AUTO_ARMED               = 220
uint8 MAV_MODE_TEST_DISARMED            = 66
uint8 MAV_MODE_TEST_ARMED               = 194

uint8 base_mode         # filled by MAV_MODE enum value or 0 if custom_mode != ''
string custom_mode      # string mode representation or integer
---
bool mode_sent          # Mode known/parsed correctly and SET_MODE are sent

If you want to check the current mode using the command line, you can use:
- ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 
"""


