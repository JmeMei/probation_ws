#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray 
from std_msgs.msg import Float64 #for depth 
from geometry_msgs.msg import Twist, PoseStamped

class GateNavigatorNode(Node):
    def __init__(self):
        super().__init__('gate_navigator_node')
        self.get_logger().info('Gate Navigator Node has been started.')

        # publishers
        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscribers
        self.sub_boxes = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.vision_callback,
            10
        )
        self.sub_depth = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.depth_callback,
            10
        )

        # Variables
        self.target_depth = -1.5804749727249146 #depth is constant
        self.depth_target_achieved = False
        self.current_depth = 0.0  # initialize so logging doesn't fail

        
        self.depth_setpoint_timer = self.create_timer(0.2, self.move_to_target_depth)

        # FIX: do not overwrite rotate_clockwise function
        # self.rotate_clockwise = self.create_timer(0.1, self.rotate_clockwise)
        # Depth setpoint publisher timer (5 Hz). Required to keep OFFBOARD/position control active.
        self.rotation_timer = self.create_timer(0.1, self._rotation_tick)

        


    def vision_callback(self, msg):
        # TODO: Implement vision processing
        pass
    
    def depth_callback(self, msg):
        self.current_depth = msg.data
        if (not self.depth_target_achieved and
                self.current_depth <= self.target_depth + 0.1):
            self.depth_target_achieved = True
            self.get_logger().info('Target depth reached. Stopping depth setpoints.')
            if self.depth_setpoint_timer:
                self.depth_setpoint_timer.cancel()

    ############# TO MOVE COMMAND #############
    def publish_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                                angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        Publish velocity command to move the drone
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s) 
            linear_z: Up/down velocity (m/s)
            angular_x: Roll rate (rad/s)
            angular_y: Pitch rate (rad/s)
            angular_z: Yaw rate (rad/s) - positive for counterclockwise, negative for clockwise
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z
        
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(f'Published velocity command: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})')
    
    def rotate_clockwise(self, yaw_rate=1.0):
        """
        Rotate the drone clockwise at specified yaw rate
        Args:
            yaw_rate: Rotation speed in rad/s (positive value will be made negative for clockwise)
        """
        self.publish_velocity_command(angular_z=-abs(yaw_rate))  # Negative for clockwise

    def stop_movement(self):
        """Stop all movement by publishing zero velocities."""
        self.publish_velocity_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Published stop command.')

    def sink(self):
        """Sink the vehicle by publishing a downward velocity command."""
        self.publish_velocity_command(linear_z=-0.2)  # Negative for downward movement
        self.get_logger().info('Published sink command with downward velocity of 0.2 m/s.')

    ############# END OFTO MOVE COMMAND #############

    # Move to target depth
    def move_to_target_depth(self):
        if self.depth_target_achieved:
            return
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.target_depth  # adjust if ENU vs NED mismatch
        self.pos_pub.publish(pose)
        self.get_logger().info(
            f"Depth setpoint: {self.target_depth:.2f} (current {self.current_depth:.2f})")
        

    def _rotation_tick(self):
        # call only if you actually want continuous rotation; comment out if not needed
        # self.rotate_clockwise()
        if not self.depth_target_achieved:
          return

def main(args=None):
    rclpy.init(args=args)
    node = GateNavigatorNode()
    # OPTIONAL: start rotation after some delay if needed
    # node.rotate_clockwise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Available Position-Related Topics:
/mavros/global_position/rel_alt - Relative altitude (depth) only

This gives you Z-position (height/depth) relative to home
You're already using this in your code
/mavros/global_position/compass_hdg - Compass heading

Gives you yaw/orientation but not position
/mavros/imu/data - IMU data

Provides orientation and angular velocities, not position
"""