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
        # /mavros/setpoint_position/local:
        # Publish PoseStamped (ENU frame: x forward, y left, z up). Must stream (>2 Hz) while in OFFBOARD/GUIDED.
        # MAVROS forwards it as a local position target; one-shot publish is ignored by FCU position controllers.
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscribers
        self.sub_boxes = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.callback_bounding_boxes,
            10
        )
        self.sub_depth = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt', 
            self.callback_depth,
            10
        )

        # Depth Variables
        self.target_depth = -1.5804749727249146 #depth is constant
        self.depth_target_achieved = False
        self.current_depth = None          # will be set after first rel_alt message
        self.depth_received = False

        self.depth_setpoint_timer = self.create_timer(0.2, self.move_to_target_depth)

        # Vision
        # FIX: do not overwrite rotate_clockwise function
        # self.rotate_clockwise = self.create_timer(0.1, self.rotate_clockwise)
        # Depth setpoint publisher timer (5 Hz). Required to keep OFFBOARD/position control active.
        
        self.rotation_timer = self.create_timer(0.1, self._rotation_tick)
        self.passed_through_the_gate = False


    ########################## CALLBACKS ##########################    

    def callback_bounding_boxes(self, msg):
        # TODO: Implement vision processing
        pass
    
    def callback_depth(self, msg):
        # rel_alt published by MAVROS (Float64). Save it for control logic.
        self.current_depth = msg.data
        self.depth_received = True
        
    def callback_bounding_boxes(self, msg: BoundingBoxArray):
        if self.passed_through_the_gate:
            return

        if not self.depth_target_achieved:
            # Don't start gate navigation until depth is stable
            return

        # --- Search behavior: if no gate detected ---
        gate_box = None
        for box in msg.bounding_boxes:
            if box.label_name == "gate" and box.conf > 0.5:
                gate_box = box
                break

        if gate_box is None:
            self.get_logger().info("No gate detected. Rotating clockwise to search...")
            self.rotate_clockwise()
            return

        # --- Gate detected: extract normalized values ---
        x = gate_box.x  # [0,1], center position of gate in image (horizontal)
        y = gate_box.y  # [0,1], vertical center (we don't use this since depth is constant)
        w = gate_box.w
        h = gate_box.h
        area = w * h
        
        # --- Compute horizontal error relative to image center ---
        error_x = x - 0.5   # >0 → gate is right, <0 → gate is left

        # --- Simple proportional controller for left-right alignment ---
        k_side = 0.3           # tuning gain for sideways correction
        forward_speed = 0.3    # constant forward motion (m/s)

        # Calculate sideways motion (left/right)
        # If gate is to right (error_x > 0), need to move left (positive y in ENU)
        # If gate is to left (error_x < 0), need to move right (negative y in ENU)
        side_correction = -k_side * error_x
        if(error_x > 0): 
            self.publish_velocity_command(
                linear_x=forward_speed,
                # linear_y=1.0,  # FIXED: Use linear_y for horizontal/sideways motion
                angular_z = -0.5,
                linear_z=0.0  # Keep depth fixed
            )
        else:
            self.publish_velocity_command(
                linear_x=forward_speed,
                # linear_y=-1.0,  # FIXED: Use linear_y for horizontal/sideways motion
                angular_z = 0.5,
                linear_z=0.0  # Keep depth fixed
            )

        # --- Stopping condition: if gate fills enough of the view ---
        if area > 0.40:
            self.get_logger().info("Gate is close (large in view). Stopping movement.")
            while(gate_box):
                self.move_forward(speed=0.3)
            self.stop_movement()
            self.passed_through_the_gate = True
            return

        # # --- Publish movement command: forward + sideways correction ---
        # self.publish_velocity_command(
        #     linear_x=forward_speed,
        #     linear_y=side_correction,  # FIXED: Use linear_y for horizontal/sideways motion
        #     linear_z=0.0  # Keep depth fixed
        # )

        self.get_logger().info(
            f"Gate detected at (x={x:.2f}, y={y:.2f}), size=({w:.2f},{h:.2f}), "
            f"error_x={error_x:.2f}, cmd: fwd={forward_speed:.2f}, side={side_correction:.2f}"
        )


    # def callback_bounding_boxes(self, msg: BoundingBoxArray):
    #     if not self.depth_target_achieved:
    #         return

    #     # Modified: rotate only if NO gate bounding box (with sufficient confidence) exists
    #     while not any(
    #         getattr(box, 'label_name', '') == "gate" and getattr(box, 'conf', 0.0) > 0.5
    #         for box in msg.bounding_boxes
    #     ):
    #         self.get_logger().info("No 'gate' bounding box detected. Rotating clockwise")
    #         self.rotate_clockwise()
    #         return

    #     # At least one gate-labeled box (conf > 0.5) exists; pick first passing threshold
    #     gate_box = None
    #     for box in msg.bounding_boxes:
    #         if box.label_name == "gate" and box.conf > 0.5:
    #             gate_box = box
    #             break

    #     if gate_box is None:
    #         # This can occur if label present but confidence filtering logic changes later
    #         self.get_logger().info("Gate label present but no box passed confidence filter.")
    #         return

    #     # Center of bounding box (normalized [0,1])
    #     x = gate_box.x
    #     y = gate_box.y
    #     w = gate_box.w
    #     h = gate_box.h

    #     # Compute errors relative to image center (0.5, 0.5)
    #     error_x = x - 0.5   # >0 → gate is to the right
    #     error_y = y - 0.5   # >0 → gate is below

    #     # Simple proportional controller
    #     k_y = 0.5   # gain for horizontal correction
    #     k_z = 0.5   # gain for vertical correction


    #     forward_speed = 0.3  # constant forward motion (m/s)

    #     linear_y = -k_y * error_x   # negative: if gate is right, move right
    #     linear_z = -k_z * error_y   # negative: if gate is below, move down


    #     # Optional stopping condition: if gate is big enough in frame
    #     if w > 0.5 and h > 0.5:
    #         self.get_logger().info("Gate reached (large in view), stopping.")
    #         self.stop_movement()
    #         return

    #     # Publish velocity command
    #     self.publish_velocity_command(
    #         linear_x=forward_speed,  # always move forward
    #         linear_y=linear_y,
    #         linear_z=linear_z
    #     )

    #     self.get_logger().info(
    #         f"Gate detected at (x={x:.2f}, y={y:.2f}), size=({w:.2f},{h:.2f}), "
    #         f"errors: (ex={error_x:.2f}, ey={error_y:.2f}), "
    #         f"cmd: fwd={forward_speed:.2f}, y={linear_y:.2f}, z={linear_z:.2f}"
    #     )


    ########################## END OF CALLBACKS ########################## 
    def move_to_target_depth(self):
        # Called periodically by self.depth_setpoint_timer
        if self.depth_target_achieved:
            return
        if not self.depth_received:
            # Haven't received rel_alt yet
            return

        # Decide based on current_depth vs target_depth
        # User request: when current_depth <= target_depth -> stop, else keep sinking
        # In here, I have received the depth AND I have the current depth, so I can compare
        if self.current_depth <= self.target_depth:
            self.depth_target_achieved = True
            self.stop_movement()
            self.get_logger().info(
                f"Target depth reached: current={self.current_depth:.3f} target={self.target_depth:.3f}. Stopping.")
            if self.depth_setpoint_timer:
                self.depth_setpoint_timer.cancel() # stop this timer
        else:
            # Continue descending
            self.sink()
            self.get_logger().info(
                f"Sinking: current={self.current_depth:.3f} > target={self.target_depth:.3f}")

    def _rotation_tick(self):
        # call only if you actually want continuous rotation; comment out if not needed
        # self.rotate_clockwise()
        if not self.depth_target_achieved:
          return  

    
    ########################## TO MOVE COMMANDS ##########################
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
    
    def rotate_clockwise(self, yaw_rate=0.3):
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

    def move_forward(self, speed=0.3):
        """Move the vehicle forward at specified speed."""
        self.publish_velocity_command(linear_x=speed)  # Positive for forward movement
        self.get_logger().info(f'Published forward command with speed of {speed} m/s.')

    def move_backward(self, speed=0.3):
        """Move the vehicle backward at specified speed."""
        self.publish_velocity_command(linear_x=-speed)  # Negative for backward movement
        self.get_logger().info(f'Published backward command with speed of {speed} m/s.')

    def move_left(self, speed=0.3):
        """Move the vehicle left at specified speed."""
        self.publish_velocity_command(linear_y=speed)  # Positive for left movement
        self.get_logger().info(f'Published left command with speed of {speed} m/s.')

    def move_right(self, speed=0.3):
        """Move the vehicle right at specified speed."""
        self.publish_velocity_command(linear_y=-speed)  # Negative for right movement
        self.get_logger().info(f'Published right command with speed of {speed} m/s.')

    def sink(self):
        """Sink the vehicle by publishing a downward velocity command."""
        self.publish_velocity_command(linear_z=-0.5)  # Negative for downward movement
        self.get_logger().info('Published sink command with downward velocity of 0.5 m/s.')

    ########################## END OF TO MOVE COMMAND ##########################

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