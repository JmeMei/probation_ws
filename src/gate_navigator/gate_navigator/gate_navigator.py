# Example vision-guided controller node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import BoundingBoxArray  # auto-generated after build
from std_srvs.srv import Empty
from mavros_msgs.srv import SetMode

class GateNavigator(Node):
    def __init__(self):
        super().__init__('gate_navigator')
        self.sub = self.create_subscription(
            BoundingBoxArray,
            '/gate_detections',   # replace with actual bbox topic
            self.bbox_cb,
            10
        )
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.timer = self.create_timer(0.1, self.control_loop)

        self.last_bbox = None
        self.missed = 0
        self.state = 'SEARCH'

    def bbox_cb(self, msg):
        if not msg.bounding_boxes:
            self.missed += 1
            return
        # Simple: pick highest confidence
        bb = max(msg.bounding_boxes, key=lambda b: b.conf)
        self.last_bbox = bb
        self.missed = 0

    def control_loop(self):
        if self.state == 'SEARCH':
            if self.last_bbox:
                self.state = 'ALIGN'
            self.publish_cmd(yaw=0.2)  # slow turn
        elif self.state == 'ALIGN':
            if not self.last_bbox:
                self.state = 'SEARCH'
                self.publish_cmd(yaw=0.2)
                return
            ex = self.last_bbox.x - 0.5
            ey = self.last_bbox.y - 0.5
            if abs(ex) < 0.03 and abs(ey) < 0.03:
                self.state = 'APPROACH'
            self.publish_cmd(forward=0.0, yaw=-1.2*ex, vertical=-0.5*ey)
        elif self.state == 'APPROACH':
            if not self.last_bbox:
                # likely passed gate
                self.state = 'SEARCH'
                self.publish_cmd()
                return
            area = self.last_bbox.w * self.last_bbox.h
            ex = self.last_bbox.x - 0.5
            ey = self.last_bbox.y - 0.5
            if area > 0.25:
                self.state = 'PASS'
            self.publish_cmd(forward=0.4, yaw=-1.0*ex, vertical=-0.4*ey)
        elif self.state == 'PASS':
            # drive forward briefly
            self.publish_cmd(forward=0.6)
        else:
            self.publish_cmd()

    def publish_cmd(self, forward=0.0, yaw=0.0, vertical=0.0):
        msg = Twist()
        msg.linear.x = forward
        msg.linear.z = vertical
        msg.angular.z = yaw
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GateNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()