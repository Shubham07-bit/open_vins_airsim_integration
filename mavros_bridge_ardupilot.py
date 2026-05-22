import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VinsToMavros(Node):

    def __init__(self):
        super().__init__('vins_to_mavros')

        self.sub = self.create_subscription(
            PoseStamped,
            '/ov_msckf/pose',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

    def callback(self, msg):
        out = PoseStamped()

        # Copy header
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'

        # (FLU->FRD) Swap Y and Z, and invert Y and Z
        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = -msg.pose.position.y
        out.pose.position.z = -msg.pose.position.z

        # ⚠️ Simple quaternion swap (works but not perfect)
        out.pose.orientation.x = msg.pose.orientation.x
        out.pose.orientation.y = -msg.pose.orientation.y
        out.pose.orientation.z = -msg.pose.orientation.z
        out.pose.orientation.w = msg.pose.orientation.w

        self.pub.publish(out)


def main():
    rclpy.init()
    node = VinsToMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()