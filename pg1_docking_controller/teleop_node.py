import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('axis_linear.x', 5)
        self.declare_parameter('axis_linear.reverse_x', 2)
        self.declare_parameter('scale_linear.x', 0.5)
        self.declare_parameter('scale_linear_turbo.x', 1)
        self.declare_parameter('scale_linear.reverse_x', -0.5)
        self.declare_parameter('scale_linear_turbo.reverse_x', -1)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('enable_turbo_button', 5)
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('scale_angular', 1)
        self.declare_parameter('scale_angular_turbo', 1.5)

        self.axis_linear_x = self.get_parameter('axis_linear.x').value
        self.scale_linear_x = self.get_parameter('scale_linear.x').value
        self.scale_linear_x_turbo = self.get_parameter('scale_linear_turbo.x').value

        self.axis_reverse_linear_x = self.get_parameter('axis_linear.reverse_x').value
        self.scale_reverse_linear_x = self.get_parameter('scale_linear.reverse_x').value
        self.scale_reverse_linear_x_turbo = self.get_parameter('scale_linear_turbo.reverse_x').value

        self.axis_angular = self.get_parameter('axis_angular').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.scale_angular_turbo = self.get_parameter('scale_angular_turbo').value

        self.enable_button = self.get_parameter('enable_button').value
        self.require_enable_button = self.get_parameter('require_enable_button').value
        self.enable_turbo_button = self.get_parameter('enable_turbo_button').value



        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.enabled = False

    def joy_callback(self, msg):
        if self.require_enable_button and msg.buttons[self.enable_button] != 1:
            return

        twist = Twist()
        print(msg.axes[self.axis_linear_x])

        if(msg.buttons[self.enable_turbo_button]):
            scale_x = self.scale_linear_x_turbo
            scale_rev = self.scale_reverse_linear_x_turbo
            scale_ang = self.scale_angular_turbo
        else:
            scale_x = self.scale_linear_x
            scale_rev = self.scale_reverse_linear_x
            scale_ang = self.scale_angular


        forward = (1-(msg.axes[self.axis_linear_x] + 1)/(2)) * scale_x
        backward = (1-(msg.axes[self.axis_reverse_linear_x] + 1)/(2)) * scale_rev

        twist.linear.x = forward if forward > 0 else backward
        twist.angular.z = msg.axes[self.axis_angular] * scale_ang

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
