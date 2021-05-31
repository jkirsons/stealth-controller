import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist

# import can
# https://python-can.readthedocs.io/en/master/listeners.html
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'motor_angle', 10)
        timer_period = 0.01  # in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.f = 0.0
#        my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')
#        self.declare_parameter('angle', 'world', my_parameter_descriptor)

    def timer_callback(self):
#        my_param = self.get_parameter('angle').get_parameter_value().string_value
        msg = Twist()
        msg.angular.z = math.sin(self.f) * math.pi * 2
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.angular.z)
        self.f += 0.01

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()