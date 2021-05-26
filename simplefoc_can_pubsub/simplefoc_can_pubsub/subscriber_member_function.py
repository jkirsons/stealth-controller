import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .can_handler import CanHandler

import can # https://python-can.readthedocs.io
import struct

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'motor_angle',
            self.ros_callback,
            10)
        self.subscription

        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        self.handler = CanHandler(self.bus)
        notifier = can.Notifier(self.bus, [ self.canReceived ])

    def ros_callback(self, msg):
        #self.get_logger().info('Angle: "%s"' % msg.angular.z)
        ba = bytearray(struct.pack("f", msg.angular.z))
        msg = can.Message(arbitration_id=0x01001001, data=ba, is_extended_id=True)
        try:
            self.bus.send(msg)
        except can.CanError:
            self.get_logger().info("CAN message NOT sent")

    def canReceived(self, msg):
        self.handler.process(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
