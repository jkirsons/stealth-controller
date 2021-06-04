from simplefoc_interfaces.srv import GetValue
from simplefoc_interfaces.srv import SetFloat
from simplefoc_interfaces.srv import SetDouble

import rclpy
from rclpy.node import Node

from .can_handler import CanHandler
import can # https://python-can.readthedocs.io
import struct

class MinimalService(Node):

  def __init__(self):
    super().__init__('minimal_service')
    #self.srv = self.create_service(GetValue, 'get_value', self.service_callback)
    self.srvFloat = self.create_service(SetFloat, 'set_float', self.service_callback_setFloat)
    self.srvDouble = self.create_service(SetDouble, 'set_double', self.service_callback_setDouble)

    self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
    self.handler = CanHandler(self.bus)
    notifier = can.Notifier(self.bus, [ self.canReceived ])

  def canReceived(self, msg):
      self.handler.process(msg)

  # https://docs.python.org/3/library/struct.html#format-characters
  def service_callback_setFloat(self, request, response):
    ba = bytearray(struct.pack("f", request.value)) 
    self.handler.send_can_message(CanHandler.data_types["float"], request.command, request.motor_id, request.bus_id, ba)
    return response

  def service_callback_setDouble(self, request, response):
    ba = bytearray(struct.pack("d", request.value)) 
    self.handler.send_can_message(CanHandler.data_type["double"], request.command, request.motor_id, request.bus_id, ba)
    return response

def main():
  rclpy.init()
  minimal_service = MinimalService()
  rclpy.spin(minimal_service)
  rclpy.shutdown()


if __name__ == '__main__':
  main()