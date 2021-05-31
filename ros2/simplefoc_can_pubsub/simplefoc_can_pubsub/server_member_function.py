from simplefoc_interfaces.srv import GetValue

import rclpy
from rclpy.node import Node

class MinimalService(Node):

  def __init__(self):
    super().__init__('minimal_service')
    self.srv = self.create_service(GetValue, 'get_value', self.service_callback)

  def service_callback(self, request, response):
    #response.sum = request.a + request.b
    #self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    pass

    return response

def main():
  rclpy.init()
  minimal_service = MinimalService()
  rclpy.spin(minimal_service)
  rclpy.shutdown()


if __name__ == '__main__':
  main()