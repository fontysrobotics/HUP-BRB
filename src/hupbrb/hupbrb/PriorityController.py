import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor


class PriorityController(Node):

    def __init__(self):
        super().__init__('priority_controller')
        self.declare_parameter('id', descriptor=ParameterDescriptor(
            description="The identifier of the current robot, used in its namespace and QR code."))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PriorityController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()