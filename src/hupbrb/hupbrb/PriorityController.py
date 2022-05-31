import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from hupbrb_msgs.msg import Identifier, Priority, RobotInformation
from hupbrb_msgs.srv import Handshake


class PriorityController(Node):

    def __init__(self):
        super().__init__('priority_controller')
        self.priority = 0
        self.id = 0
        self.robots = []                                                                                        #List of robot id's and priorities

        self.id_publisher = self.create_publisher(RobotInformation, '/robot_info', 10)                          #publisher for projector node

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.declare_parameter('id', descriptor=ParameterDescriptor(
            description="The identifier of the current robot, used in its namespace and QR code."))

        self.id_subscriber = self.create_subscription(Identifier, '/scanned_ids', self.scanned_id_callback)   #subscribes to get robot ID
        self.srv = self.create_service(Handshake, self.id + '/handshake', self.handshake_callback)              #Service to esteblish connection

    
    def timer_callback(self):
        self.publisher_.publish(self.id)


    def scanned_id_callback(self, msg: Identifier):
        newId = msg.id
        self.cli = self.create_client(Handshake, newId + '/handshake')                                          #Maybe make this part a seperate node???
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Handshake.Request()
        self.send_request()


    def handshake_callback(self, request, response):
        response.priority = self.priority
        robot = []
        robot.append(request.id)
        robot.append(request.priority)
        self.robots.append(robot)
        return response

    def send_request(self):
        self.req.id = self.id
        self.req.priority = self.priority
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    priority_controller = PriorityController()

    rclpy.spin(priority_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    priority_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()