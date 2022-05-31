import rclpy
from rclpy.node import Node
import os

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from hupbrb_msgs.msg import RobotInformation, Identifier
from hupbrb_msgs.srv import Handshake

class PriorityController(Node):

    def __init__(self):
        super().__init__('priority_controller')
        self.info = RobotInformation()
        self.info.name = self.get_namespace()
        self.info.priority = 0
        self.info.radius = 0.15
        self.robots = dict()

        self.sub = self.subscribe_to_bot(self.info.name)

        self.id_publisher = self.create_publisher(RobotInformation, 'robot_info', 10)                          #publisher for projector node

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.id_subscriber = self.create_subscription(Identifier, 'scanned_ids', self.scanned_id_callback, 10)     #subscribes to get robot ID
        self.srv = self.create_service(Handshake, 'handshake', self.handshake_accept)              #Service to esteblish connection

    def timer_callback(self):
        self.id_publisher.publish(self.info)
        
        for bot in self.robots:
            if not "subscriptions" in self.robots[bot]:             
                self.robots[bot] = self.subscribe_to_bot(bot)

            if self.robots[bot]['results'].items():
                if "odom" in self.robots[bot]['results'] and "odom" in self.sub['results']:
                    msg1 : Odometry = self.sub['results']['odom']
                    msg2 : Odometry = self.robots[bot]['results']['odom']
                    x1 = msg1.pose.pose.position.x
                    y1 = msg1.pose.pose.position.y
                    x2 = msg2.pose.pose.position.x
                    y2 = msg2.pose.pose.position.y

                    dist = (x2-x1)**2 + (y2-y1)**2

                    self.get_logger().info(str(dist))

                    if dist > 10.0:
                        self.get_logger().info(f"Distance = {dist} > 10.0\nUnsubscribing...")
                        self.kill_bot(bot)
                        break
                # self.get_logger().info(str(bot['results']))

    def subscribe_to_bot(self, name):
        resp = {
            'subscriptions': dict(),
            'results': dict()
        }
        def add_result(sub, msg):
            # if name not in robots: robots[name] = {'results': dict()}
            resp['results'][sub] = msg
        resp['subscriptions'] ={
                    'plan': self.create_subscription(Path, f"{name}/plan", lambda m: add_result('plan', m), 10),
                    'odom': self.create_subscription(Odometry, f"{name}/odom", lambda m: add_result('odom', m), 10),
                    'cmd_vel': self.create_subscription(Twist, f"{name}/cmd_vel", lambda m: add_result('cmd_vel', m), 10),
                    'info': self.create_subscription(RobotInformation, f"{name}/robot_info", lambda m: add_result('info', m), 10),
                }
        return resp

    def kill_bot(self, bot_name):
        for sub, obj in self.robots[bot_name]['subscriptions'].items():
            self.destroy_subscription(obj)

        del self.robots[bot_name]
        
            


    def scanned_id_callback(self, msg: Identifier):
        self.get_logger().info(f"Initializing communication between {self.info.name} (local) and {msg.name} (remote)")
        self.cli = self.create_client(Handshake, f"{msg.name}/handshake")                                          #Maybe make this part a seperate node???
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = Handshake.Request()
        req.info = self.info
        future = self.cli.call_async(req)
        future.add_done_callback(self.handshake_callback)


    def handshake_accept(self, request, response):
        self.robots[request.info.name] = {'info': request.info} #save the incoming request
        self.get_logger().info(f"Communication initialized between {self.info.name} (local) and {request.info.name} (remote)")
        
        response.info = self.info
        return response
        

    def handshake_callback(self, future):
        self.robots[future.result().info.name] = {'info': future.result().info} #save the request response


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