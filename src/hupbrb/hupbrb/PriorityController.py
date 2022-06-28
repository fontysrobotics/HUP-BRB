from math import cos, sin, pi, sqrt
import rclpy
from rclpy.node import Node
import os
# import numpy as np

from .collision import get_collision_coords

# from tf2_ros import TransformException
# from tf2_ros.transform_listener import TransformListener
# from tf2_ros.buffer import Buffer

# import tf_transformations as tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from hupbrb_msgs.msg import RobotInformation, Identifier, Collision
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
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.id_publisher = self.create_publisher(RobotInformation, '/robot_info', 10)                          #publisher for projector node
        self.global_collision_publisher = self.create_publisher(Collision, 'global_costmap/collision_point', 10)
        self.local_collision_publisher = self.create_publisher(Collision, 'local_costmap/collision_point', 10)

        timer_period = 0.5#0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.id_subscriber = self.create_subscription(Identifier, 'scanned_ids', self.scanned_id_callback, 10)     #subscribes to get robot ID
        self.srv = self.create_service(Handshake, 'handshake', self.handshake_accept)              #Service to esteblish connection

    def timer_callback(self):
        self.id_publisher.publish(self.info)
        print(self.info)
        
        for bot in self.robots:
            if not "subscriptions" in self.robots[bot]:             
                self.robots[bot] = self.subscribe_to_bot(bot)

            available = lambda sub: (sub in self.robots[bot]['results'] and sub in self.sub['results'])

            if self.robots[bot]['results'].items():
                if available("odom"):
                    msg1 : Odometry = self.sub['results']['odom']
                    msg2 : Odometry = self.robots[bot]['results']['odom']
                    x1 = msg1.pose.pose.position.x
                    y1 = msg1.pose.pose.position.y
                    x2 = msg2.pose.pose.position.x
                    y2 = msg2.pose.pose.position.y

                    dist = sqrt((x2-x1)**2 + (y2-y1)**2)

                    # self.get_logger().info(str(dist))

                    if False:#dist > inf:
                        self.get_logger().info(f"Distance = {dist} > 10.0\nUnsubscribing...")
                        self.kill_bot(bot)
                        break

                if available("plan") and available("odom"):
                    x1 = [pose.pose.position.x for pose in self.sub['results']['plan'].poses]
                    y1 = [pose.pose.position.y for pose in self.sub['results']['plan'].poses]
                    x2 = [pose.pose.position.x for pose in self.robots[bot]['results']['plan'].poses]
                    y2 = [pose.pose.position.y for pose in self.robots[bot]['results']['plan'].poses]

                    v1 = self.sub['results']['odom'].twist.twist.linear
                    v2 = self.robots[bot]['results']['odom'].twist.twist.linear
                    

                    collision = get_collision_coords(x1, y1, v1, x2, y2, v2, 0.35)

                    # self.get_logger().info(f"Collision: {collision}")

                    msg = Collision()
                    msg.enabled = True


                    if collision == True:
                        #Connection should be killed
                        # self.get_logger().info(f"Unsubscribing...\n")
                        # msg.enabled = False
                        # self.kill_bot(bot)
                        # self.collision_publisher.publish(msg)
                        break
                        
                    elif collision:
                        try:
                            x, y, angle = collision

                            self.get_logger().info(f"Collision at ({x}, {y}) detected!\nAngle: {angle*180/pi}")
                            
                            # now = rclpy.time.Time()
                            # trans : Quaternion = self.tf_buffer.lookup_transform(
                            #     'base_link',
                            #     'map',
                            #     now).transform.rotation
                            # quat = trans.x, trans.y, trans.z, trans.w
                            # mat = tf.quaternion_matrix(quat)


                            # self.get_logger().info(f"Rotation matrix: ({mat})")

                            # coord = np.array([x, y, 0, 0]).T
                            # self.get_logger().info(f"Coord <map>: ({coord})")
                            # coord = np.matmul(mat, coord)
                            # self.get_logger().info(f"Coord <base_link>: ({coord})")
                            # coord[1] += 0.4
                            # self.get_logger().info(f"Coord <base_link>: ({coord})")
                            # coord = np.matmul(np.linalg.inv(mat), coord)
                            # self.get_logger().info(f"Coord <map>: ({coord})")
                            msg.radius = 1.0
                            msg.enabled = True
                            msg.point.x = x + 0.8*msg.radius*cos(pi/2 + angle)
                            msg.point.y = y + 0.8*msg.radius*sin(pi/2 + angle)
                            self.global_collision_publisher.publish(msg)
                            self.local_collision_publisher.publish(msg)

                            
                        except TransformException as ex:
                            self.get_logger().info(
                                f'Could not transform <map> to <base_link>: {ex}')
                        except Exception as ex:
                            self.get_logger().error(f"{ex}")
                            raise(ex)
                        
                    else:
                        # msg.enabled = False
                        # no collision detected yet, possible in future..
                        pass


                    # if not hasattr(self, 'plt'):
                    #     self.plt = Graph(self.info.name)
                    #     self.plt.update(x1, y1, v1, x2, y2, v2)
                    #     # self.plt.rescale((0,0))
                    # else:
                    #     self.plt.update(x1, y1, v1, x2, y2, v2)
                    #     # self.plt.rescale((0,0))

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
        if msg.name in self.robots:
            self.get_logger().debug(f"Connection already established between {self.info.name} (local) and {msg.name} (remote)")
            return
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