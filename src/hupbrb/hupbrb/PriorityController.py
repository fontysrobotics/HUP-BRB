from math import cos, sin, pi, sqrt
import rclpy
from rclpy.node import Node
import datetime
import numpy as np

from .collision import EulerToQuat, QuatToEuler, get_collision_coords

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import tf_transformations as tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped, Transform, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from hupbrb_msgs.msg import RobotInformation, Identifier, NoGoCircle, CollisionPoint
from hupbrb_msgs.srv import Handshake

class PriorityController(Node):
    collision_point : PointStamped = None

    def __init__(self):
        super().__init__('priority_controller')
        self.declare_parameter('priority', 3, descriptor=ParameterDescriptor(description = "The priority of this node 1 to 5 (higher has more priority)."))
        
        
        self.info = RobotInformation()
        self.info.name = self.get_namespace()
        self.info.priority = int(self.get_parameter('priority').value)
        self.info.radius = 0.15

        self.robots = dict()
        self.sub = self.subscribe_to_bot(self.info.name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.global_info_publisher = self.create_publisher(RobotInformation, '/robot_info', 10)                          #publisher for projector node
        self.local_info_publisher = self.create_publisher(RobotInformation, 'robot_info', 10)                          #publisher for projector node

        self.global_collision_publisher = self.create_publisher(NoGoCircle, 'global_costmap/collision_point', 10)
        self.local_collision_publisher = self.create_publisher(NoGoCircle, 'local_costmap/collision_point', 10)

        timer_period = 0.5 #0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.id_publishing_timer = self.create_timer(1, self.publish_info)
        
        self.tf_timer = self.create_timer(0.5, self.save_tf)
        self.id_subscriber = self.create_subscription(Identifier, 'scanned_ids', self.scanned_id_callback, 10)     #subscribes to get robot ID
        self.collision_subscriber = self.create_subscription(CollisionPoint, 'collision_point', self.send_collision_to_costmap, 10)     #subscribes collision notifications from other robots

        self.srv = self.create_service(Handshake, 'handshake', self.handshake_accept)              #Service to esteblish connection

    def publish_info(self):
        self.global_info_publisher.publish(self.info)
        self.local_info_publisher.publish(self.info)

    def save_tf(self):
        now = rclpy.time.Time()
        trans : Transform = self.tf_buffer.lookup_transform(
            'base_link',
            'map',
            now).transform

        quat = trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w

        rotateMapToRobot = tf.quaternion_matrix(quat)
        rotateRobotToMap = np.linalg.inv(rotateMapToRobot)
        
        translateMapToRobot = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
        self.rotateMapToRobot = rotateMapToRobot
        self.rotateRobotToMap = rotateRobotToMap

        self.translateMapToRobot = translateMapToRobot

    def check_collision_relevance(self):
        if not self.collision_point: return
        colInWorld = [self.collision_point.x, self.collision_point.y, self.collision_point.z, 0]
        rotated = np.matmul(self.rotateMapToRobot, np.array(colInWorld)) 
        collisionInRobotFrame = self.translateMapToRobot + rotated[:3]

        if collisionInRobotFrame[0] > 0: return

        self.get_logger().info(f"Collision point is behind robot ({collisionInRobotFrame[0]:.2}, {collisionInRobotFrame[1]:.2}). Removing")

        self.collision_point = None
        msg = NoGoCircle()
        msg.enabled = False
        self.global_collision_publisher.publish(msg)

    def timer_callback(self):  
        self.check_collision_relevance()

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

                    if collision == True:
                        break
                        
                    elif collision:
                        try:
                            thisPos, thatPos = collision
                            self.get_logger().info(f"Collision at ({thisPos[0]}, {thisPos[1]}) detected")

                            thisCollision = CollisionPoint()
                            thisCollision.this_point.x, thisCollision.this_point.y = thisPos
                            thisCollision.that_point.x, thisCollision.that_point.y = thatPos
                            thisCollision.robot_info = self.robots[bot]['results']['info']

                            self.sub['publishers']['collision'].publish(thisCollision)

                            
                            thatCollision = CollisionPoint()
                            thatCollision.this_point.x, thatCollision.this_point.y = thatPos
                            thatCollision.that_point.x, thatCollision.that_point.y = thisPos
                            thatCollision.robot_info = self.info

                            self.robots[bot]['publishers']['collision'].publish(thatCollision)
                            
                        except Exception as ex:
                            self.get_logger().error(f"{ex}")
                            raise(ex)

    def send_collision_to_costmap(self, collision: CollisionPoint):
        self.collision_point = collision.this_point

        this_point = np.array([collision.this_point.x, collision.this_point.y, 0, 0]).T

        msg = NoGoCircle()
        msg.enabled = True

        if self.info.priority == collision.robot_info.priority:
            this_point = np.matmul(self.rotateMapToRobot, this_point)
            this_point[1] += 0.4
            this_point = np.matmul(self.rotateRobotToMap, this_point)

            msg.radius = 1.0
            msg.point.x = this_point[0]
            msg.point.y = this_point[1]

        elif self.info.priority < collision.robot_info.priority:
            that_point = np.array([collision.that_point.x, collision.that_point.y, 0, 0]).T

            that_point_in_robot = np.matmul(self.rotateMapToRobot, that_point)
            this_point_in_robot = np.matmul(self.rotateMapToRobot, this_point)
            
            angle = pi/2 if this_point_in_robot[1] > that_point_in_robot[1] else -pi/2

            that_point = that_point - this_point

            msg.point.x = that_point[0] * cos(angle) - that_point[1] * sin(angle) + this_point[0]
            msg.point.y = that_point[0] * sin(angle) + that_point[1] * cos(angle) + this_point[1]

            msg.radius = 1.3
            self.get_logger().info(f"{msg}")
        else:
            return # this robot has right of way

        self.global_collision_publisher.publish(msg)
        # self.local_collision_publisher.publish(msg)

    def subscribe_to_bot(self, name):
        resp = {
            'subscriptions': dict(),
            'publishers' : dict(),
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

        resp['publishers'] = {
            'collision':  self.create_publisher(CollisionPoint, f'{name}/collision_point', 10)
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