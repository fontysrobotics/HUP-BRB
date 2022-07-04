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

class CriticalPointController(Node):
    robots = {}

    def __init__(self):
        super().__init__('critical_point_controller')

        self.declare_parameter('radius', value=1, descriptor=ParameterDescriptor(description = "The radius of the circle representing the critical zone"))
        self.declare_parameter('posX')
        self.declare_parameter('posY')

        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.posX = self.get_parameter('posX').get_parameter_value().double_value
        self.posY = self.get_parameter('posY').get_parameter_value().double_value
        self.get_logger().info(f"Starting critical point at position ({self.posX}, {self.posY}) with radius {self.radius}")
        self.robotinformation_subscription = self.create_subscription(RobotInformation, '/robot_info', self.robot_info_callback, 10)
        # self.drawer_timer = self.create_timer(0.5, self.tick)
    
    def robot_info_callback(self, msg: RobotInformation):
        robot = msg
        time_of_msg = self.get_clock().now()
        
 
        if not robot.name in self.robots:
            global_path_sub = self.create_subscription(Path, robot.name+'/plan', lambda msg: self.save_info(robot, 'plan', msg), 10)
            collision_sub = self.create_subscription(CollisionPoint, robot.name+'/collision_point', lambda msg: self.handle_collision(robot, msg), 10)
            
            local_no_go_pub = self.create_publisher(NoGoCircle, robot.name+'/local_costmap/collision_point', 10)
            global_no_go_pub = self.create_publisher(NoGoCircle, robot.name+'/global_costmap/collision_point', 10)

            self.robots[robot.name] = {
                "info": robot,
                "global_path_sub": global_path_sub,
                "collision_sub": collision_sub,
                "local_no_go_pub": local_no_go_pub,
                "global_no_go_pub": global_no_go_pub,
                }  
    
        self.robots[robot.name]["time"] = time_of_msg.to_msg()
    
    def save_info(self, robot: RobotInformation, property: str, msg: Path):
        self.robots[robot.name][property] = msg.poses
        
    def handle_collision(self, thisRobot: RobotInformation, msg: CollisionPoint):
        if not msg.this_point.x or not msg.this_point.y:
            self.get_logger().warn('Collision detected without X and Y coordinates. Ignoring')
            return

        if ((msg.this_point.x - self.posX) ** 2 + (msg.this_point.y - self.posY) ** 2 > self.radius ** 2):
            return

        otherRobot = msg.robot_info

        low_prio = self.robots[thisRobot.name] if thisRobot.priority < otherRobot.priority else self.robots[otherRobot.name]
        msg = NoGoCircle()

        msg.point.x = self.posX
        msg.point.y = self.posY
        msg.radius = self.radius * 1.2

        low_prio['local_no_go_pub'].publish(msg)

        collision = {
            'robot': low_prio,
        }

        timer = self.create_timer(1, lambda : self.remove_costmap(collision))
        collision['timer'] = timer
        
    def remove_costmap(self, collision: dict):
        if 'timer' in collision:
            collision['timer'].cancel()
        msg = NoGoCircle()
        msg.enabled = False

        collision['robot']['local_no_go_pub'].publish(msg)
        collision['robot']['global_no_go_pub'].publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CriticalPointController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()