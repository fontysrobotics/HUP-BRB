from turtle import color
from matplotlib import colors
from matplotlib.pyplot import draw
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy import interpolate
from scipy.spatial.transform import Rotation

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from hupbrb_msgs.msg import RobotInformation

window_x = 1024
window_y = 768
map_x, map_y = 3.0, 2.25                     #meters

class PathProjection(Node):

    def __init__(self):
        super().__init__('path_projection')

        self.robotinformation_subscription = self.create_subscription(RobotInformation, '/robot_info', self.robot_info_callback, 10)
        self.alive_checker_timer = self.create_timer(5, self.alive_check)
        
        self.RobotLocation = None
        self.RobotOrientation = None
        self.robots = {}
        self.color = {
        # "red": (0, 0, 255),
        "yellow": (128, 255, 255),
        # "green": (0, 255, 0),
        "lightBlue": (255, 255, 128),
        # "blue": (255, 0, 0),
        "purple": (255, 128, 255)}

    def alive_check(self):
        
        pass


    def robot_info_callback(self, msg: RobotInformation):
        robot = RobotInformation()
        robot.name = msg.name
        robot.priority = msg.priority
        robot.radius = msg.radius
 
        if not robot.name in self.robots:
            path_subscription = self.create_subscription(Path, robot.name+'/plan', lambda msg: self.save_plan(robot, msg), 10)
            location_subscription = self.create_subscription(Odometry, robot.name+'/odom', self.location_callback, 10)
            index = len(self.robots) 
            self.robots[robot.name] = {"info": robot, "path_subscription": path_subscription, "location_subscription": location_subscription, "color": list(self.color.values())[index]} 

        
        self.draw_lines()  
    

    def save_plan(self, robot: RobotInformation, msg: Path):
        self.robots[robot.name]["path"] = msg.poses
        print(robot.name)

    def draw_lines(self):
        NUM_VERT = 4
        frame = np.full((window_y, window_x, 3),0).astype(np.uint8)

        #robot specifications:
        robot_diameter = 0.178                  #meters
        robot_radius = robot_diameter/2         #meters

        #enviroment specifications:
        # cv2.rectangle(frame, (138, 10), (886, 758), (255, 255, 255), 10)
        offset = 0
        pt1 = (offset, 0)
        pt2 = ((pt1[0]+window_x), window_y)
        cv2.rectangle(frame, pt1, pt2, (255, 0, 0), 3)

        for x in self.robots:
            points = []
            empty = []
            empty_array = np.array(empty)
            if 'path' in self.robots[x].keys():
                for poseStamped in self.robots[x]["path"]:
                    coordinates = scale_coordinates(poseStamped.pose.position.x, poseStamped.pose.position.y)
                    points.append(coordinates)


                points = np.array(points)

                if len(points) >= 5:
                    mask = np.int32(np.floor(np.arange(0, NUM_VERT+1, 1)*((len(points)-1)/NUM_VERT)))

                    tck, _ = interpolate.splprep(points[mask].T, s=0)
                    unew = np.arange(0, 1, 0.01)
                    out = np.array(interpolate.splev(unew, tck), dtype=np.int32).T
                else:
                    out = empty_array

                last_x, last_y = points[-1]
                circle_radius = int((robot_radius/map_y)*window_y)
                center_x = int(last_x)
                center_y = int(last_y)
                cv2.circle(frame, (center_x, center_y), circle_radius, self.robots[x]["color"], 2)
                cv2.polylines(frame, [out], isClosed = False, color = self.robots[x]["color"], thickness = 100
                #cv2.polylines(frame, [points], isClosed = False, color = (0, 0, 255), thickness = 1)

                """if self.RobotLocation:
                    pos_x, pos_y = self.RobotLocation
                    #cv2.circle(frame, (pos_x, pos_y), circle_radius, (0, 255, 0), 3)
                    robot_orientation_array = self.RobotOrientation.as_euler('zxy', degrees=False)
                    robot_orientation_z = robot_orientation_array[0]+1.57
                    add_x = int(np.sin(robot_orientation_z)*circle_radius)
                    add_y = int(np.cos(robot_orientation_z)*circle_radius)
                    cv2.line(frame, (pos_x, pos_y), (pos_x+add_x,pos_y+add_y), (0, 150, 0), 2) """

        cv2.namedWindow('map', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('map', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("map", frame)
        cv2.waitKey(1)

    def location_callback(self, msg: Odometry):
        self.RobotLocation = scale_coordinates(msg.pose.pose.position.x, msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.RobotOrientation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        
def scale_coordinates(x, y):
    x = int((x * window_x / map_x))
    y = int((map_y-y) * window_y / map_y)
    return x,y

def main(args=None):
    rclpy.init(args=args)

    path_projection_node = PathProjection()

    rclpy.spin(path_projection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_projection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()