import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy import interpolate
from scipy.spatial.transform import Rotation

from nav_msgs.msg import Path, OccupancyGrid, Odometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.path_subscription = self.create_subscription(Path, '/plan', self.listener_callback, 10)
        self.path_subscription  # prevent unused variable warning
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_subscription
        self.robotlocation_subscription = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.robotlocation_subscription
        
        self.RobotLocation = None
        self.RobotOrientation = None


    def listener_callback(self, msg: Path):
        window_x = 1280
        window_y = 1024

        NUM_VERT = 4
        frame = np.full((window_y, window_x, 3),0).astype(np.uint8)

        #robot specifications:
        robot_diameter = 0.178                  #meters
        robot_radius = robot_diameter/2         #meters

        #enviroment specifications:
        map_x, map_y = 3, 3                     #meters
        
        # cv2.rectangle(frame, (138, 10), (886, 758), (255, 255, 255), 10)
        offset = int((window_x-window_y)/2)
        pt1 = (offset, 0)
        pt2 = ((pt1[0]+window_y), window_y)
        cv2.rectangle(frame, pt1, pt2, (255, 0, 0), 3)
        points = []
        empty = []
        empty_array = np.array(empty)

        for poseStamped in msg.poses:
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
        cv2.circle(frame, (center_x, center_y), circle_radius, (0, 0, 255), 2)

        cv2.polylines(frame, [out], isClosed = False, color = (255, 0, 0), thickness = 2)
        cv2.polylines(frame, [points], isClosed = False, color = (0, 0, 255), thickness = 1)

        if self.RobotLocation:
            pos_x, pos_y = self.RobotLocation
            cv2.circle(frame, (pos_x, pos_y), circle_radius, (0, 255, 0), 3)
            robot_orientation_array = self.RobotOrientation.as_euler('zxy', degrees=False)
            robot_orientation_z = robot_orientation_array[0]+1.57
            add_x = int(np.sin(robot_orientation_z)*circle_radius)
            add_y = int(np.cos(robot_orientation_z)*circle_radius)
            cv2.line(frame, (pos_x, pos_y), (pos_x+add_x,pos_y+add_y), (0, 150, 0), 2)
        
        

        cv2.namedWindow('map', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('map', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        cv2.imshow("map", frame)
        cv2.waitKey(1)

    def map_callback(self, msg: OccupancyGrid):
        print('Frame:')
        print('Height:', msg.info.height)
        print('Width:', msg.info.width)
        print('')
        print('Orientation:')
        print('X:', msg.info.origin._position._x)
        print('Y:', msg.info.origin._position._y)
        print('X:', msg.info.origin._position._z)
    
    def location_callback(self, msg: Odometry):
        self.RobotLocation = scale_coordinates(msg.pose.pose.position.x, msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        self.RobotOrientation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        
def scale_coordinates(x, y):
    window_x = 1280
    window_y = 1024
    offset = int((window_x-window_y)/2)
    x = int(((x/3)*1024)+offset)
    y = int(((3-y)/3)*1024)
    return x,y

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()