#!/usr/bin/python3

__author__ = 'Noah Otte <nvethrandil@gmail.com>'
__version__= '0.1'
__license__= 'MIT'

# Ros libraries
import rospy
# Python libraries
import numpy as np
import time
# My libraries
from qfcerrt_noot.src.QFCE_RRT import QFCERRT as Planner
# Ros messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, PointStamped

# Planner to ROS System bridge
class PlannerBridge():
    def __init__(self, 
                 map_id: str, 
                 robotpose_id: str,
                 goal_id: str,
                 traversability_upper_boundary: int, 
                 unknown_are: int,
                 safety_buffer_in_meters: float,
                 iterations: int,
                 stepdistance: int
                 ):
        
        print("Initializing planner . . .")
        # Placeholders
        self.VERBOSE = False
        self.robo_coords = None
        self.goal_coords = None
        self.height_map = None
        self.mapUGV = None
        self.raw_occmap = None
        self.latest_path = None
        # Subscriber IDs
        self.map_id = map_id
        self.robotpose_id = robotpose_id
        self.goal_id = goal_id
        # Planning parameters
        self.planner = None
        self.traversability_upper_boundary = traversability_upper_boundary
        self.unknown_are = unknown_are
        self.safety_buffer = safety_buffer_in_meters
        self.iterations = iterations
        self.step_distance = stepdistance
        
        # Subscribe to relevant topics
        self.subscriber_pose = rospy.Subscriber(self.robotpose_id, Odometry, self.callback_pose)
        self.subscriber_goal = rospy.Subscriber(self.goal_id, PointStamped, self.callback_goal)
        # Planning is initiated by callback of map
        self.subscriber_map = rospy.Subscriber(self.map_id, OccupancyGrid, self.callback_map)
        # Create publisher
        self.publisher = rospy.Publisher('path_pub', Path, queue_size=1)
        
    def callback_pose(self, data: Pose):
        if self.VERBOSE:
            print('recieved posedata of type: "%s' % data.format)
        #print("callback_pose")    
        self.robo_coords = [data.pose.pose.position.x, data.pose.pose.position.y]
    
    def callback_goal(self, msg: PointStamped):
        if self.VERBOSE:
            print('recieved posedata of type: "%s' % msg.format)
        #print("callback_goal")     
        self.goal_coords = [msg.point.x, msg.point.y]    
    # This is where the magic happens    
    def callback_map(self, msg: OccupancyGrid):
            if self.VERBOSE:
                print('recieved mapUGV of type: "%s' % msg.format)
            if self.robo_coords and self.goal_coords:
                #print("callback_map") 
                # Convert Occupancy grid into Numpy Array
                mapUGV =  self.occupancygrid_to_numpy(msg)
                trav_bound_upper = self.traversability_upper_boundary
                unknown_are = self.unknown_are
                # everything above upper is 1 == nontraversable
                mapUGV = np.where(mapUGV >= trav_bound_upper, 1, mapUGV) 
                # unkown pixels are set to unknown_are value
                mapUGV = np.where(mapUGV == -1, unknown_are, mapUGV) 
                mapUGV = np.where(mapUGV > 1, 1, mapUGV)
                
                self.mapUGV = mapUGV
                self.raw_occmap = msg
                
                start = self.world2map([self.robo_coords[0], self.robo_coords[1]], msg) 
                #goal = self.goal_coords
                goal = self.world2map([self.goal_coords[0], self.goal_coords[1]], msg)
                
                # Clumsy way of handling replanning
                if self.planner:
                    if self.planner.need2replan(start, self.mapUGV):
                        if self.plan_path(start, goal, msg):
                            self.publish_path()
                    else:
                        self.publish_path()
                else:
                    if self.plan_path(start, goal, msg):
                            self.publish_path()
                '''if self.plan_path(start, goal, msg):
                        self.publish_path()'''
            else:
                time.sleep(0.01)
         
    def plan_path(self, start, goal, msg):
        print("Started planning . . .")     
        self.planner = Planner(
            map = self.mapUGV,
            start = start,
            goal = goal,
            max_iterations = self.iterations,
            stepdistance = self.step_distance,
            plot_enabled = False,
            search_radius_increment = 0.5,
            max_neighbour_found = 8,
            bdilation_multiplier = self.safety_buffer,
            cell_sizes= [10, 20])
        path = self.planner.search()
        if len(path) > 1:
            self.latest_path =  self.planner2world(path, msg)
            return True
        else:
            return False
    
    def publish_path(self):
        path2publish = self.latest_path
        msg = Path()
        msg.header.seq = 0 #??
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        
        for i in range(len(path2publish)):
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = i
            pose_stamped.header.frame_id = "/world_frame"
            pose_stamped.header.stamp = rospy.Time.now()
            
            [x, y] = path2publish[i]
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            # 2D path -> z = 0
            pose_stamped.pose.position.z = 0
            
            msg.poses.append(pose_stamped) 
                
        rospy.loginfo(msg)
        self.publisher.publish(msg) 

    def world2map(self, point, msg):
            resolution = msg.info.resolution
            x = round((point[0] - msg.info.origin.position.x) / resolution, 3)
            y = round((point[1] - msg.info.origin.position.y) / resolution, 3)
            return [x, y]

    def map2world(self, point, msg):
        resolution = msg.info.resolution
        x = round((point[0] * resolution + msg.info.origin.position.x), 3)
        y = round((point[1] * resolution + msg.info.origin.position.y), 3)
        return [x, y]

    def planner2world(self, path, occmap):
            world_path = []
            for p in path:
                    a = self.map2world([p[0], p[1]], occmap)
                    world_path.append(a)
            return world_path

    def occupancygrid_to_numpy(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return np.ma.array(data)
 
# Main loop executed upon call
def main():
    try:
        # Initialize ROS node
        rospy.init_node('main_planner_node', anonymous=False)
        # Topic IDs
        map_id= "/mapUGV"
        robotpose_id = "/robot/dlio/odom_node/odom"
        goal_id = "/clicked_point"
        # Planner settings
        traversability_upper_boundary = 20
        unknown_are = 0
        safety_buffer_in_meters = 4 #pixels
        iterations = 500
        stepdistance = 1
        # Initialize planner-ros-bridge
        planner = PlannerBridge(map_id, robotpose_id, goal_id, traversability_upper_boundary, unknown_are, safety_buffer_in_meters, iterations, stepdistance)
        rospy.spin()
        
    except KeyboardInterrupt:
        print(" Shutting down ROS planner main node")
        
    except Exception as e:
        print(e)
        
if __name__ == '__main__':
    main()