#!/usr/bin/python3

__author__ = 'Noah Otte <nvethrandil@gmail.com>'
__version__= '4.0'
__license__= 'MIT'

# Ros libraries
import rospy
# Python libraries
import numpy as np
import time
# Pathplanner libraries
from qfcerrt_noot.src.QFCE_RRT import QFCERRT
from qfcerrt_noot.src.QFCE_RRT_Star import QFCERRTStar
# Ros messages
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, PointStamped

# planner to ROS System bridge
class PlannerBridge():
    """
    A planner-ros-bridge class which handles the transition between ROS messages and the python based pathplanner
    """
    # initialization function
    def __init__(self,
                 run_rrt_star: bool,
                 map_id: str,
                 robotpose_id: str,
                 goal_id: str,
                 path_id: str,
                 traversability_upper_boundary: int,
                 unknown_are: int,
                 safety_buffer: float,
                 iterations: int,
                 stepdistance: int,
                 mode_select: int,
                 danger_zone: int,
                 fov: int
                 ):
        """
        Initialization method for the PlannerBridge class

        Args:
            run_rrt_star (bool):
                A flag to indicate if RRT* shall be run, else regular RRT version
            map_id (str):
                The ROS topic name of the map to plan in
            robotpose_id (str):
                The ROS topic name of the robots position
            goal_id (str):
                The ROS topic name of the goal point to reach
            traversability_upper_boundary (int):
                The highest tolerated traversability score of cells in the map
            unknown_are (int):
                The value to interprete unknown cells in the map as
            safety_buffer_in_meters (float):
                The amount of to inflate each occupied cell in the map
            iterations (int):
                The maximum amount of iterations to let the planner run
            stepdistance (int):
                The root of the minimum cell area tolerated in the Quadtree
            danger_zone (int):
                An integer distance (in pixels) at which detected collisions will trigger replanning
        """
        print("PlannerBridge is waiting for all topics . . .")
        # Placeholders
        self.VERBOSE = False
        self.robo_coords = None
        self.goal_coords = None
        self.height_map = None
        self.mapUGV = None
        self.latest_path = None
        # Subscriber IDs
        self.map_id = map_id
        self.robotpose_id = robotpose_id
        self.goal_id = goal_id
        self.path_id = path_id
        # Planner version
        self.run_rrt_star = run_rrt_star
        # Planning parameters
        self.planner = None
        self.traversability_upper_boundary = traversability_upper_boundary
        self.unknown_are = unknown_are
        self.safety_buffer = safety_buffer
        self.iterations = iterations
        self.step_distance = stepdistance
        self.mode = mode_select
        self.danger_zone = danger_zone
        self.fov = fov
        # Subscribe to relevant topics
        self.subscriber_pose = rospy.Subscriber(self.robotpose_id, Odometry, self.callback_pose)
        self.subscriber_goal = rospy.Subscriber(self.goal_id, PointStamped, self.callback_goal)
        # Planning is initiated by callback of map
        self.subscriber_map = rospy.Subscriber(self.map_id, OccupancyGrid, self.callback_map)
        # Create publisher
        self.publisher = rospy.Publisher(self.path_id, Path, queue_size=1)


    def callback_pose(self, data: Pose):
        """Callback method for the robots position topic subscriber

        Args:
            data (Pose):
                The robots position topic ROS message
        """
        if self.VERBOSE:
            print('recieved posedata of type: "%s' % data.format)

        self.robo_coords = [data.pose.pose.position.x, data.pose.pose.position.y]


    def callback_goal(self, msg: PointStamped):
        """
        Callback method for the goal topic subscriber.

        Args:
            msg (PointStamped):
                The goal point topic ROS message
        """
        if self.VERBOSE:
            print('recieved posedata of type: "%s' % msg.format)

        self.goal_coords = [msg.point.x, msg.point.y]

    # pathplanning is initiated by this callback method
    def callback_map(self, msg: OccupancyGrid):
        """
        Callback method for the occupancy map subscriber. Converts recieved map into numpy array and
        initiates pathplanning on said map, including replanning if needed.

        Args:
            msg (OccupancyGrid):
                The occupancy map ROS message
        """
        if self.VERBOSE:
            print('recieved mapUGV of type: "%s' % msg.format)
        if self.robo_coords and self.goal_coords:
            # convert Occupancy grid into Numpy Array
            mapUGV =  self.occupancygrid_to_numpy(msg)
            trav_bound_upper = self.traversability_upper_boundary
            unknown_are = self.unknown_are
            # everything above upper is 1 == nontraversable
            mapUGV = np.where(mapUGV <= trav_bound_upper, 0, 1)

            self.mapUGV = mapUGV

            start = self.world2map([self.robo_coords[0], self.robo_coords[1]], msg)
            goal = self.world2map([self.goal_coords[0], self.goal_coords[1]], msg)

            # clumsy replanning logic

            # if there has been a path
            if self.planner:
                # and replanning is needed
                if self.planner.need2replan(start, self.mapUGV):
                    print("Replanning was needed.")

                    # publish rover position to make it stop EXPERIMENTAL
                    self.latest_path = [self.robo_coords, self.robo_coords]
                    self.publish_path()

                    # plan a new path
                    if self.plan_path(start, goal, msg):
                        # if a new path was found, publish
                        self.publish_path()
                else:
                    # no replanning was needed, publish old path
                    self.publish_path()
            else:
                # there has not been a path before, plan one
                if self.plan_path(start, goal, msg):
                        self.publish_path()
            '''if self.plan_path(start, goal, msg):
                    self.publish_path()'''
        else:
            time.sleep(0.01)


    def plan_path(self, start: list, goal: list, msg: OccupancyGrid) -> bool:
        """_summary_

        Args:
            start (list):
                The starting point for the planner
            goal (list):
                The goal point to be found
            msg (OccupancyGrid):
                The occupancy map in which to plan in

        Returns:
            bool:
                True if planner returned a path, False if not
        """
        print("Started planning . . .")

        # if RRT* version shall be run
        if self.run_rrt_star:
            self.planner = QFCERRTStar(
                map = self.mapUGV,
                start = start,
                goal = goal,
                max_iterations = self.iterations,
                stepdistance = self.step_distance,
                plot_enabled = False,
                max_neighbour_found = 8,
                bdilation_multiplier = self.safety_buffer,
                cell_sizes= [10, 20],
                mode_select= self.mode,
                danger_zone= self.danger_zone,
                fov = self.fov
                )
            # execute planning
            path = self.planner.search_rrtstar()

        # else regular RRT version shall be run
        else:
            self.planner = QFCERRT(
                map = self.mapUGV,
                start = start,
                goal = goal,
                max_iterations = self.iterations,
                stepdistance = self.step_distance,
                plot_enabled = False,
                max_neighbour_found = 10,
                bdilation_multiplier = self.safety_buffer,
                cell_sizes = [10, 20],
                mode_select = self.mode,
                danger_zone= self.danger_zone,
                fov = self.fov
                )
            # execute planning
            path = self.planner.search()



        if len(path) > 1:
            self.latest_path =  self.planner2world(path, msg)
            return True
        else:
            return False


    def publish_path(self):
        """
        Publishes the path contained in self.latest_path as a path ROS message
        """
        path2publish = self.latest_path

        # exclude an empty path in edge-case errors
        if not path2publish:
            pass
        else:
            msg = Path()
            msg.header.seq = 0
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

            #rospy.loginfo(msg)
            self.publisher.publish(msg)


    def world2map(self, point: list, msg: OccupancyGrid) -> list:
        """
        Convert a given single point from world-frame to map-frame

        Args:
            point (list):
                The point to convert from world-frame
            msg (OccupancyGrid):
                The occupancy map which the point is in

        Returns:
            converted_point (list):
                The point converted into map-frame
        """
        resolution = msg.info.resolution
        x = round((point[0] - msg.info.origin.position.x) / resolution, 3)
        y = round((point[1] - msg.info.origin.position.y) / resolution, 3)
        converted_point = [x, y]

        return converted_point


    def map2world(self, point: list, msg: OccupancyGrid) -> list:
        """
        Convert a given single point from map-frame to world-frame

        Args:
            point (list):
                The point to convert from map-frame
            msg (OccupancyGrid):
                The occupancy map which the point is in

        Returns:
            converted_point (list):
                The point converted into world-frame
        """
        resolution = msg.info.resolution
        x = round((point[0] * resolution + msg.info.origin.position.x), 3)
        y = round((point[1] * resolution + msg.info.origin.position.y), 3)
        converted_point = [x, y]

        return converted_point


    def planner2world(self, path: list, occmap: OccupancyGrid) -> list:
        """
        Convert a list of points, a path, from map-frame to world-frame

        Args:
            path (list):
                The path to convert in map-frame
            occmap (OccupancyGrid):
                The occupancy map in which the path was planned

        Returns:
            world_path (list):
                The path converted into world-frame
        """
        world_path = []
        for p in path:
                a = self.map2world([p[0], p[1]], occmap)
                world_path.append(a)

        return world_path


    def occupancygrid_to_numpy(self, msg: OccupancyGrid) -> np.ma.array:
        """
        Convert an occupancygrid ROS message into numpy array.

        Args:
            msg (OccupancyGrid):
                The occupancy grid to convert

        Returns:
            np.ma.array:
                The numpy array equivalent of the occupancy grid
        """
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        numpy_data = np.ma.array(data)
        return numpy_data

# main loop executed upon call
def main():
    """
    Main method for executing the QFCE-RRT pathplanner in a live ROS environment
    """
    try:
        # initialize ROS node
        planner_node_str = 'main_planner_node'
        rospy.init_node(planner_node_str, anonymous=False)

        # topic IDs
        map_id= "/mapUGV"
        robotpose_id = "/robot/dlio/odom_node/odom"
        goal_id = "/clicked_point"
        path_id = "path_pub"
        # planner variant (RRT or RRT*)
        run_rrt_star = True
        # planner settings
        traversability_upper_boundary = 90
        unknown_are = 0
        safety_buffer_pixels = 4
        iterations = 750
        stepdistance = 1
        mode = 2
        danger_zone = 50
        fov = 90
        # print settings in terminal
        print(planner_node_str + f' initialized with settings:' +
              f'\nMap ID: ' + map_id +
              f'\nRobotpose ID: ' + robotpose_id +
              f'\nGoal ID: ' + goal_id +
              f'\nPath ID: ' + path_id +
              f'\n--\n' +
              f'Run RRT* is: {run_rrt_star}' +
              f'\nDanger zone distance is {danger_zone} pixels. \n' +
              f'Cells above {traversability_upper_boundary} are non-traversable.\n' +
              f'Unknown cells are interpreted as {unknown_are}. \n' +
              f'Inflate objects by {safety_buffer_pixels} pixels.\n' +
              f'Run for a maximum {iterations} iterations.\n' +
              f'Minimum Quadtree cells are {stepdistance} x {stepdistance} pixels.\n' +
              f'The selected post-processing mode was {mode}. \n' +
              f'FOV is {fov} degrees.'
              )

        # initialize planner-ros-bridge
        planner = PlannerBridge(run_rrt_star = run_rrt_star,
                                map_id = map_id,
                                robotpose_id = robotpose_id,
                                goal_id = goal_id,
                                path_id = path_id,
                                traversability_upper_boundary = traversability_upper_boundary,
                                unknown_are = unknown_are,
                                safety_buffer = safety_buffer_pixels,
                                iterations = iterations,
                                stepdistance = stepdistance,
                                mode_select= mode,
                                danger_zone= danger_zone,
                                fov=fov
                                )

        # keep the ROS node running
        rospy.spin()

    except KeyboardInterrupt:
        print("--- Shutting down ROS planner main node ---")

    except Exception as e:
        print("--- main() execution failed ---")
        print(e)

# ensure main() is run upon script execution
if __name__ == '__main__':
    main()
