#!/usr/bin/python3

__author__ = 'Noah Otte <nvethrandil@gmail.com>'
__version__= '0.1'
__license__= 'MIT'

from qfcerrt_noot.planner_bridge import PlannerROSBridge
import rospy

# Main loop executed upon call
def main():
    try:
        # Initialize ROS node
        rospy.init_node('main_planner_node', anonymous=False)
        
        # Topic IDs and planner settings
        map_id= "/mapUGV"
        robotpose_id = "/robot/dlio/odom_node/odom"
        goal_id = "/clicked_point"
        traversability_upper_boundary = 60
        unknown_are = 0
        safety_buffer_in_meters = 1 #pixels
        iterations = 500
        stepdistance = 1
        
        # Initialize planner-ros-bridge
        print("Initializing planner . . .")
        planner = PlannerROSBridge(map_id, 
                                   robotpose_id, 
                                   goal_id, 
                                   traversability_upper_boundary, 
                                   unknown_are, 
                                   safety_buffer_in_meters, 
                                   iterations,
                                   stepdistance
                                   )
        # Wait for all necessary topics to arrive
        planner.wait_for_topics()
        # Try to plan path
        if planner.subscribe_to_topics():
            if planner.plan_path():
                # Publish if successfully planner
                print("Publishing topics . . .")
                planner.publish_path()
            else:
                # Goal was not found or has been reached
                print("GOAL not found OR already reached")
        else:
            print("- Not all topics published -")
        # Keep the ROS node alive   
        rospy.spin()
        
    except KeyboardInterrupt:
        print(" Shutting down ROS planner main node")
        
    except Exception as e:
        print(e)
        
if __name__ == '__main__':
    main()