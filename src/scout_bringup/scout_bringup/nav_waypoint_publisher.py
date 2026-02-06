from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main(args=None):
	rclpy.init()
	navigator = BasicNavigator()

	# Estimates the vehicle's current position based on provided pose
	estimate_pose = PoseStamped()
	estimate_pose.header.frame_id = 'map'
	estimate_pose.header.stamp = navigator.get_clock().now().to_msg()
	estimate_pose.pose.position.x = float(input("Enter initial: "))
	estimate_pose.pose.position.y = float(input("Enter initial y position: ")) 
	estimate_pose.pose.orientation.z = 0.0
	estimate_pose.pose.orientation.w = 1.0
	navigator.setInitialPose(estimate_pose)


	goal_poses = []
	
	# Set coordinates for waypoint 1
	goal_pose = PoseStamped()
	goal_pose1.header.frame_id = 'map'
	goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
	goal_pose1.pose.position.x = float(input("Enter waypoint x1: "))
	goal_pose1.pose.position.y = float(input("Enter waypoint y1: "))
	goal_pose1.pose.orientation.w = 1.0
	goal_pose1.pose.orientation.z = 0.0
	goal_poses.append(goal_pose)

	# Set coordinates for waypoint 2
	goal_pose = PoseStamped()
	goal_pose1.header.frame_id = 'map'
	goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
	goal_pose1.pose.position.x = float(input("Enter waypoint x2: "))
	goal_pose1.pose.position.y = float(input("Enter waypoint y2: "))
	goal_pose1.pose.orientation.w = 1.0
	goal_pose1.pose.orientation.z = 0.0
	goal_poses.append(goal_pose)

	navigator.followWaypoints(goal_poses)

if __name__ == '__main__':
	main()

