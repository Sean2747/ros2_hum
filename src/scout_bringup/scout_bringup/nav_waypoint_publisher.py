from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def main(args=None):
	rclpy.init()
	navigator = BasicNavigator()

	# Estimates the vehicle's current position based on provided pose
	estimate_pose = PoseStamped()
	estimate_pose.header.frame_id = 'map'
	estimate_pose.header.stamp = navigator.get_clock().now().to_msg()
	estimate_pose.pose.position.x = float(input("Enter initial x position: "))
	estimate_pose.pose.position.y = float(input("Enter initial y position: ")) 
	estimate_pose.pose.orientation.z = 0.0
	estimate_pose.pose.orientation.w = 1.0
	navigator.setInitialPose(estimate_pose)

	goal_poses = []
	
	# Set coordinates for waypoint 1
	yaw_deg = float(input("Enter vehicle heading for waypoint 1: "))
	quaternion = quaternion_from_euler(0,0,yaw_deg)

	goal_pose = PoseStamped()
	goal_pose.header.frame_id = 'map'
	goal_pose.header.stamp = navigator.get_clock().now().to_msg()
	goal_pose.pose.position.x = float(input("Enter waypoint x1: "))
	goal_pose.pose.position.y = float(input("Enter waypoint y1: "))
	goal_pose.pose.orientation.x = quaternion[0]
	goal_pose.pose.orientation.y = quaternion[1]
	goal_pose.pose.orientation.z = quaternion[2]
	goal_pose.pose.orientation.w = quaternion[3]
	goal_poses.append(goal_pose)

	# Set coordinates for waypoint 2
	yaw_deg = float(input("Enter vehicle heading for waypoint 2: "))
	quaternion = quaternion_from_euler(0,0,yaw_deg)

	goal_pose = PoseStamped()
	goal_pose.header.frame_id = 'map'
	goal_pose.header.stamp = navigator.get_clock().now().to_msg()
	goal_pose.pose.position.x = float(input("Enter waypoint x2: "))
	goal_pose.pose.position.y = float(input("Enter waypoint y2: "))
	goal_pose.pose.orientation.x = quaternion[0]
	goal_pose.pose.orientation.y = quaternion[1]
	goal_pose.pose.orientation.z = quaternion[2]
	goal_pose.pose.orientation.w = quaternion[3]
	goal_poses.append(goal_pose)

	navigator.followWaypoints(goal_poses)

	# Prints status of the vehicle's progress
	while not navigator.isTaskComplete():
		feedback = navigator.getFeedback()
		print(feedback)

	# Prints status of the vehicle at the end of the program
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED:
		print("Goal succeeded")
	elif result == TaskResult.CANCELED:
		print("Goal was cancelled")
	elif result == TaskResult.FAILED:
		print(f"Goal failed. {navigator.error()}")

if __name__ == '__main__':
	main()

