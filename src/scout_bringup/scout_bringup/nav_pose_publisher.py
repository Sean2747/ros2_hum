import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.425
    initial_pose.pose.position.y = -1.798
    initial_pose.pose.orientation.z = 0.000
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -1.808
    goal_pose.pose.position.y = -1.785
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    go_to_pose_task = navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete(task=go_to_pose_task):
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print('Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
