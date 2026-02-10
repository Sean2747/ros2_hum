import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.682
    initial_pose.pose.position.y = 0.132
    initial_pose.pose.orientation.z = -1.0
    initial_pose.pose.orientation.w = 0.000
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    yaw_deg = 90
    quaternion = quaternion_from_euler(0, 0, yaw_deg)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -1.808
    goal_pose.pose.position.y = -1.785
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            print(
                'Distance remaining: '
                + '{:.2f}'.format(feedback.distance_remaining)
                + ' meters.'
            )

            #print(
            #    'Position error: '
            #    + '{:.2f}'.format(feedback.position_tracking_error)
            #    + ' meters.'
            #)

            #print(
            #    'Heading error: '
            #    + '{:.2f}'.format(feedback.heading_tracking_error)
            #    + ' radians.'
            #)

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                go_to_pose_task = navigator.goToPose(goal_pose)

    # Do something depending on the return code
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
