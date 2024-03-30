import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from sailbot_msgs.action.action import FollowPath


class PathActionServer(Node):
    def __init__(self):
        super().__init__('path_action_server')
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = FollowPath.Feedback()
        success = False

        for index, position in enumerate(goal_handle.request.path):

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return FollowPath.Result(success=False)
            
            self.get_logger().info(f'Navigating to {position.latitude}, {position.longitude}')
            feedback_msg.current_target_waypoint_index = index
            goal_handle.publish_feedback(feedback_msg)
            rclpy.sleep(1)

        success = True
        result = FollowPath.Result()
        result.success = success
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = PathActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()