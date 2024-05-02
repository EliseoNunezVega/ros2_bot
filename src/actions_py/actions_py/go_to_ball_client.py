import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_interfaces.action import GoToBall


class GoToBallActionClient(Node):

    def __init__(self):
        super().__init__('go_to_ball_action_client')
        self._action_client = ActionClient(self, GoToBall, 'gotoball')

    def send_goal(self, go_to_ball_true):
        goal_msg = GoToBall.Goal()
        goal_msg.go_to_ball_true = go_to_ball_true

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = GoToBallActionClient()

    future = action_client.send_goal(True)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
