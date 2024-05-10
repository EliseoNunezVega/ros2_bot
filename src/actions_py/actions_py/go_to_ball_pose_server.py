import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, PoseStamped
from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
import actions_py.process_image as proc
from nav2_simple_commander.robot_navigator import BasicNavigator

from action_interfaces.action import GoToBall


class GoToBallServer(Node):

    def __init__(self, navigator):
        super().__init__('go_to_ball_pose_server')
        self.cb_group_ = ReentrantCallbackGroup()

        self.navigator = navigator
        self._action_server = ActionServer(
            self,
            GoToBall,
            'gotoball',
            self.execute_callback,
            callback_group=self.cb_group_)

        self.image_sub = self.create_subscription(Point,"/detected_ball_3d",self.detection_callback,callback_group=self.cb_group_, qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.msg_publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        self.msg_timer = self.create_timer(0.1, self.send_message, callback_group=self.cb_group_ )

        self.ball_found  = False
        self.desired_z_distance = 0.1
        self.target_x = 0.0
        self.max_x_offset = 0.3
        self.xP = 0.3
        self.zP = 0.4

        self.msg = Twist()

    def send_message(self):
        self.msg_publisher.publish(self.msg)

    def detection_callback(self, data):
        # Process incoming images
        self.ball_coordinates = data
        self.ball_found = True


    def execute_callback(self, goal_handle):
        self.msg = Twist()
        result = GoToBall.Result()

        x_offset = self.ball_coordinates.x - self.target_x
        z_offset = abs(self.ball_coordinates.z) - self.desired_z_distance

        if (abs(self.ball_coordinates.z) >= self.desired_z_distance):
            self.get_logger().info("Ball is close enough")
            self.msg.linear.x = 0.0
            result.at_ball = True
        else:
            self.msg.linear.x = 0.2 #z_offset * self.zP
            self.get_logger().info(f"Z_Offset: {z_offset}!!!!!!!!!!!!")
            self.get_logger().info('Moving towards ball in Z...')


        if (abs(x_offset) < self.max_x_offset):
            self.msg.angular.z = 0.0
        else:
            self.msg.angular.z = -x_offset * self.xP
            self.get_logger().info(f'Moving towards ball in X...{self.msg.angular.z}')


        goal_handle.succeed()

        return result



def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header
    initial_pose.pose.orientation =
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    find_ball_action_server = GoToBallServer(navigator=navigator)
    executor = MultiThreadedExecutor()
    executor.add_node(find_ball_action_server)
    executor.spin()
    # while 1:
    #     rclpy.spin_once(find_ball_action_server)



if __name__ == '__main__':
    main()