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
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


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
        self.first_pose = True

        self.msg = Twist()

    def send_message(self):
        self.msg_publisher.publish(self.msg)

    def detection_callback(self, data):
        # Process incoming images
        if self.first_pose:
            self.ball_coordinates = data
            self.ball_found = True
            self.first_pose = False


    def execute_callback(self, goal_handle):
        result = GoToBall.Result()
        
        pose = PoseStamped()
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = 0.80#self.ball_coordinates.x
        pose.pose.position.y = 0.4#self.ball_coordinates.y
        pose.pose.position.z = 0.0#self.ball_coordinates.z
        (x, y, z, w) = quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        pose.header.stamp = self.get_clock().now().to_msg()


        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Goal pose: {pose}')
        
        self.first_pose = True
        result.at_ball = True
    
        goal_handle.succeed()

        return result



def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # Setting up initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "camera_link"
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    (x, y, z, w) = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose.pose.orientation.x = x
    initial_pose.pose.orientation.y = y
    initial_pose.pose.orientation.z = z
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    initial_pose.pose.orientation.w = w
    navigator.setInitialPose(initial_pose)

    # navigator.waitUntilNav2Active()

    find_ball_action_server = GoToBallServer(navigator=navigator)
    executor = MultiThreadedExecutor()
    executor.add_node(find_ball_action_server)
    executor.spin()
    # while 1:
    #     rclpy.spin_once(find_ball_action_server)



if __name__ == '__main__':
    main()