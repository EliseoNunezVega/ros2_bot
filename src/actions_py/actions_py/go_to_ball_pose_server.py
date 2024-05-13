import rclpy
from rclpy.action import ActionServer
import rclpy.duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Quaternion
import time
import actions_py.process_image as proc
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros

from action_interfaces.action import GoToBall
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
from tf2_msgs.msg import TFMessage
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
        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose_autonomy', 10)
        self.cam_pose_publisher = self.create_publisher(PoseStamped, "/camera_frame_pose", 10)
        self.msg_timer = self.create_timer(0.1, self.send_message, callback_group=self.cb_group_ )
        self.msg_timer.cancel()
        # self.map_pose_sub = self.create_subscription(TFMessage, "/tf", self.tf_callback, callback_group=self.cb_group_,qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value )

        self.ball_found  = False
        self.desired_z_distance = 0.1
        self.target_x = 0.0
        self.max_x_offset = 0.3
        self.xP = 0.3
        self.zP = 0.4
        self.first_pose = True

        self.current_robot_rotation = Quaternion()
        self.current_rotation_found = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.msg = Twist()

    def tf_callback(self, data):
        if self.ball_found:
            for transform in data.transforms:
                if transform.header.frame_id == "map":
                    self.current_robot_rotation = transform.transform.rotation
                    self.current_rotation_found = True

    def send_message(self):
        self.msg_publisher.publish(self.msg)

    def detection_callback(self, data):
        # Process incoming images
        if self.first_pose:
            self.ball_coordinates = data
            self.ball_found = True
            self.first_pose = False
            self.msg_timer.reset()


    def execute_callback(self, goal_handle):
        result = GoToBall.Result()

        pose = PoseStamped()
        pose.header.frame_id = "camera_link_optical"
        pose.pose.position.x = self.ball_coordinates.x
        pose.pose.position.y = self.ball_coordinates.y
        pose.pose.position.z = self.ball_coordinates.z

        (x, y, z, w) = quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation.x = x#trans.transform.rotation.x
        pose.pose.orientation.y = y#trans.transform.rotation.y
        pose.pose.orientation.z = z#trans.transform.rotation.z
        pose.pose.orientation.w = w#trans.transform.rotation.w


        pose.header.stamp = self.get_clock().now().to_msg()

        self.cam_pose_publisher.publish(pose)
        transformed_pose = self.tfBuffer.transform(pose, "map", timeout=rclpy.duration.Duration(seconds=3.0))

        trans = self.tfBuffer.lookup_transform("map", "base_link",  rclpy.time.Time())
        transformed_pose.pose.orientation.x = trans.transform.rotation.x
        transformed_pose.pose.orientation.y = trans.transform.rotation.y
        transformed_pose.pose.orientation.z = trans.transform.rotation.z
        transformed_pose.pose.orientation.w = trans.transform.rotation.w

        self.pose_publisher.publish(transformed_pose)

        self.get_logger().info(f'Goal pose: {transformed_pose}')

        if self.ball_found:
            self.ball_found = False
            self.navigator.goToPose(transformed_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Goal pose: {transformed_pose}')

        self.get_logger().info("GOOAAAL REACHED")

        self.first_pose = True
        result.at_ball = True

        goal_handle.succeed()

        return result



def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # Setting up initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
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