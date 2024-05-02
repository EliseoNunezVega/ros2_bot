import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
import actions_py.process_image as proc


from action_interfaces.action import GoToBall


class GoToBallServer(Node):

    def __init__(self):
        super().__init__('go_to_ball_server')
        self.cb_group_ = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            GoToBall,
            'gotoball',
            self.execute_callback,
            callback_group=self.cb_group_)

        self.image_sub = self.create_subscription(Point,"/detected_ball",self.detection_callback,callback_group=self.cb_group_, qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.msg_publisher = self.create_publisher(Twist, '/cmd_vel', 40)

        self.ball_found  = False
        self.desired_z_distance = 0.2
        self.target_x = 0.0
        self.max_x_offset = 0.01
        self.xP = 0.3


    def detection_callback(self, data):
        # Process incoming images
        self.ball_coordinates = data
        self.ball_found = True
    

    def execute_callback(self, goal_handle):
        msg = Twist()
        result = GoToBall.Result()

        x_offset = self.ball_coordinates.x - self.target_x

        if (self.ball_coordinates.z >= self.desired_z_distance) and (abs(x_offset) < self.max_x_offset):
            self.get_logger().info("Ball is close enough")
            msg.linear.x = 0.0
            result.at_ball = True
            
        else:
            

            result.at_ball = False

            if self.ball_coordinates.z < self.desired_z_distance:
                msg.linear.x = 0.1
                self.get_logger().info('Moving towards ball in Z...')
            if abs(x_offset) > self.max_x_offset:
                self.get_logger().info('Moving towards ball in X...')
                msg.angular.z = -x_offset * self.xP

        self.msg_publisher.publish(msg)
        goal_handle.succeed()

        return result
    


def main(args=None):
    rclpy.init(args=args)

    find_ball_action_server = GoToBallServer()
    executor = MultiThreadedExecutor()
    executor.add_node(find_ball_action_server)
    executor.spin()
    # while 1:
    #     rclpy.spin_once(find_ball_action_server)



if __name__ == '__main__':
    main()