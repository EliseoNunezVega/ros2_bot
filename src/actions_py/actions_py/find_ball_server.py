import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
import actions_py.process_image as proc



from action_interfaces.action import FindBall

global image_data

class FindBallServer(Node):

    def __init__(self):
        super().__init__('find_ball_server')
        self._action_server = ActionServer(
            self,
            FindBall,
            'findball',
            self.execute_callback)
    
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.msg_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.image_sub = self.create_subscription(Image,"/camera/image_raw/uncompressed",self.image_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.action_received = False

        self.declare_parameter('tuning_mode', False)


        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",27)
        self.declare_parameter("h_max",75)
        self.declare_parameter("s_min",31)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",21)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",4)
        self.declare_parameter("sz_max",82)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }


        self.bridge = CvBridge()
        self.current_ball_point = Point()
        self.ball_found  = False


    def image_callback(self, data):
        # Process incoming images
        global image_data
        image_data = data
    
    def process_image(self, image_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = image_data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = image_data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size
    
                # self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
                self.ball_found = True

        except CvBridgeError as e:
            print(e)  


    def execute_callback(self, goal_handle):

        self.get_logger().info('Looking for ball...')
        result = FindBall.Result()

        if not self.ball_found:
            msg = Twist()
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            self.msg_publisher.publish(msg)
            result.sequence = [0,0]

        if self.ball_found:
            result.sequence = [1,1]

        goal_handle.succeed()
           
        self.ball_found = False

        return result
    


def main(args=None):
    rclpy.init(args=args)

    find_ball_action_server = FindBallServer()
    
    while 1:
        rclpy.spin_once(find_ball_action_server)
        find_ball_action_server.process_image(image_data)



if __name__ == '__main__':
    main()