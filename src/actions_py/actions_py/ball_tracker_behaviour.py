import py_trees_ros
import py_trees
from action_interfaces.action import FindBall
from rclpy.node import Node
import rclpy
import py_trees.console as console

class ScanForBall(Node):
    def __init__(self):
        super().__init__('scan_for_ball')


def main(args=None):
    rclpy.init(args=args)
    setup_kwargs = {'node' : ScanForBall }
    root = py_trees.composites.Sequence("Find and Track Ball", True)

    scan_rotate = py_trees_ros.action_clients.FromConstant(
            name="Rotate",
            action_type= FindBall,
            action_name="findball",
            action_goal=10,  # noqa
            # generate_feedback_message=lambda msg: "{:.2f}%%".format(msg.feedback.percentage_completed)
    )

    root.add_child(scan_rotate)

    tree = py_trees.trees.BehaviourTree(
        root=root,
    )
   

   
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()