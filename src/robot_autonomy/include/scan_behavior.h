#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/find_ball.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ScanForBall : public BT::StatefulActionNode
{
public:
  ScanForBall(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using FindBall = action_interfaces::action::FindBall;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<FindBall>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<FindBall>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};