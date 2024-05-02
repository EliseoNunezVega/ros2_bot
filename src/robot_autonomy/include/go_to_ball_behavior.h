#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/go_to_ball.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GetToBall : public BT::StatefulActionNode
{
public:
  GetToBall(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using GoToBall = action_interfaces::action::GoToBall;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<GoToBall>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<GoToBall>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void go_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};