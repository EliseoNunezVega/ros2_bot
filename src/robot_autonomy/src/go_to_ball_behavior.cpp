#include "go_to_ball_behavior.h"
#include "yaml-cpp/yaml.h"
#include <string>

GetToBall::GetToBall(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<GoToBall>(node_ptr_, "gotoball");
  done_flag_ = false;
}

BT::PortsList GetToBall::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GetToBall::onStart()
{
//   // Get location key from port and read YAML file
//   BT::Optional<std::string> loc = getInput<std::string>("loc");
//   const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

//   YAML::Node locations = YAML::LoadFile(location_file);

//   std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<GoToBall>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GetToBall::go_to_pose_callback, this, std::placeholders::_1);

//   // make pose
  auto goal_msg = GoToBall::Goal();
  goal_msg.go_to_ball_true = 1;
//   goal_msg.pose.header.frame_id = "map";
//   goal_msg.pose.pose.position.x = pose[0];
//   goal_msg.pose.pose.position.y = pose[1];

//   tf2::Quaternion q;
//   q.setRPY(0, 0, pose[2]);
//   q.normalize(); // todo: why?
//   goal_msg.pose.pose.orientation = tf2::toMsg(q);

//   // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent scan goal\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetToBall::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GetToBall::go_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}