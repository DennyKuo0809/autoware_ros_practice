#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GoalPosePublisher : public rclcpp::Node
{
  public:
    GoalPosePublisher()
    : Node("goal_pose_publisher"), message(geometry_msgs::msg::PoseStamped())
    {
      /* Declare parameters */
      this->declare_parameter<double>("position.x");
      this->declare_parameter<double>("position.y");
      this->declare_parameter<double>("position.z");
      this->declare_parameter<double>("orientation.x");
      this->declare_parameter<double>("orientation.y");
      this->declare_parameter<double>("orientation.z");
      this->declare_parameter<double>("orientation.w");

      double px, py, pz;
      double ox, oy, oz, ow;

      this->get_parameter("position.x", px);
      this->get_parameter("position.y", py);
      this->get_parameter("position.z", pz);
      this->get_parameter("orientation.x", ox);
      this->get_parameter("orientation.y", oy);
      this->get_parameter("orientation.z", oz);
      this->get_parameter("orientation.w", ow);
      

      /* Create message */
      message = geometry_msgs::msg::PoseStamped();
      /* message: header */
      message.header.frame_id = "map";
      message.header.stamp.sec = 0.0;
      message.header.stamp.nanosec = 0.0;

      /* message: position */
      message.pose.position.x = px;
      message.pose.position.y = py;
      message.pose.position.z = pz;

      /* message: orientation */
      message.pose.orientation.x = ox;
      message.pose.orientation.y = oy;
      message.pose.orientation.z = oz;
      message.pose.orientation.w = ow;
    }

    geometry_msgs::msg::PoseStamped get_msg() const{
      return message;
    }
    
    std::vector<std::string> get_required_params() const{
      return required_params;
    }

    
  private:
    std::vector<std::string> required_params = {
      "position.x",
      "position.y",
      "position.z",
      "orientation.x",
      "orientation.y",
      "orientation.z",
      "orientation.w",
    };
    geometry_msgs::msg::PoseStamped message;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  /* Create Node */
  std::shared_ptr<GoalPosePublisher> goal_setter = std::make_shared<GoalPosePublisher>();

  /* Ensure all parameter are ready */
  std::vector<std::string> required_params = goal_setter -> get_required_params();
  bool params_ready = false;
  while(!params_ready){
    params_ready = true;
    for(const auto& para_name : required_params){
      if(!goal_setter -> has_parameter(para_name)){
        params_ready = false;
        break;
      }
    }
    sleep(1);
  }

  /* Create reliable publisher */
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto publisher = goal_setter->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", qos_profile);
  
  /* Publish the message */
  publisher->publish(goal_setter -> get_msg());
  sleep(1);
  rclcpp::shutdown();
  return 0;
}