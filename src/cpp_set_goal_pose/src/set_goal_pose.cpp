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

class GoalPosePublisher : public rclcpp::Node
{
  public:
    GoalPosePublisher()
    : Node("goal_pose_setter"), message(geometry_msgs::msg::PoseStamped())
    {
      /* Declare parameters. */
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
    
    
  private:
    geometry_msgs::msg::PoseStamped message;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  /* Create node. */
  std::shared_ptr<GoalPosePublisher> goal_setter = std::make_shared<GoalPosePublisher>();

  /* Create a reliable publisher. */
  auto publisher = goal_setter->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);

  /* Wait for subscriber */
  while(true){
    auto cnt = goal_setter -> count_subscribers("/planning/mission_planning/goal");
    std::cerr << "[INFO] Number of subscriber: " << cnt << std::endl;
    if(!cnt) sleep(1);
    else break;
  }

  /* Publish the message. */
  publisher -> publish(goal_setter->get_msg());
  sleep(1);
  
  rclcpp::shutdown();
  return 0;
}