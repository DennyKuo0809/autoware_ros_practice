#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GoalPosePublisher : public rclcpp::Node
{
  public:
    GoalPosePublisher()
    : Node("goal_pose_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&GoalPosePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message.header.frame_id = "map";
      message.header.stamp.sec = 0.0;
      message.header.stamp.nanosec = 0.0;
      message.pose.position.x = 3729.45654296875;
      message.pose.position.y = 73723.671875;
      message.pose.position.z = 0.0;
      message.pose.orientation.x = 0.0;
      message.pose.orientation.y = 0.0;
      message.pose.orientation.z = -0.9624967609145418;
      message.pose.orientation.w = 0.27129317210172393;
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPosePublisher>());
  rclcpp::shutdown();
  return 0;
}