#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <string>
using namespace std::chrono_literals;

class DiffDrivePublisher : public rclcpp::Node
{
public:
  DiffDrivePublisher()
  : Node("diff_drive_publisher")
  {
      // robot to drive, e.g. robot_0 (empty: use the node namespace)
      const std::string robot = this->declare_parameter<std::string>("robot", "");
      std::string topic = "diff_drive_base_controller/cmd_vel_unstamped";
      if (!robot.empty()) {
        topic = "/" + robot + "/" + topic;
      }
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
      RCLCPP_INFO(this->get_logger(), "Publishing on %s", publisher_->get_topic_name());
        
      timer_ = this->create_wall_timer(
        10ms, std::bind(&DiffDrivePublisher::publishCommand, this));
  }   

private:
  void publishCommand()
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = 0.0;  // Forward speed in m/s
    command.angular.z = 3; // Angular speed in rad/s
    publisher_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDrivePublisher>());
  rclcpp::shutdown();
  return 0;
}
