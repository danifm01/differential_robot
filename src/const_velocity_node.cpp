#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ConstantVelocityControl : public rclcpp::Node
{
    private:
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher;
        rclcpp::TimerBase::SharedPtr timer;
        void node_callback()
        {
            geometry_msgs::msg::Twist vel;
            vel.linear.x = 0.1;
            this->publisher->publish(vel);
            return;
        };
    public:
        ConstantVelocityControl() : Node("constant_velocity_node")
        {
            publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer = this->create_wall_timer(500ms, std::bind(&ConstantVelocityControl::node_callback, this));
        };
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConstantVelocityControl>());
  rclcpp::shutdown();
  return 0;
}