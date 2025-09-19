#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/last_message.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::LastMessage::Request> request,
          std::shared_ptr<example_interfaces::srv::LastMessage::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("last_message_server");

  rclcpp::Service<example_interfaces::srv::LastMessage>::SharedPtr service =
    node->create_service<example_interfaces::srv::LastMessage>("last_message", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}