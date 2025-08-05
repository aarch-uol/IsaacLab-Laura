#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// Include your custom service header.
// Assuming your package is named 'my_server_package' and the service is GetLatestMessage.srv
#include "cpp_srvcli/srv/get_latest_message.hpp"

#include <memory>
#include <string>

// Define a namespace for convenience, matching your package name
namespace cpp_srvcli
{

class LatestMessageServer : public rclcpp::Node
{
public:
  // Constructor for the node
  LatestMessageServer()
  : Node("latest_message_server"),
    latest_topic_message_("No message received yet on /topic") // Initialize with a default message
  {
    // 1. Create a subscriber to '/topic'
    // It will listen for String messages and call the topic_callback method.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/topic",
      10, // QoS history depth
      std::bind(&LatestMessageServer::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to /topic...");

    // 2. Create a service server
    // It will provide the 'get_latest_message' service using your custom service type.
    // When the service is called, it will execute the get_latest_message_callback method.
    service_ = this->create_service<srv::GetLatestMessage>(
      "get_latest_message",
      std::bind(&LatestMessageServer::get_latest_message_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service 'get_latest_message' ready.");
  }

private:
  // Callback function for the subscriber
  // This method is called every time a new message is received on '/topic'.
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_topic_message_ = msg->data; // Store the latest message data
    RCLCPP_INFO(this->get_logger(), "Received on /topic: '%s'", msg->data.c_str());
  }

  // Callback function for the service server
  // This method is called when a client requests the 'get_latest_message' service.
  void get_latest_message_callback(
    const std::shared_ptr<srv::GetLatestMessage::Request> request, // Request (empty for this service)
    std::shared_ptr<srv::GetLatestMessage::Response> response)    // Response to fill
  {
    (void)request; // Suppress unused variable warning for the request
    response->latest_message = latest_topic_message_; // Set the response to the stored message
    RCLCPP_INFO(this->get_logger(), "Service called. Returning latest message: '%s'", response->latest_message.c_str());
  }

  // Member variables for the subscriber, service, and stored message
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<srv::GetLatestMessage>::SharedPtr service_;
  std::string latest_topic_message_; // Stores the last message from /topic
};

} // namespace my_server_package

// Main function to initialize ROS2 and run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS2 client library
  // Create an instance of your server node and spin it
  rclcpp::spin(std::make_shared<my_server_package::LatestMessageServer>());
  rclcpp::shutdown(); // Shutdown the ROS2 client library when the node stops
  return 0;
}
