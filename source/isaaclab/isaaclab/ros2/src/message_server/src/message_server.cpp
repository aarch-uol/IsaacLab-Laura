#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "message_server/srv/get_latest_message.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MessageServer : public rclcpp::Node
{
public:
    MessageServer() : Node("message_server")
    {
        // Initialize latest message as empty
        latest_message_ = "No messages received yet.";

        // Subscriber to '/topic'
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/topic", 10,
            std::bind(&MessageServer::topic_callback, this, _1));

        // Service Server
        service_ = this->create_service<message_server::srv::GetLatestMessage>(
            "get_latest_message",
            std::bind(&MessageServer::handle_service, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Message server is up.");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        latest_message_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", latest_message_.c_str());
    }

    void handle_service(
        const std::shared_ptr<message_server::srv::GetLatestMessage::Request> request,
        std::shared_ptr<message_server::srv::GetLatestMessage::Response> response)
    {
        (void)request;  // unused
        response->latest_message = latest_message_;
        RCLCPP_INFO(this->get_logger(), "Service called. Responding with: '%s'", latest_message_.c_str());
    }

    std::string latest_message_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Service<message_server::srv::GetLatestMessage>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageServer>());
    rclcpp::shutdown();
    return 0;
}
