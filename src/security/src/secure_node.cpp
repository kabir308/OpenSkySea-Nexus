#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <oqs/oqs.h>

class SecureNode : public rclcpp::Node
{
public:
  SecureNode()
  : Node("secure_node")
  {
    sig_ = OQS_SIG_new(OQS_SIG_alg_dilithium_2);
    if (sig_ == NULL) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create signature object");
      return;
    }

    public_key_ = new uint8_t[sig_->length_public_key];
    secret_key_ = new uint8_t[sig_->length_secret_key];
    OQS_STATUS status = OQS_SIG_keypair(sig_, public_key_, secret_key_);
    if (status != OQS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate keypair");
      return;
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("signed_message", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "signed_message", 10, std::bind(&SecureNode::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&SecureNode::publish_message, this));
  }

  ~SecureNode()
  {
    if (sig_) {
      OQS_SIG_free(sig_);
    }
    delete[] public_key_;
    delete[] secret_key_;
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // For simplicity, we assume the message is a concatenation of the signature and the message
    size_t signature_len = sig_->length_signature;
    if (msg->data.length() < signature_len) {
      RCLCPP_ERROR(this->get_logger(), "Invalid message format");
      return;
    }

    std::string signature_str = msg->data.substr(0, signature_len);
    std::string message_str = msg->data.substr(signature_len);

    OQS_STATUS status = OQS_SIG_verify(
      sig_, (const uint8_t *)message_str.c_str(), message_str.length(),
      (const uint8_t *)signature_str.c_str(), signature_len, public_key_);

    if (status == OQS_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Successfully verified message: %s", message_str.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to verify message");
    }
  }

  void publish_message()
  {
    std::string message_str = "Hello, world!";
    uint8_t signature[sig_->length_signature];
    size_t signature_len;

    OQS_STATUS status = OQS_SIG_sign(
      sig_, signature, &signature_len, (const uint8_t *)message_str.c_str(),
      message_str.length(), secret_key_);

    if (status == OQS_SUCCESS) {
      auto message = std_msgs::msg::String();
      message.data = std::string((char *)signature, signature_len) + message_str;
      publisher_->publish(message);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to sign message");
    }
  }

  OQS_SIG *sig_ = nullptr;
  uint8_t *public_key_ = nullptr;
  uint8_t *secret_key_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SecureNode>());
  rclcpp::shutdown();
  return 0;
}
