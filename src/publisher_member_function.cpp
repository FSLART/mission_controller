#include <functional>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("mission", 10);
    
  }
  
  void start_mission(){
    while(rclcpp::ok()){
      mission_send();
    }
  }

private:
  void mission_send()
  {
    int x;
    cin >> x; //read input from keyboard to test mission processing
    auto message = std_msgs::msg::Int8();
    message.data = static_cast<int8_t>(x);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = make_shared<Publisher>();
  node->start_mission();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
