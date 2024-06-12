#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"


enum critical_as_mission {
    ACCELERATION=1,
    SKIDPAD=2,
    TRACKDRIVE=3,
    BRAKEDRIVE=4,
    INSPECTION=5,
    AUTOCROSS=6
    //UNKNOWN_MISSION
};

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber(): Node("subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int8>(
      "mission", 10, std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int8 & msg) const
  {
    process_mission(msg.data);// send the recived message to a function that processes it
  }

  void process_mission( int8_t data) const 
  {
    int mission = data;

    switch(mission){
      case ACCELERATION:
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        break;

      case SKIDPAD:
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        break;

      case TRACKDRIVE:
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        break;

      case BRAKEDRIVE:
        RCLCPP_INFO(this->get_logger(), "Mission is brakeDrive('%d')", mission);
        break;

      case INSPECTION:
        RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
        break;

      case AUTOCROSS:
        RCLCPP_INFO(this->get_logger(), "Mission is autocross('%d')", mission);
        break;

      default:
        RCLCPP_INFO(this->get_logger(), "Unknown mission('%d')", mission);
    }
  }
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
