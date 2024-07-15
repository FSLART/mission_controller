#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/state.hpp"

#define LAPS_ACCELERATION 1
#define LAPS_SKIDPAD 2
#define LAPS_TRACKDRIVE 10 //according to D8.3.1 from rule book
#define LAPS_EBS_TEST 1 // if needed, not implemented for now
#define LAPS_AUTOCROSS 1 // according to D6.4.2 from rule book

enum critical_as_mission {
    ACCELERATION=1,
    SKIDPAD=2,
    TRACKDRIVE=3,
    EBS_TEST=4,
    INSPECTION=5,
    AUTOCROSS=6
    //UNKNOWN_MISSION
};

using std::placeholders::_1;

class Mission_controller : public rclcpp::Node
{
public:
  Mission_controller(): Node("mission_controller"), lap_counter(0)
  {
    lap_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("LapCount", 5, std::bind(&Mission_controller::lap_count, this, _1));
    subscription_ = this->create_subscription<std_msgs::msg::Int8>("mission_button", 10, std::bind(&Mission_controller::process_mission, this, _1));//ask the correct name of the topic
    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("mission", 10);
    mission_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("mission_finished", 10);//publisher to state_controller true if all laps were made
    mission_finished.data=false;
  }

private:
  lart_msgs::msg::Mission mission_msg;
  std_msgs::msg::Bool mission_finished;
  int32_t lap_counter;

  void lap_count(const std_msgs::msg::Int32 & msg) 
  {
    lap_counter = msg.data;
  }

  void process_mission( const std_msgs::msg::Int8 & msg)
  {
    int mission = msg.data;

    switch(mission){
      case ACCELERATION:
        mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        mission_pub_->publish(mission_msg);
        if (lap_counter >= LAPS_ACCELERATION){
          mission_finished.data = true;
          mission_finished_pub_->publish(mission_finished);
        }
        break;

      case SKIDPAD:
        mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        mission_pub_->publish(mission_msg);
        if (lap_counter >= LAPS_SKIDPAD){
          mission_finished.data = true;
          mission_finished_pub_->publish(mission_finished);
        }
        break;

      case TRACKDRIVE:
        mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        mission_pub_->publish(mission_msg);
        if (lap_counter >= LAPS_TRACKDRIVE){
          mission_finished.data = true;
          mission_finished_pub_->publish(mission_finished);
        }
        break;

      case EBS_TEST:
        mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
        RCLCPP_INFO(this->get_logger(), "Mission is ebs test('%d')", mission);
        mission_pub_->publish(mission_msg);
        break;

      case INSPECTION:
        mission_msg.data= lart_msgs::msg::Mission::INSPECTION;
        RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
        mission_pub_->publish(mission_msg);
        break;

      case AUTOCROSS:
        mission_msg.data= lart_msgs::msg::Mission::AUTOCROSS;
        RCLCPP_INFO(this->get_logger(), "Mission is autocross('%d')", mission);
        mission_pub_->publish(mission_msg);
        if (lap_counter >= LAPS_AUTOCROSS){
          mission_finished.data = true;
          mission_finished_pub_->publish(mission_finished);
        }
        break;

      default:
        RCLCPP_INFO(this->get_logger(), "Unknown mission('%d')", mission);
    }
  }
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_finished_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
