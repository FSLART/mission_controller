#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/state.hpp"

#define LAPS_ACCELERATION 1
#define LAPS_SKIDPAD 2
#define LAPS_TRACKDRIVE 10 //according to D8.3.1 from rule book
#define LAPS_EBS_TEST 1 // if needed, not implemented for now
#define LAPS_AUTOCROSS 1 // according to D6.4.2 from rule book

using std::placeholders::_1;
void check_laps(int laps);

class Mission_controller : public rclcpp::Node
{
public:
  Mission_controller(): Node("mission_controller"), lap_counter(0)
  {
    lap_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("LapCount", 5, std::bind(&Mission_controller::lap_count, this, _1));
    subscription_ = this->create_subscription<std_msgs::msg::Int8>("mission_button", 10, std::bind(&Mission_controller::process_mission, this, _1));//ask the correct name of the topic
    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("mission", 10);
    mission_finished_pub_ = this->create_publisher<std_msgs::msg::Int8>("mission_finished", 10);//publisher to state_controller true if all laps were made
    mission_finished.data=0;//1 if true
  }

private:
  lart_msgs::msg::Mission current_mission_msg;
  lart_msgs::msg::Mission previous_mission_msg;
  std_msgs::msg::Int8 mission_finished;
  int32_t lap_counter;

  void lap_count(const std_msgs::msg::Int32 & msg) 
  {
    lap_counter = msg.data;
  }

  void process_mission( const std_msgs::msg::Int8 & msg)
  {
    int mission = msg.data;

    switch(mission){
      case lart_msgs::msg::Mission::ACCELERATION:
        current_mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        check_laps(LAPS_ACCELERATION);
        break;

      case lart_msgs::msg::Mission::SKIDPAD:
        current_mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        check_laps(LAPS_SKIDPAD);
        break;

      case lart_msgs::msg::Mission::TRACKDRIVE:
        current_mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        check_laps(LAPS_TRACKDRIVE);
        break;

      case lart_msgs::msg::Mission::EBS_TEST:
        current_mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
        RCLCPP_INFO(this->get_logger(), "Mission is ebs test('%d')", mission);
        break;

      case lart_msgs::msg::Mission::INSPECTION:
        current_mission_msg.data= lart_msgs::msg::Mission::INSPECTION;
        RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
        break;

      case lart_msgs::msg::Mission::AUTOCROSS:
        current_mission_msg.data= lart_msgs::msg::Mission::AUTOCROSS;
        RCLCPP_INFO(this->get_logger(), "Mission is autocross('%d')", mission);
        check_laps(LAPS_AUTOCROSS);
        break;

      default:
        RCLCPP_INFO(this->get_logger(), "Unknown mission('%d')", mission);
    }
    
    if(current_mission_msg.data != previous_mission_msg.data){
      previous_mission_msg.data = current_mission_msg.data;
      mission_pub_->publish(current_mission_msg);
    }
  }

  void check_laps(int laps){
    if(lap_counter >= laps){
      mission_finished.data = 1;
      mission_finished_pub_->publish(mission_finished);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_finished_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
