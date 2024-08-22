// While it has not been defined, i added to ASStatus the int8 mission_finished field for the mission controller communicate to the state controller that all laps have been made
#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
//#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/as_status.hpp"

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
  Mission_controller(): Node("mission_controller")
  {
    lap_subscriber_ = this->create_subscription<lart_msgs::msg::ASStatus>("LapCount", 5, std::bind(&Mission_controller::lap_count, this, _1));//need to know the full path of the topic
    acu_mission_sub_ = this->create_subscription<lart_msgs::msg::Mission>("/acu_origin/system_status/critical_as/mission", 10, std::bind(&Mission_controller::process_mission, this, _1));//get the mission from the ACU
    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);
    mission_finished_pub_ = this->create_publisher<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as/", 10);//publisher to state_controller true if all laps were made, topic to be defined
  }

private:
  lart_msgs::msg::Mission current_mission_msg;
  lart_msgs::msg::Mission previous_mission_msg;
  int32_t lap_counter;

  void lap_count(const lart_msgs::msg::ASStatus msg) 
  {
    //lap_counter = msg.data;
  }

  void process_mission( const lart_msgs::msg::Mission msg)
  {
    /* mission = msg.data;

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
    }*/
  }

  void check_laps(int laps){
    if(lap_counter >= laps){
      lart_msgs::msg::ASStatus msg;
      msg.state.data=lart_msgs::msg::State::FINISH;
      mission_finished_pub_->publish(msg);
    }
  }

  rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr acu_mission_sub_;
  rclcpp::Subscription<lart_msgs::msg::ASStatus>::SharedPtr lap_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<lart_msgs::msg::ASStatus>::SharedPtr mission_finished_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
