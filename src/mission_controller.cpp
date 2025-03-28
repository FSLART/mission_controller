// While it has not been defined, i added to ASStatus the int8 mission_finished field for the mission controller communicate to the state controller that all laps have been made
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <boost/process.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"

#include "std_msgs/msg/bool.hpp"

#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/as_status.hpp"

#define LAPS_ACCELERATION 1
#define LAPS_SKIDPAD 2
#define LAPS_TRACKDRIVE 10 //according to D8.3.1 from rule book
#define LAPS_EBS_TEST 1 // if needed, not implemented for now
#define LAPS_AUTOCROSS 1 // according to D6.4.2 from rule book

#define SKIDPAD_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=2"
#define TRACKDRIVE_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=4"

using std::placeholders::_1;
namespace bp = boost::process;


class Mission_controller : public rclcpp::Node
{
public:
  Mission_controller(): Node("mission_controller")
  {
    lap_subscriber_ = this->create_subscription<std_msgs::msg::Int16>("/lapCount", 5, std::bind(&Mission_controller::lap_count, this, _1));//need to know the full path of the topic
    acu_mission_sub_ = this->create_subscription<lart_msgs::msg::Mission>("/acu_origin/system_status/critical_as/mission", 10, std::bind(&Mission_controller::process_mission, this, _1));//get the mission from the ACU
    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);
    mission_finished_pub_ = this->create_publisher<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as", 10);//publisher to state_controller true if all laps were made, topic to be defined
    
    timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Mission_controller::check_laps, this));
  }

  ~Mission_controller() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Mission_controller and terminating the path_planner.");
    if (child_process_ && child_process_->running()) {
        child_process_->terminate();
    }
}

private:
  lart_msgs::msg::Mission current_mission_msg;
  lart_msgs::msg::Mission previous_mission_msg;
  int32_t lap_counter = -1;
  int32_t laps = 0;
  bool is_planner_running = false;

  void lap_count(const std_msgs::msg::Int16::SharedPtr msg) 
  {
    lap_counter = msg->data;
  }

  void check_laps(){
    if(lap_counter >= laps){
      lart_msgs::msg::ASStatus msg;
      msg.state.data=lart_msgs::msg::State::FINISH;
      mission_finished_pub_->publish(msg);
    }
  }

  void activate_planner(std::string planner_mode){
    //check if the planner is already running
    if(is_planner_running){
      return;
    }

    do{
      //intializes the path_planner node with the desired mode
      child_process_ = std::make_unique<bp::child>(planner_mode); 
    }while(!child_process_->running());
    RCLCPP_INFO(this->get_logger(), "Planner activated in %s mode", planner_mode.c_str());
    
    is_planner_running = true;

  }

  void process_mission( const lart_msgs::msg::Mission::SharedPtr msg)
  {
    auto mission = msg->data;

    switch(mission){
      case lart_msgs::msg::Mission::ACCELERATION:
        current_mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
        laps = LAPS_ACCELERATION;
        activate_planner(TRACKDRIVE_PLANNER); 
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        break;

      case lart_msgs::msg::Mission::SKIDPAD:
        //call the custom launch file for the skidpad mission mode of the panner. :(
        current_mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
        laps = LAPS_SKIDPAD;
        activate_planner(SKIDPAD_PLANNER); 
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        break;

      case lart_msgs::msg::Mission::TRACKDRIVE:
        current_mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
        laps = LAPS_TRACKDRIVE;
        activate_planner(TRACKDRIVE_PLANNER); 
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        break;

      case lart_msgs::msg::Mission::EBS_TEST:
        current_mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
        laps = LAPS_EBS_TEST;
        activate_planner(TRACKDRIVE_PLANNER); 
        RCLCPP_INFO(this->get_logger(), "Mission is ebs test('%d')", mission);
        break;

      case lart_msgs::msg::Mission::INSPECTION:
        current_mission_msg.data= lart_msgs::msg::Mission::INSPECTION;
        RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
        break;

      case lart_msgs::msg::Mission::AUTOCROSS:
        current_mission_msg.data= lart_msgs::msg::Mission::AUTOCROSS;
        laps = LAPS_AUTOCROSS;
        activate_planner(TRACKDRIVE_PLANNER); 
        RCLCPP_INFO(this->get_logger(), "Mission is autocross('%d')", mission);
        break;

      default:
        RCLCPP_INFO(this->get_logger(), "Unknown mission('%d')", mission);
    }
    
    if(current_mission_msg.data != previous_mission_msg.data){
      previous_mission_msg.data = current_mission_msg.data;
      mission_pub_->publish(current_mission_msg);
    }
  }

  rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr acu_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lap_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<lart_msgs::msg::ASStatus>::SharedPtr mission_finished_pub_;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<bp::child> child_process_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
