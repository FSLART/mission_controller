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
#include "lart_msgs/msg/slam_stats.hpp"


#define LAPS_ACCELERATION 1
#define LAPS_SKIDPAD 2
#define LAPS_TRACKDRIVE 10 //according to D8.3.1 from rule book
#define LAPS_EBS_TEST 1 // if needed, not implemented for now
#define LAPS_AUTOCROSS 1 // according to D6.4.2 from rule book

#define SKIDPAD_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=2"
#define TRACKDRIVE_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=4"
#define ACCELERATION_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=1"
#define INSPECTION_MISSION "ros2 run inspection_mission inspection_mission_node"

#define SPAC "ros2 launch spac2_0 drivemodel.launch.xml"

#define ZED_BRIDGE "ros2 launch zed_bridge zed_bridge_launch.py"

using std::placeholders::_1;
namespace bp = boost::process;


class Mission_controller : public rclcpp::Node
{
public:
  Mission_controller(): Node("mission_controller")
  {
    lap_subscriber_ = this->create_subscription<lart_msgs::msg::SlamStats>("/ekf/stats", 5, std::bind(&Mission_controller::lap_count, this, _1));//need to know the full path of the topic
    acu_mission_sub_ = this->create_subscription<lart_msgs::msg::Mission>("/acu_origin/system_status/critical_as/mission", 10, std::bind(&Mission_controller::process_mission, this, _1));//get the mission from the ACU
    
    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);
    mission_finished_pub_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as", 10);//publisher to state_controller true if all laps were made, topic to be defined
    
    this->current_mission_msg.data = lart_msgs::msg::Mission::MANUAL; //default mission
  }

  ~Mission_controller() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Mission_controller and terminating the path_planner.");
    if (planner_process_ && planner_process_->running()) {
        planner_process_->terminate();
    }
    if (inspection_process_ && inspection_process_->running()) {
        inspection_process_->terminate();
    }
    if (spac_process_ && spac_process_->running()) {
        spac_process_->terminate();
    }
    if (zed_process_ && zed_process_->running()) {
        zed_process_->terminate();
    }
}

private:
  lart_msgs::msg::Mission current_mission_msg;
  lart_msgs::msg::Mission previous_mission_msg;
  int32_t lap_counter = -1;
  int32_t laps = 0;
  bool is_planner_running = false;
  bool is_spac_running = false;
  bool is_inspection_running = false;
  bool is_zed_running = false;

  void lap_count(const lart_msgs::msg::SlamStats::SharedPtr msg) 
  {
    lap_counter = msg->lap_count;

    if (lap_counter >= laps){
      RCLCPP_INFO(this->get_logger(), "Mission finished, laps completed: %d", lap_counter);

      lart_msgs::msg::State msg;
      msg.data = lart_msgs::msg::State::FINISH;
      mission_finished_pub_->publish(msg);

      is_planner_running = false; // Reset planner state
      if (planner_process_ && planner_process_->running()) {
        planner_process_->terminate(); // Terminate the planner process
       }
      is_spac_running = false; // Reset SPAC state
      if (spac_process_ && spac_process_->running()) {
        spac_process_->terminate(); // Terminate the SPAC process
      }
      is_inspection_running = false; // Reset inspection state
      if (inspection_process_ && inspection_process_->running()) {
        inspection_process_->terminate(); // Terminate the inspection process
      }
      is_zed_running = false; // Reset ZED state
      if (zed_process_ && zed_process_->running()) {
        zed_process_->terminate(); // Terminate the ZED process
      }
    }
  }

  void activate_planner(std::string planner_mode){
    //check if the planner is already running
    if(is_planner_running){
      return;
    }

    do{
      //intializes the path_planner node with the desired mode
      planner_process_ = std::make_unique<bp::child>(planner_mode); 
    }while(!planner_process_->running());
    RCLCPP_INFO(this->get_logger(), "Planner activated in %s mode", planner_mode.c_str());
    
    is_planner_running = true;

  }

  void activate_inspection(){
    if(is_inspection_running){
      return;
    }

    do{
      //intializes the inspection node
      inspection_process_ = std::make_unique<bp::child>(INSPECTION_MISSION); 
    }while(!inspection_process_->running());
    RCLCPP_INFO(this->get_logger(), "Inspection activated");

    is_inspection_running = true;
  }

  void activate_spac(){
    if(is_spac_running){
      return;
    }

    do{
      //intializes the SPAC node
      spac_process_ = std::make_unique<bp::child>(SPAC); 
    }while(!spac_process_->running());
    RCLCPP_INFO(this->get_logger(), "SPAC activated");
    
    is_spac_running = true;
  }

  void activate_zed_bridge(){
    if(zed_process_ && zed_process_->running()){
      return;
    }

    do{
      //intializes the zed_bridge node
      zed_process_ = std::make_unique<bp::child>(ZED_BRIDGE); 
    }while(!zed_process_->running());
    RCLCPP_INFO(this->get_logger(), "ZED Bridge activated");
  }

  void process_mission( const lart_msgs::msg::Mission::SharedPtr msg)
  {
    auto mission = msg->data;

    switch(mission){
      case lart_msgs::msg::Mission::MANUAL:
        break;
      case lart_msgs::msg::Mission::ACCELERATION:
        current_mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
        laps = LAPS_ACCELERATION;
        activate_planner(ACCELERATION_PLANNER);
        activate_spac();
        activate_zed_bridge();
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        break;

      case lart_msgs::msg::Mission::SKIDPAD:
        //call the custom launch file for the skidpad mission mode of the panner. :(
        current_mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
        laps = LAPS_SKIDPAD;
        activate_planner(SKIDPAD_PLANNER);
        activate_spac();
        activate_zed_bridge();
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        break;

      case lart_msgs::msg::Mission::TRACKDRIVE:
        current_mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
        laps = LAPS_TRACKDRIVE;
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
        activate_zed_bridge();
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        break;

      case lart_msgs::msg::Mission::EBS_TEST:
        current_mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
        laps = LAPS_EBS_TEST;
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
        activate_zed_bridge();
        RCLCPP_INFO(this->get_logger(), "Mission is ebs test('%d')", mission);
        break;

      case lart_msgs::msg::Mission::INSPECTION:
        current_mission_msg.data= lart_msgs::msg::Mission::INSPECTION;
        activate_inspection();
        RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
        break;

      case lart_msgs::msg::Mission::AUTOCROSS:
        current_mission_msg.data= lart_msgs::msg::Mission::AUTOCROSS;
        laps = LAPS_AUTOCROSS;
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
        activate_zed_bridge();
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
  rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr lap_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr mission_finished_pub_;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<bp::child> planner_process_;
  std::unique_ptr<bp::child> spac_process_;
  std::unique_ptr<bp::child> zed_process_;
  std::unique_ptr<bp::child> inspection_process_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
