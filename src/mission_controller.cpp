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

// #define SKIDPAD_PLANNER "ros2 run path_planner my_node --ros-args -p planner_mode:=2"
#define TRACKDRIVE_PLANNER "/home/lart-tasha/Documents/repos/ros2_ws/install/path_planner/lib/path_planner/my_node --ros-args -p planner_mode:=4"
#define ACCELERATION_PLANNER "/home/lart-tasha/Documents/repos/ros2_ws/install/path_planner/lib/path_planner/my_node --ros-args -p planner_mode:=1"
#define INSPECTION_MISSION "/home/lart-tasha/Documents/repos/ros2_ws/install/inspection_mission/lib/inspection_mission/inspection_mission_node"
#define SKIDPAD_PLANNER "/home/lart-tasha/Documents/repos/ros2_ws/install/path_planner/lib/path_planner/my_node --ros-args -p planner_mode:=2"


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

    state_subscriber_ = this->create_subscription<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10, std::bind(&Mission_controller::process_state, this, _1));//get the state from the state controller

    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);
    mission_finished_pub_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as", 10);//publisher to state_controller true if all laps were made, topic to be defined
    
    this->current_mission_msg.data = lart_msgs::msg::Mission::MANUAL; //default mission
  }

  ~Mission_controller() {
    sleep(5);
    RCLCPP_INFO(this->get_logger(), "Shutting down Mission_controller and terminating the path_planner.");
    do{
      RCLCPP_WARN(this->get_logger(), "%d", this->planner_process_->id());
      this->planner_process_->terminate();
      if (this->planner_process_->running()) 
        // If the process is still running, send SIGKILL
        ::kill(this->planner_process_->id(), SIGKILL);
      planner_process_->wait();
    }while(this->planner_process_ && this->planner_process_->running());
    if (inspection_process_ && inspection_process_->running()) {
        ::kill(this->inspection_process_->id(), SIGINT);
    }
    RCLCPP_WARN(this->get_logger(), "%d", this->spac_process_->id());
    if (spac_process_ && spac_process_->running()) {
        ::kill(this->spac_process_->id(), SIGINT);
    }
    if (zed_process_ && zed_process_->running()) {
        ::kill(this->zed_process_->id(), SIGINT);
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
  std::chrono::steady_clock::time_point finish_change_time;

  void lap_count(const lart_msgs::msg::SlamStats::SharedPtr msg) 
  { 
    if (this->current_mission_msg.data == lart_msgs::msg::Mission::MANUAL) {
      return; // Do not process lap count in manual mode
    }

    lap_counter = msg->lap_count;

    if (lap_counter == laps){
      RCLCPP_INFO(this->get_logger(), "Mission finished, laps completed: %d", lap_counter);

      lart_msgs::msg::State msg;
      msg.data = lart_msgs::msg::State::FINISH;
      finish_change_time = std::chrono::steady_clock::now();
      mission_finished_pub_->publish(msg);

      if (std::chrono::steady_clock::now() - finish_change_time > std::chrono::seconds(5)) {
        RCLCPP_INFO(this->get_logger(), "Mission finished, shutting down node.");
        rclcpp::shutdown(); // Shutdown the node after 5 seconds of mission finish
      }
    }
  }

  void process_state(const lart_msgs::msg::State::SharedPtr msg)
  {
    if (msg->data == lart_msgs::msg::State::FINISH || msg->data == lart_msgs::msg::State::EMERGENCY) {
      rclcpp::shutdown(); // Shutdown the node if mission is finished or emergency state is reached
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
      //inspection_process_ = std::make_unique<bp::child>(INSPECTION_MISSION);
      inspection_process_ = std::make_unique<bp::child>("/bin/bash",  "-c" ,INSPECTION_MISSION);
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
      spac_process_ = std::make_unique<bp::child>("/bin/bash",  "-c" ,SPAC); 
    }while(!spac_process_->running());
    RCLCPP_INFO(this->get_logger(), "SPAC activated");
    
    is_spac_running = true;
  }

  void activate_zed_bridge(){
    if(zed_process_ && zed_process_->running()){
      return;
    }

    if(is_zed_running){
      return;
    }

    do{
      //intializes the zed_bridge node
      zed_process_ = std::make_unique<bp::child>("/bin/bash",  "-c" ,ZED_BRIDGE); 
    }while(!zed_process_->running());
    RCLCPP_INFO(this->get_logger(), "ZED Bridge activated");

    is_zed_running = true;
  }

  void process_mission( const lart_msgs::msg::Mission::SharedPtr msg)
  {
    auto mission = msg->data;

    switch(mission){
      case lart_msgs::msg::Mission::MANUAL:
        break;
      case lart_msgs::msg::Mission::ACCELERATION:
        laps = LAPS_ACCELERATION;
        current_mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
        activate_zed_bridge();
        activate_planner(ACCELERATION_PLANNER);
        activate_spac();
        RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
        break;

      case lart_msgs::msg::Mission::SKIDPAD:
        //call the custom launch file for the skidpad mission mode of the panner. :(
        laps = LAPS_SKIDPAD;
        current_mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
        activate_zed_bridge();
        activate_planner(SKIDPAD_PLANNER);
        activate_spac();
        RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
        break;

      case lart_msgs::msg::Mission::TRACKDRIVE:
        laps = LAPS_TRACKDRIVE;
        current_mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
        activate_zed_bridge();
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
        RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
        break;

      case lart_msgs::msg::Mission::EBS_TEST:
        current_mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
        laps = LAPS_EBS_TEST;
        activate_zed_bridge();
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
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
        activate_zed_bridge();
        activate_planner(TRACKDRIVE_PLANNER); 
        activate_spac();
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
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber_;
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
