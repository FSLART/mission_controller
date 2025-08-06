// While it has not been defined, i added to ASStatus the int8 mission_finished field for the mission controller communicate to the state controller that all laps have been made
#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <boost/process.hpp>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"

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
#define LAPS_INSPECTION 1

#define INSPECTION_MISSION "/home/lart-tasha/Documents/repos/ros2_ws/install/inspection_mission/lib/inspection_mission/inspection_mission_node"
#define LAUNCH_FILE_ACCELERATION "ros2 launch startup_system acceleration.launch.py"
#define LAUNCH_FILE_SKIDPAD "ros2 launch startup_system skidpad.launch.py"
#define LAUNCH_FILE_TRACKDRIVE "ros2 launch startup_system autocross.launch.py"

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

    ignition_subscriber_ = this->create_subscription<std_msgs::msg::UInt16>("/system/ignition", 10, std::bind(&Mission_controller::process_ignition, this, _1));//get the ignition status from the state controller

    mission_pub_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);
    mission_finished_pub_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as", 10);//publisher to state_controller true if all laps were made, topic to be defined
    
    this->current_mission_msg.data = lart_msgs::msg::Mission::MANUAL; //default mission
  }

  ~Mission_controller() {
    sleep(5);
    RCLCPP_INFO(this->get_logger(), "Shutting down Mission_controller and terminating the path_planner.");
    if (inspection_process_ && inspection_process_->running()) {
        ::kill(this->inspection_process_->id(), SIGINT);
    }
    RCLCPP_WARN(this->get_logger(), "%d", this->nodes_process_->id());
    if (nodes_process_ && nodes_process_->running()) {
        ::kill(this->nodes_process_->id(), SIGINT);
    }

}

private:
  lart_msgs::msg::Mission current_mission_msg;
  lart_msgs::msg::Mission previous_mission_msg;
  int32_t lap_counter = -1;
  int32_t laps = 0;
  uint16_t ignition_status = 0; // 0 - off, 1 - on
  
  bool is_nodes_running = false;
  bool is_inspection_running = false;
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

  void process_ignition(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    ignition_status = msg->data;

  }

  void process_state(const lart_msgs::msg::State::SharedPtr msg)
  {
    if (msg->data == lart_msgs::msg::State::FINISH || msg->data == lart_msgs::msg::State::EMERGENCY) {
      rclcpp::shutdown(); // Shutdown the node if mission is finished or emergency state is reached
    }
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

  void activate_nodes(std::string cmd){
    if(is_nodes_running){
      return;
    }

    do{
      //intializes the nodes
      nodes_process_ = std::make_unique<bp::child>("/bin/bash",  "-c" ,cmd); 
    }while(!nodes_process_->running());
    RCLCPP_INFO(this->get_logger(), "nodes activated");
    
    is_nodes_running = true;
  }


  void process_mission( const lart_msgs::msg::Mission::SharedPtr msg)
  {
    auto mission = msg->data;
    if (ignition_status == 1){
      switch(mission){
        case lart_msgs::msg::Mission::MANUAL:
          break;
        case lart_msgs::msg::Mission::ACCELERATION:
          laps = LAPS_ACCELERATION;
          current_mission_msg.data= lart_msgs::msg::Mission::ACCELERATION;
          activate_nodes(LAUNCH_FILE_ACCELERATION);
          RCLCPP_INFO(this->get_logger(), "Mission is acceleration('%d')", mission);
          break;

        case lart_msgs::msg::Mission::SKIDPAD:
          //call the custom launch file for the skidpad mission mode of the panner. :(
          laps = LAPS_SKIDPAD;
          current_mission_msg.data= lart_msgs::msg::Mission::SKIDPAD;
          activate_nodes(LAUNCH_FILE_SKIDPAD);
          RCLCPP_INFO(this->get_logger(), "Mission is skidpad('%d')", mission);
          break;

        case lart_msgs::msg::Mission::TRACKDRIVE:
          laps = LAPS_TRACKDRIVE;
          current_mission_msg.data= lart_msgs::msg::Mission::TRACKDRIVE;
          activate_nodes(LAUNCH_FILE_TRACKDRIVE);
          RCLCPP_INFO(this->get_logger(), "Mission is trackDrive('%d')", mission);
          break;

        case lart_msgs::msg::Mission::EBS_TEST:
          current_mission_msg.data= lart_msgs::msg::Mission::EBS_TEST;
          laps = LAPS_EBS_TEST;
          activate_nodes(LAUNCH_FILE_TRACKDRIVE);
          RCLCPP_INFO(this->get_logger(), "Mission is ebs test('%d')", mission);
          break;

        case lart_msgs::msg::Mission::INSPECTION:
          current_mission_msg.data= lart_msgs::msg::Mission::INSPECTION;
          laps = LAPS_INSPECTION;
          activate_inspection();
          RCLCPP_INFO(this->get_logger(), "Mission is inspection('%d')", mission);
          break;

        case lart_msgs::msg::Mission::AUTOCROSS:
          current_mission_msg.data= lart_msgs::msg::Mission::AUTOCROSS;
          laps = LAPS_AUTOCROSS;
          activate_nodes(LAUNCH_FILE_TRACKDRIVE);
          RCLCPP_INFO(this->get_logger(), "Mission is autocross('%d')", mission);
          break;

        default:
          RCLCPP_INFO(this->get_logger(), "Unknown mission('%d')", mission);
      }
    }
    mission_pub_->publish(current_mission_msg);
  }

  rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr acu_mission_sub_;
  rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr lap_subscriber_;
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr ignition_subscriber_;
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub_;
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr mission_finished_pub_;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<bp::child> nodes_process_;
  std::unique_ptr<bp::child> inspection_process_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission_controller>());
  rclcpp::shutdown();
  return 0;
}
