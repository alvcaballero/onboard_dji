/** @file landing_manager.cpp
 *
 */
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>

using namespace dji_osdk_ros;

// global variables
bool exist_mission = true;

void ModeCallback(const std_msgs::UInt8::ConstPtr& msg){
  // msg->data   joystick? = 14
  ros::NodeHandle n;
  MissionWpGetInfo mission_get_info;
  auto get_mission_info_client = n.serviceClient<MissionWpGetInfo>("dji_osdk_ros/mission_waypoint_getInfo");
  get_mission_info_client.call(mission_get_info);
  MissionWaypointTask waypointTask = mission_get_info.response.waypoint_task;

  if(waypointTask.mission_waypoint.empty() && exist_mission){
    FlightTaskControl control_task;
    control_task.request.task = FlightTaskControl::Request::TASK_FORCE_LANDING;
    ros::ServiceClient task_control_client  = n.serviceClient<FlightTaskControl>("flight_task_control");
    task_control_client.call(control_task);
    std::cout << "Landing!" << std::endl;
    exist_mission = false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vs_manager_node");
  ros::NodeHandle nh;

  ros::Subscriber flight_mode_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);

  ros::spin();
  
  return 0;
}
