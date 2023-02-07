/** @file inspector_shoot_interval.cpp
 *
 */
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>

#include <std_srvs/SetBool.h>

#define CENTER_FOV 0.5   // Center of field of view  = (0.5, 0.5)

using namespace dji_osdk_ros;

void set_focus_point();

// global variables
bool is_flying = false;
bool exist_mission = true;
bool vs_activated = false;
short focus_trigger = 4;
float cur_height;

void StateCallback(const std_msgs::UInt8::ConstPtr& msg){
  // msg->data   STOPED = 0, ON_GROUND = 1,  IN_AIR = 2
  if(msg->data > 1 && !is_flying){
    is_flying = true;
  }else if(msg->data <= 1 && is_flying){
    is_flying = false;
  }
}

void ModeCallback(const std_msgs::UInt8::ConstPtr& msg){
  // msg->data   joystick? = 14
  //if(msg->data == 14 && is_flying && exist_mission){
    ros::NodeHandle n;
    MissionWpGetInfo mission_get_info;
    auto get_mission_info_client = n.serviceClient<MissionWpGetInfo>("dji_osdk_ros/mission_waypoint_getInfo");
    get_mission_info_client.call(mission_get_info);
    MissionWaypointTask waypointTask = mission_get_info.response.waypoint_task;

    if(waypointTask.mission_waypoint.empty() && exist_mission){
      std_srvs::SetBool vs_msg;
      vs_msg.request.data = true;
      auto vs_client = n.serviceClient<std_srvs::SetBool>("autonomous_landing/activate_visualservoing");
      vs_client.call(vs_msg);
      std::cout << "Activated VS" << std::endl;
      exist_mission = false;
      vs_activated = true;
    }
  //}
}

void HeightCallback(const std_msgs::Float32::ConstPtr& msg){
  cur_height = msg->data;
  if(vs_activated){
    if(cur_height < 1.5 && focus_trigger){
      set_focus_point();
      focus_trigger--;
    } else if( cur_height < 2 && focus_trigger == 2){
      set_focus_point();
      focus_trigger--;
    } else if( cur_height < 5 && focus_trigger == 3){
      set_focus_point();
      focus_trigger--;
    } else if( cur_height < 8 && focus_trigger == 4){
      set_focus_point();
      focus_trigger--;
    }
  }
}

void set_focus_point(){
  ros::NodeHandle n;

  auto camera_set_focus_point_client = n.serviceClient<CameraFocusPoint>("camera_task_set_focus_point");

  CameraFocusPoint cameraFocusPoint;
  cameraFocusPoint.request.payload_index = static_cast<uint8_t>(PayloadIndex::PAYLOAD_INDEX_0);
  cameraFocusPoint.request.x = CENTER_FOV;
  cameraFocusPoint.request.y = CENTER_FOV;
  camera_set_focus_point_client.call(cameraFocusPoint);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vs_manager_node");
  ros::NodeHandle nh;

  ros::Subscriber state_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &StateCallback);
  ros::Subscriber flight_mode_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);
  ros::Subscriber height_subscriber = nh.subscribe<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 1, &HeightCallback);

  ros::spin();
  
  return 0;
}
