/** @file mission_node.cpp
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief node of hotpoint 1.0/waypoint 1.0.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>
//#include <dji_osdk_ros/dji_waypoint.hpp>
#include <aerialcore_onboard_dji/dji_mission_node.h>
#include <aerialcore_common/ConfigMission.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>
using namespace DJI::OSDK;


// global variables
ros::ServiceClient     waypoint_upload_client;
ros::ServiceClient     waypoint_action_client;
ros::ServiceClient     flight_control_client;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber        gps_pos_subscriber;

float damping;
int uav_id =1;
float velocity_range;
float idle_velocity;
float start_altitude;
int yaw_mode = 3;  //0 automode, 1 lock as inicital value , 2 control Rc, 3 use waypoint's yaw
int trace_mode; // 0 point to point, after reaching the target waypoint hover, complete waypt action (if any), then fly to the next waypt 
                // 1: Coordinated turn mode, smooth transition between waypts, no waypts task 
int finish_action;// 0: no action, 1: return to home, 2: auto-landing, 3: return to point 0, 4: infinite mode, no exit
int landing_type = 0; 

// new variables to make robust the mission node adding yaw and gimbal pitch capabilities
int gimbal_pitch_mode = 1; // 0 free (no control on gimbal) , 1 auto (smooth transition between waypoints on gimbal)

// Variables to publish mission info
// Publisher for state_machine info
ros::Publisher command_mission_pub;
ros::Publisher upload_mission_pub;

bool lading_activated = false;
bool mission_started = false;
bool mission_status = false;   // becomes true when the mission started
bool was_on_air = false;
bool dont_more_missionwaypoints = true;

std_msgs::Bool command_mission_msg ;
std_msgs::Bool upload_mission_msg;

void StartRosbag()
{
  std::string id = std::to_string(uav_id);
  std::string bashscript ("rosbag record -O ~/bags/uav_"+ id +"_");

  char timeString[40];
  time_t t = time(0);
  struct tm tm = *localtime(&t);

  ROS_WARN("Start of ROS BAG");
  strftime(timeString, sizeof(timeString), "%Y_%m_%d_%H_%M", &tm);
  bashscript = bashscript +  timeString+ ".bag  -e \"/uav_"+ id +"/dji_osdk_ros/(.*)\" __name:=node_bag_uav"+id+" &";
  system( bashscript.c_str() );
}
void StopRosbag()
{
  std::string id = std::to_string(uav_id);
  std::string bashscript  = "rosnode kill node_bag_uav"+id;
  system( bashscript.c_str() );
  ROS_WARN("END of ROS BAG");
}
bool sendFiles(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
  ROS_WARN("Init to pass bag files ");
  std::string bashscript  = "sshpass -p 112358 rsync -ae ~/bags/ arpa@10.42.0.2:~/bags";
  system( bashscript.c_str() );
  res.success = true;
  res.message = "Success";
  return true;
}

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos = *msg;
}

bool runWaypointMission(std::vector<sensor_msgs::NavSatFix> gpsList, std_msgs::Float64MultiArray yawList,std_msgs::Float64MultiArray gimbalPitchList, int responseTimeout){
  ros::spinOnce();

  // Waypoint Mission : Initialization
  dji_osdk_ros::MissionWaypointTask waypointTask;
  setWaypointInitDefaults(waypointTask);

  // Waypoint Mission: Create Waypoints
  float32_t start_alt = start_altitude;
  ROS_INFO("Creating Waypoints..\n");

  std::vector<WayPointSettings> generatedWaypts =
  createWaypoints(gpsList,yawList,gimbalPitchList, start_alt);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result)
  {
    ROS_INFO("Waypoint upload command sent successfully");
    upload_mission_msg.data = true;
    upload_mission_pub.publish(upload_mission_msg);
  }
  else
  {
    ROS_WARN("Failed sending waypoint upload command");
    return false;
  }
  return true;
}

void setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = damping;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = velocity_range;
  waypointTask.idle_velocity      = idle_velocity;

  waypointTask.action_on_finish   = finish_action; //dji_osdk_ros::MissionWaypointTask::FINISH_AUTO_LANDING;
  if(finish_action == 1){
    waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_RETURN_TO_HOME;
    ROS_WARN("Mission final action: Return to home");
  }
  if(finish_action == 2){
    waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_AUTO_LANDING;
    ROS_WARN("Mission final action: Auto-landing");
  }
  if(finish_action == 3){
    waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_RETURN_TO_POINT;
    ROS_WARN("Mission final action: Return to first waypoint");
  }
  if(finish_action == 4){
    waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_NO_EXIT;
    ROS_WARN("Mission final action: Continue motion");
  }
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = yaw_mode; //dji_osdk_ros::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = trace_mode; //dji_osdk_ros::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_osdk_ros::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = 1;//GIMBAL_PITCH_FREEdji_osdk_ros::MissionWaypointTask::GIMBAL_PITCH_AUTO
}

std::vector<WayPointSettings>
createWaypoints(std::vector<sensor_msgs::NavSatFix> gpsList, std_msgs::Float64MultiArray yawList, std_msgs::Float64MultiArray gimbalPitchList,
                float32_t start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);
  start_wp.latitude  = gps_pos.latitude;
  start_wp.longitude = gps_pos.longitude;
  start_wp.altitude  = start_alt;
  ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
           gps_pos.longitude, start_alt);

  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // First waypoint
  start_wp.index = 0;
  wp_list.push_back(start_wp);

  // Iterative algorithm
  for (int i = 0; i < gpsList.size(); i++)
  {
    WayPointSettings  wp;
    setWaypointDefaults(&wp);

    wp.index     = i+1;
    wp.latitude  = gpsList[i].latitude;
    wp.longitude = gpsList[i].longitude;
    wp.altitude  = gpsList[i].altitude;
    wp.yaw = yawList.data[i];
    wp.gimbalPitch = gimbalPitchList.data[i];
    // Turn mode values:  0: clockwise, 1: counter-clockwise 
    if (wp.yaw > yawList.data[i-1])
      wp.turnMode           = 0; // depends on the yaw
    else{
      wp.turnMode           = 1;
    }
    wp_list.push_back(wp);
      ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f \t gimbal pitch: %d \t yaw: %d \n", wp.latitude,
           wp.longitude, wp.altitude, wp.gimbalPitch, wp.yaw);
  }
  
  return wp_list;
}

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int responseTimeout,
                     dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  dji_osdk_ros::MissionWaypoint waypoint;
  int i=0; //counter to choose the turn mode
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp, i++)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f \t gimbal pitch: %d \t yaw: %d \n", wp->latitude,
           wp->longitude, wp->altitude, wp->gimbalPitch, wp->yaw);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = wp->yaw;
    waypoint.target_gimbal_pitch = wp->gimbalPitch; // in orther to inspect in a good way, the gimbal pitch depends on the wp
    waypoint.turn_mode           = wp->turnMode; //0: clockwise, 1: counter-clockwise
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}

ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  dji_osdk_ros::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_client.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action)
{
  dji_osdk_ros::MissionWpAction missionWpAction;
  dji_osdk_ros::MissionHpAction missionHpAction;
  missionWpAction.request.action = action;
  waypoint_action_client.call(missionWpAction);
  if (!missionWpAction.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
              missionWpAction.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
  }

  return { missionWpAction.response.result,
            missionWpAction.response.cmd_set,
            missionWpAction.response.cmd_id,
            missionWpAction.response.ack_data };
}

bool takeoff()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_TAKEOFF;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool land()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_LAND;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool config_mission(aerialcore_common::ConfigMission::Request  &req,
         aerialcore_common::ConfigMission::Response &res){
  ROS_WARN("Received mission");
  std::vector<sensor_msgs::NavSatFix> gps_list = req.waypoint;
  std_msgs::Float64MultiArray yaw_list = req.yaw;  
  std_msgs::Float64MultiArray gimbal_pitch_list = req.gimbalPitch; //new to include gimbal pitch
  velocity_range = req.maxVel;
  idle_velocity = req.idleVel;
  yaw_mode = req.yawMode;
  trace_mode = req.traceMode;
  gimbal_pitch_mode = req.gimbalPitchMode; //new to include gimbal pitch
  finish_action = req.finishAction;
  ROS_WARN("Finish action: %d",finish_action);
  ROS_WARN("Gimbal Pitch Mode: %d",gimbal_pitch_mode);
  
  res.success = runWaypointMission(gps_list, yaw_list,gimbal_pitch_list, 1);
  return true;
}

bool run_mission(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res){
  ROS_WARN("Starting mission");

  bool input = req.data;
  bool response =false;
  std::string message ("Mission ");
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                    MISSION_ACTION::START)
        .result)
  {
    ROS_INFO("Mission start command sent successfully");
    command_mission_msg.data = true;
    command_mission_pub.publish(command_mission_msg);
    message += "start";
    response =true;
    StartRosbag();
    mission_status = true;
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    message += " fail ";
  }

  res.success = response;
  res.message = message;
  return true;
}

void flyStatusCallback(const std_msgs::UInt8::ConstPtr &msg)
{
  int flystatus = msg->data;//0 stoped //1 on_ground // 2 in air
  
  if( flystatus ==0 &&  mission_status == 1 && was_on_air==true && dont_more_missionwaypoints ==true){
    StopRosbag();
    was_on_air = false;
  }
  if (flystatus == 2){
    was_on_air = true;
  }
}

void ModeCallback(const std_msgs::UInt8::ConstPtr& msg){
  // msg->data   joystick? = 14
  //if(msg->data == 14 && is_flying && exist_mission){
    ros::NodeHandle n;
    dji_osdk_ros::MissionWpGetInfo mission_get_info;
    auto get_mission_info_client = n.serviceClient<dji_osdk_ros::MissionWpGetInfo>("dji_osdk_ros/mission_waypoint_getInfo");
    get_mission_info_client.call(mission_get_info);
    dji_osdk_ros::MissionWaypointTask waypointTask = mission_get_info.response.waypoint_task;

    if(waypointTask.idle_velocity != 0.0 && !mission_started){
      mission_started = true;
      landing_type = waypointTask.action_on_finish;
      ROS_WARN("LANDING TYPE DJI MISION NODE %d", landing_type);
    }
    if(waypointTask.mission_waypoint.empty() && !lading_activated && mission_started && landing_type == 2){
      ROS_WARN("Activated DJI LANDING");
      lading_activated = true;
      land();
    }

    if(waypointTask.mission_waypoint.empty()){
      dont_more_missionwaypoints = true;
    }else{
      dont_more_missionwaypoints = false;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle nh;

  nh.getParam("mission_node/damping", damping);
  nh.getParam("mission_node/start_altitude", start_altitude);
  nh.getParam("mission_node/uav_id", uav_id);

  command_mission_msg.data = false;
  upload_mission_msg.data = false;

  
  // ROS stuff
  waypoint_upload_client = nh.serviceClient<dji_osdk_ros::MissionWpUpload>("dji_osdk_ros/mission_waypoint_upload");
  waypoint_action_client = nh.serviceClient<dji_osdk_ros::MissionWpAction>("dji_osdk_ros/mission_waypoint_action");
  flight_control_client =nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");

  gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &gpsPosCallback);
  ros::Subscriber flight_mode_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/display_mode", 1, &ModeCallback);
  ros::Subscriber fly_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &flyStatusCallback);

  ros::ServiceServer service_config_mission = nh.advertiseService("dji_control/configure_mission", config_mission);
  ros::ServiceServer service_run_mission = nh.advertiseService("dji_control/start_mission", run_mission);
  ros::ServiceServer service_send_bags = nh.advertiseService("dji_control/send_bags", sendFiles);

  //info publishers 
  upload_mission_pub = nh.advertise<std_msgs::Bool>("dji_sm/upload_mission", 1);
  command_mission_pub = nh.advertise<std_msgs::Bool>("dji_sm/command_mission", 1);
  
  ros::spin();

  return 0;
}
