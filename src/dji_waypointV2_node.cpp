/** @file mission_node.cpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief node of waypoint V2.0.
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

#include <waypointV2/dji_waypointV2_node.h>
#include <aerialcore_common/ConfigMission.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>

// For media management we will need some extra libraries
#include <sys/stat.h> 
#include <iostream> 
// For folder names including time references
#include <chrono>
#include <ctime>

#include <std_srvs/SetBool.h>

// Our changes goes from here:
//Global Variables:
std::vector<sensor_msgs::NavSatFix> gpsList_global;
std_msgs::Float64MultiArray yaw_list_global;
std_msgs::Float64MultiArray gimbal_pitch_list_global;
std_msgs::Float64MultiArray speed_global;
std::vector<bool> take_a_photo;
std::vector<bool> start_recording;
std::vector<bool> stop_recording;

int velocity_range;
int idle_velocity;
int finish_action;
int yaw_mode_global;
int actionNumber;

std::time_t start_time;
std::time_t end_time;
bool mission_status = false;   // becomes true when the mission started
bool was_on_air = false;
int uav_id=14;

// Bags management
void StartRosbag()
{
  ROS_WARN("Starting ROS bag of uav:");
  //rosbag record -O ~/bags/uav_14_2021_01_20_15_00.bag  -e "/uav_14/dji_osdk_ros/(.*)" __name:=node_bag_uav1 &
  std::string id = std::to_string(uav_id);
  std::string bashscript ("rosbag record -O ~/bags/uav_"+ id +"_");
  char timeString[40];
  time_t t = time(0);
  struct tm tm = *localtime(&t);
  
  ROS_WARN("Start of ROS BAG");
  strftime(timeString, sizeof(timeString), "%Y_%m_%d_%H_%M", &tm);
  bashscript = bashscript +  timeString+ ".bag  -e \"/uav_"+ id +"/dji_osdk_ros/(.*)\" __name:=uav"+id+"_node_bag &";
  system( bashscript.c_str() );
}
void StopRosbag()
{
  std::string id = std::to_string(uav_id);
  std::string bashscript  = "rosnode kill uav"+id+"_node_bag";
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

class tme
{
public:
  tme(std::string& a)          // extracts the time date for further customization
    :day{a.substr(0,3),a.substr(8,2)},month{a.substr(4,3)},year{a.substr(20,4)}
  {
    tie = a.substr(11, 8);
  }
  
  std::string day[2]{}; //has day details 
 std::string month{}; // month details
 std::string year{}; //year details
  std::string tie{}; //time details
};

// Creation of the waypoints depending on what the user wants
std::vector<dji_osdk_ros::WaypointV2> createWaypoints(ros::NodeHandle &nh,std::vector<sensor_msgs::NavSatFix> gpsList)
{
  // Let's create a vector to store our waypoints in.
  std::vector<dji_osdk_ros::WaypointV2> waypointList;
  dji_osdk_ros::WaypointV2 startPoint;
  dji_osdk_ros::WaypointV2 waypointV2;
  
  /*startPoint.latitude  = gps_position_.latitude * C_PI / 180.0;
  startPoint.longitude = gps_position_.longitude * C_PI / 180.0;
  startPoint.relativeHeight = gpsList[0].altitude;
  setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);*/

  // Iterative algorithm
  for (int i = 0; i < gpsList.size(); i++) {
    
    setWaypointV2Defaults(waypointV2);
    /*TEST to try different flightpaths
    if(i<=3){
      waypointV2.waypointType = dji_osdk_ros::DJIWaypointV2FlightPathModeGoToPointAlongACurve;
    }
    
    if(i>(gpsList.size()-2)){
      waypointV2.waypointType = dji_osdk_ros::DJIWaypointV2FlightPathModeGoToPointAlongACurveAndStop;
    }*/
    

    waypointV2.config.useLocalCruiseVel = 1;
    waypointV2.config.useLocalMaxVel = 1;
    waypointV2.maxFlightSpeed= velocity_range;//TBD: velocity_range;
    // Simple TEST for varying the velocity between waypoints
    waypointV2.autoFlightSpeed = speed_global.data[i];
    waypointV2.latitude       = gpsList[i].latitude * C_PI / 180.0;
    waypointV2.longitude      = gpsList[i].longitude * C_PI / 180.0;
    waypointV2.relativeHeight = gpsList[i].altitude;
    
    
    // Heading management
    //waypointV2.heading = yaw_list_global.data[i];
    if (yaw_list_global.data[i]< yaw_list_global.data[i+1] && i <= gpsList.size())
      waypointV2.turnMode           = 0; // depends on the yaw
    else{
      waypointV2.turnMode           = 1;
    }
    waypointList.push_back(waypointV2);
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f ", waypointV2.latitude, waypointV2.longitude, waypointV2.relativeHeight); // add more info when advances come

  }
  //waypointList.push_back(startPoint); // starts and end at the same point

  return waypointList;
}

// Creation of the waypoints depending on what the user wants
std::vector<dji_osdk_ros::WaypointV2> resetWaypoints(ros::NodeHandle &nh,std::vector<sensor_msgs::NavSatFix> gpsList)
{
  // Let's create a vector to store our waypoints in.
  std::vector<dji_osdk_ros::WaypointV2> waypointList;
  
  dji_osdk_ros::WaypointV2 waypointV2;
  
  

  // Iterative algorithm
  for (int i = 0; i < gpsList.size(); i++) {
    
    setWaypointV2Defaults(waypointV2);
    

  }
  //waypointList.push_back(startPoint); // starts and end at the same point

  return waypointList;
}

// Three kind of actions: Camera, Gimbal and Heading
bool generateWaypointV2AllActions(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector_camera;
    dji_osdk_ros::WaypointV2Action actionVector_gimbal;
    dji_osdk_ros::WaypointV2Action actionVector_heading;
    int id=0; // counter of actions for do actions right
    // First of all we start with the camera actions, in our case we need to start recording in the first waypoint, 
    // could be interesting testing if we can do photos at the same time
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = 0;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);
    
    id+=1;
    for (uint16_t i = 0; i <= gpsList_global.size(); i++)
    {
      // If it is not the first wp, we start doing photos
      /*actionVector.actionId  = i;
      actionVector.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
      actionVector.waypointV2CameraActuator.actuatorIndex = 0;
      actionVector.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto;
      generateWaypointV2Action_.request.actions.push_back(actionVector);
      */
      // Gimbal control, we need to use different IDs for the actions obviously
      actionVector_gimbal.actionId  = id;//+ actionNum + 1;
      actionVector_gimbal.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint; // Good for now
      actionVector_gimbal.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector_gimbal.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector_gimbal.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeGimbal;
      // We are gonna rotate the gimbal somehow so we need to set this operation type
      actionVector_gimbal.waypointV2GimbalActuator.DJIWaypointV2ActionActuatorGimbalOperationType = dji_osdk_ros::WaypointV2GimbalActuator::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal;
      actionVector_gimbal.waypointV2GimbalActuator.actuatorIndex = 0;
      // Gimbal Parameters
      // Gimbal roll angle
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.x = 0; 
      // Gimbal pitch angle
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y = 10*gimbal_pitch_list_global.data[i]; // TBD: Change it acording to user needs -> gimbal_pitch_list_global.data[i];
      // Gimbal yaw angle
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.z = 0;//10*yaw_list_global.data[i]; 

      // Gimbal Control mode
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.ctrl_mode = 0; // 0: absolute angle, 1: relative angle

      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.rollCmdIgnore = 1;
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.pitchCmdIgnore = 0;
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.yawCmdIgnore = 1;

      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.absYawModeRef = 1; //0: relative to the aircraft, 1: relative to North
   
      // Gimbal Control speed
      actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.duationTime = 10; // rotate time
      id+=1;  
      ROS_INFO("Gimbal action created with ID: %d at wp: %d and angle %d", actionVector_gimbal.actionId, actionVector_gimbal.waypointV2SampleReachPointTrigger.waypointIndex, actionVector_gimbal.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y); // add more info when advances come

      generateWaypointV2Action_.request.actions.push_back(actionVector_gimbal);


      // Heading control
      actionVector_heading.actionId  = id;//*2 + 1;
      actionVector_heading.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector_heading.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector_heading.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector_heading.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
      // Config of the aircraft control (our case yaw angles)
      actionVector_heading.waypointV2AircraftControlActuator.actuatorIndex = 0;
      actionVector_heading.waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw;
      actionVector_heading.waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.isRelative = 0;
      actionVector_heading.waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw = yaw_list_global.data[i]; // works with manual mode

      ROS_INFO("Heading action created with ID: %d at wp: %d and angle %d ", actionVector_heading.actionId, actionVector_heading.waypointV2SampleReachPointTrigger.waypointIndex, actionVector_heading.waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw); // add more info when advances come
      id+=1;  
      generateWaypointV2Action_.request.actions.push_back(actionVector_heading);


    }
    // Stop recording the video
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = actionNum;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);
    id+=1;
    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}

bool generateWaypointV2AllActions_(ros::NodeHandle &nh, uint16_t actionNum, int &lastActionID)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector_camera;
    //dji_osdk_ros::WaypointV2Action actionVector_gimbal;
    //dji_osdk_ros::WaypointV2Action actionVector_heading;
    auto *action = new dji_osdk_ros::WaypointV2Action;
    int id=0;//lastActionID; // counter of actions for do actions right
    
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = 1;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);
    id++;
    lastActionID++;

    for (uint16_t j = 0; j < gpsList_global.size(); j++)
    {
      // Heading control
      action->actionId  = id;//*2 + 1;
      action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint; // Good for now
      action->waypointV2SampleReachPointTrigger.waypointIndex = j;
      action->waypointV2SampleReachPointTrigger.terminateNum = 0;

      action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
      // Config of the aircraft control (our case yaw angles)
      action->waypointV2AircraftControlActuator.actuatorIndex = 0;
      action->waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw;
      action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.isRelative = 0;
      action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw = yaw_list_global.data[j]; // works with manual mode

      ROS_INFO("Heading action created with ID: %d at wp: %d and angle %f ", action->actionId, action->waypointV2SampleReachPointTrigger.waypointIndex, yaw_list_global.data[j]);//action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw); // add more info when advances come
      id+=1; 
      lastActionID++; 
      generateWaypointV2Action_.request.actions.push_back(*action);
      delete action;
      action = new dji_osdk_ros::WaypointV2Action;

    }
    
    for (uint16_t i = 0; i < gpsList_global.size(); i++)
    {
      
      // Gimbal control, we need to use different IDs for the actions obviously
      action->actionId  = id;//+ actionNum + 1;

      // action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
      // action->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised;
      // action->waypointV2AssociateTrigger.waitingTime = 0;
      // action->waypointV2AssociateTrigger.actionIdAssociated = i+1;
      
      action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint; // Good for now
      action->waypointV2SampleReachPointTrigger.waypointIndex = i;
      action->waypointV2SampleReachPointTrigger.terminateNum = 0;

      action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeGimbal;
      // We are gonna rotate the gimbal somehow so we need to set this operation type
      action->waypointV2GimbalActuator.DJIWaypointV2ActionActuatorGimbalOperationType = dji_osdk_ros::WaypointV2GimbalActuator::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal;
      action->waypointV2GimbalActuator.actuatorIndex = 0;
      // Gimbal Parameters
      // Gimbal roll angle
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.x = 0; 
      // Gimbal pitch angle
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y = 10*gimbal_pitch_list_global.data[i]; // TBD: Change it acording to user needs -> gimbal_pitch_list_global.data[i];
      // Gimbal yaw angle
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.z = 0;//10*yaw_list_global.data[i]; 

      // Gimbal Control mode
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.ctrl_mode = 0; // 0: absolute angle, 1: relative angle

      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.rollCmdIgnore = 1;
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.pitchCmdIgnore = 0;
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.yawCmdIgnore = 1;

      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.absYawModeRef = 1; //0: relative to the aircraft, 1: relative to North
   
      // Gimbal Control speed
      action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.duationTime = 20; // rotate time
      id+=1; 
      lastActionID++; 
      ROS_INFO("Gimbal action created with ID: %d associated to action: %d and with angle %d", action->actionId, action->waypointV2AssociateTrigger.actionIdAssociated, action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y); // add more info when advances come

      generateWaypointV2Action_.request.actions.push_back(*action);
      delete action;
      action = new dji_osdk_ros::WaypointV2Action;
      

    }
    // taking photos depending on the action value
    for(uint16_t k = 0; k < gpsList_global.size(); k++)
    {
      if (take_a_photo[k] == true)
      {
        action->actionId  = id;
        action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
        action->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised;
        action->waypointV2AssociateTrigger.waitingTime = 0;
        action->waypointV2AssociateTrigger.actionIdAssociated = k+1;

        action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
        action->waypointV2CameraActuator.actuatorIndex = 0;
        action->waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto;
        
        generateWaypointV2Action_.request.actions.push_back(*action);
        delete action;
        id+=1; 

        action = new dji_osdk_ros::WaypointV2Action;
      }else{
        continue;
      }
         
    }
    // if we want to take a picture in that exact waypoint we need to wait until the gimbal is in the correct position, we estimate more or less 2 seconds
    for(uint16_t l = 0; l < gpsList_global.size(); l++)
    {
      if (take_a_photo[l] == true)
      {
        // Stop the aircraft at waypoint 2
        action->actionId  = id;
        action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
        action->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeSimultaneously;
        action->waypointV2AssociateTrigger.waitingTime = 3;
        action->waypointV2AssociateTrigger.actionIdAssociated = l+1;

        action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
        // Config of the aircraft control (in this case stop or start flying)
        action->waypointV2AircraftControlActuator.actuatorIndex = 0;
        action->waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
        action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorFlying.isStartFlying = 0;
        
        generateWaypointV2Action_.request.actions.push_back(*action);
        delete action;
        id+=1; 
        action = new dji_osdk_ros::WaypointV2Action;

        // Start flying again the aircraft at waypoint 2
        action->actionId  = id;
        action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
        action->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised;
        action->waypointV2AssociateTrigger.waitingTime = 0;
        action->waypointV2AssociateTrigger.actionIdAssociated = id-1;

        action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
        // Config of the aircraft control (in this case stop or start flying)
        action->waypointV2AircraftControlActuator.actuatorIndex = 0;
        action->waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
        action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorFlying.isStartFlying = 1;

        generateWaypointV2Action_.request.actions.push_back(*action);
        delete action;
        id+=1; 

        action = new dji_osdk_ros::WaypointV2Action;
        
      }else{
        continue;
      }
         
    }
    
    
        



    //Stop recording video
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = gpsList_global.size()-1;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);
    lastActionID++;
    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    // after the call we need to reset the action vector
    generateWaypointV2Action_.request.actions.clear();

    return generateWaypointV2Action_.response.result;
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
}

void flyStatusCallback(const std_msgs::UInt8::ConstPtr &msg)
{
  int flystatus = msg->data;//0 stoped //1 on_ground // 2 in air
  
  /*if( flystatus ==0 &&  mission_status == 1 && was_on_air==true){ 
    StopRosbag();
    was_on_air = false;
    // Getting the time for the folder name
    auto r=std::chrono::system_clock::now();
    auto rp=std::chrono::system_clock::to_time_t(r);
    std::string h(ctime(&rp)); //converting to c++ string
    tme curtime(h);   // creating a tme object
    std::string user;
    // We create the folder name and then the folder
    //we need to include the username to the folder name, FIX
    char const* usr = getenv( "USER" );
    if ( usr == NULL ) {
        ROS_ERROR("EEPA Error: $USER not set\n");
    } else {
        user = std::string( usr );
        //  ...
    }

    std::string foldername = "/home/"+ user + "uav_media" + "_" + curtime.day[0] + "_" + curtime.day[1] + "_" + curtime.month + "_" + curtime.year + "_" + curtime.tie;
    
    const char *cstr = foldername.c_str();
    int check = mkdir(cstr,0777);

    if (!check)
        ROS_INFO("Media Directory created successfully");
    else {
        ROS_ERROR("Unable to create Media directory");
        //exit(1);
    }


  }
  if (flystatus == 2){
    was_on_air = true;
  }*/
}

// A class to improve the quality of the code:
class WaypointV2Node{
  public:
    WaypointV2Node() {
	
    actionIDCounter=0; 
      nh.getParam("waypointV2_node/uav_id", uav_id);

     // Our changes goes from here:
    service_config_mission = nh.advertiseService("dji_control/configure_mission", &WaypointV2Node::configMission,this);
    gpsPositionSub = nh.subscribe<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
    fly_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_osdk_ros/flight_status", 1, &flyStatusCallback);

    obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
      "obtain_release_control_authority");

    //if you want to fly without rc ,you need to obtain ctrl authority.Or it will enter rc lost.
    obtainCtrlAuthority.request.enable_obtain = true;
    obtain_ctrl_authority_client.call(obtainCtrlAuthority);

   
    ROS_WARN("WaypointV2Node started");
    service_run_mission = nh.advertiseService("dji_control/start_mission",&WaypointV2Node::startWaypointV2Mission,this);
    //ros::ServiceServer service_send_bags = nh.advertiseService("dji_control/send_bags", sendFiles);


    };
    ~WaypointV2Node(){};
  private:
    // ROS stuff
    ros::NodeHandle nh;
    ros::ServiceServer service_config_mission;
    ros::Subscriber gpsPositionSub;
    ros::Subscriber fly_status_subscriber;
   
    ros::ServiceClient obtain_ctrl_authority_client;
    ros::ServiceServer service_run_mission;
    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
    // WaypointV2  variables
    int actionIDCounter;

    // Configuration of the mission obtained from the .YAML file
    bool configMission(aerialcore_common::ConfigMission::Request  &req,
            aerialcore_common::ConfigMission::Response &res){
      ROS_WARN("Received mission");

      //ros::NodeHandle nodehandler;

      
      gpsList_global = req.waypoint; // WORKS
      yaw_list_global = req.yaw; // WORKS
      yaw_mode_global = req.yawMode; // TBD
      gimbal_pitch_list_global = req.gimbalPitch; // TEST:  to include gimbal pitch
      velocity_range = req.maxVel;
      idle_velocity = req.idleVel;
      finish_action = req.finishAction;

      //varying velocity
      speed_global = req.speed;

      // actions functionality
      std_msgs::Float64MultiArray acommandList = req.commandList; //TBD 
      std_msgs::Float64MultiArray acommandParameter = req.commandParameter; //TBD
      actionNumber=0;
      /*
      *WP_ACTION_STAY                 = 0,  /*!< no action.uint of action parameter:ms
      WP_ACTION_SIMPLE_SHOT          = 1,  /*!< take picture action.action parameters Action parameter have no effect.limit time:6s
      WP_ACTION_VIDEO_START          = 2,  /*!< start take video action.action parameters Action parameter have no effect.limit time:6s
      WP_ACTION_VIDEO_STOP           = 3,  /*!< stop video action.action parameters Action parameter have no effect.limit time:6s
      WP_ACTION_CRAFT_YAW            = 4,  /*!< craft control yaw action.uint of action parameter:degree. range:-180 ~ 180
      WP_ACTION_GIMBAL_PITCH         = 5,  
      * 
      */
      // we must initialize the boolean vectors to false
      take_a_photo.resize(gpsList_global.size(),false);
      start_recording.resize(gpsList_global.size(),false);
      stop_recording.resize(gpsList_global.size(),false);
      
      // We need to create a vector of actions
      if (acommandList.data.size() != acommandParameter.data.size()){
        ROS_ERROR("The number of actions and parameters is not the same");
        return false;
      }

      // We need to be sure that exists actions
      if (acommandList.data.size() >= 0){
        for (int i = 0; i < gpsList_global.size(); i++){
          for (int j = 0; j < 10; j++)
          {
            // counting the number of actions
            if (acommandList.data[i*10+j]) actionNumber++;
            switch (int(acommandList.data[i*10+j]))
            {
            case 1:// take a photo
              take_a_photo[i] = true;
              break;
            case 2:// start recording
              start_recording[i] = true;
              break;
            case 3:// stop recording
              stop_recording[i] = true; 
              break;
            case 4:// craft control yaw
              yaw_list_global.data[i] = acommandParameter.data[i*10+j];
              break;
            case 5:// gimbal pitch 
              gimbal_pitch_list_global.data[i] = acommandParameter.data[i*10+j];
              break;
            default:
              break;
            }
            ;
            

          }
        }
      }
      
      ROS_WARN("Total number of actions: %d", actionNumber);
        
      ROS_WARN("Last ActionID: %d",this->actionIDCounter);
      ROS_WARN("Finish action: %d",finish_action);

      // if everything goes right we run the whole thing
      res.success = runWaypointV2Mission(this->nh,this->actionIDCounter);
      return true;
    }
    bool startWaypointV2Mission(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res)
    {
      bool response =false;
      std::string message ("Mission ");

        waypointV2_start_mission_client = this->nh.serviceClient<dji_osdk_ros::StartWaypointV2Mission>("dji_osdk_ros/waypointV2_startMission");
        waypointV2_start_mission_client.call(startWaypointV2Mission_);

        if(startWaypointV2Mission_.response.result)
        {
          StartRosbag();
          ROS_INFO("Start waypoint v2 mission successfully!\n");
          
          mission_status = true;
          start_time = std::time(0);
          response =true;
        }
        else
        {
          ROS_ERROR("Start waypoint v2 mission failed!\n");
          message += " fail ";
        }

        //return startWaypointV2Mission_.response.result;
        res.success = response;
        res.message = message;
        return true;
    }
    


};
// Gimbal control management
bool generateGimbalActions(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector;
    int id=0;
    // array with 4 gimbal values
    int array[9] = {-300, 10, -400, -100,0,0,0,0,0};
    for (uint16_t i = 0; i <= gpsList_global.size(); i++)
    {
      actionVector.actionId  = id; // to be different than the camera actions 
      actionVector.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint; // Good for now
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeGimbal;
      
      // We are gonna rotate the gimbal somehow so we need to set this operation type
      actionVector.waypointV2GimbalActuator.DJIWaypointV2ActionActuatorGimbalOperationType = dji_osdk_ros::WaypointV2GimbalActuator::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal;
      actionVector.waypointV2GimbalActuator.actuatorIndex = 0;
      // Gimbal Parameters
      // Gimbal roll angle
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.x = 0; 
      // Gimbal pitch angle
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y = array[i];//10*gimbal_pitch_list_global.data[i]; // TBD: Change it acording to user needs -> gimbal_pitch_list_global.data[i];
      // Gimbal yaw angle
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.z = 0; //10*yaw_list_global.data[i]; 

      // Gimbal Control mode
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.ctrl_mode = 1; // 0: absolute angle, 1: relative angle

      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.rollCmdIgnore = 0;
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.pitchCmdIgnore = 0;
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.yawCmdIgnore = 0;

      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.absYawModeRef = 1; //0: relative to the aircraft, 1: relative to North
   
      // Gimbal Control speed
      actionVector.waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.duationTime = 10; // rotate time

      generateWaypointV2Action_.request.actions.push_back(actionVector);
      id++;
    }

    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}

bool generateHeadingV2Actions(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector;
    for (uint16_t i = 0; i <= actionNum; i++)
    {
      actionVector.actionId  = i+actionNum*2+1;
      actionVector.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i+1;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
      // Config of the aircraft control (our case yaw angles)
      actionVector.waypointV2AircraftControlActuator.actuatorIndex = 0;
      actionVector.waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw;
      actionVector.waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.isRelative = 0;
      actionVector.waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw = yaw_list_global.data[i]; // works with manual mode

      generateWaypointV2Action_.request.actions.push_back(actionVector);
    }

    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}



// TBD: Know the functionalities of the Events
void waypointV2MissionEventSubCallback(const dji_osdk_ros::WaypointV2MissionEventPush::ConstPtr& waypointV2MissionEventPush)
{
  waypoint_V2_mission_event_push_ = *waypointV2MissionEventPush;

  ROS_INFO("waypoint_V2_mission_event_push_.event ID :0x%x\n", waypoint_V2_mission_event_push_.event);

  if(waypoint_V2_mission_event_push_.event == 0x01)
  {
    ROS_INFO("interruptReason:0x%x\n", waypoint_V2_mission_event_push_.interruptReason);
  }
  if(waypoint_V2_mission_event_push_.event == 0x02)
  {
    ROS_INFO("recoverProcess:0x%x\n", waypoint_V2_mission_event_push_.recoverProcess);
  }
  if(waypoint_V2_mission_event_push_.event== 0x03)
  {
    ROS_INFO("finishReason:0x%x\n", waypoint_V2_mission_event_push_.finishReason);
    if (mission_status){
      StopRosbag();
    
      // Getting the time for the folder name
      auto r=std::chrono::system_clock::now();
      auto rp=std::chrono::system_clock::to_time_t(r);
      std::string h(ctime(&rp)); //converting to c++ string
      tme curtime(h);   // creating a tme object
      struct tm date_tm; 
      char timeString[40];
      time_t t = time(0);
      struct tm tm = *localtime(&t);
      
      strftime(timeString, sizeof(timeString), "%Y_%m_%d_%H_%M", &tm);
      
      
      //std::string bashscript ("mkdir -p ~/uav_media/mission_" + curtime.day[0] + "_" + curtime.day[1] + "_" + curtime.month + "_" + curtime.year + "_" + curtime.tie);

      
      std::string bashscript ("mkdir -p ~/uav_media/mission_");
      bashscript = bashscript +  timeString;
      system( bashscript.c_str() );
      mission_status=false;
    }
          
      

    
  }

  if(waypoint_V2_mission_event_push_.event == 0x10)
  {
    ROS_INFO("current waypointIndex:%d\n", waypoint_V2_mission_event_push_.waypointIndex);
  }

  if(waypoint_V2_mission_event_push_.event == 0x11)
  {
    ROS_INFO("currentMissionExecNum:%d\n", waypoint_V2_mission_event_push_.currentMissionExecNum);
  }
}

// This function Show the mission state
void waypointV2MissionStateSubCallback(const dji_osdk_ros::WaypointV2MissionStatePush::ConstPtr& waypointV2MissionStatePush)
{
  waypoint_V2_mission_state_push_ = *waypointV2MissionStatePush;

  /*ROS_INFO("waypointV2MissionStateSubCallback");
  ROS_INFO("missionStatePushAck->commonDataVersion:%d\n",waypoint_V2_mission_state_push_.commonDataVersion);
  ROS_INFO("missionStatePushAck->commonDataLen:%d\n",waypoint_V2_mission_state_push_.commonDataLen);
  ROS_INFO("missionStatePushAck->data.state:0x%x\n",waypoint_V2_mission_state_push_.state);
  ROS_INFO("missionStatePushAck->data.curWaypointIndex:%d\n",waypoint_V2_mission_state_push_.curWaypointIndex);
  ROS_INFO("missionStatePushAck->data.velocity:%d\n",waypoint_V2_mission_state_push_.velocity);*/
}
//TBD: Better understanding
//TBD: modify this function with the parameters given by the user from the GCS
void setWaypointV2Defaults(dji_osdk_ros::WaypointV2& waypointV2)
{
  waypointV2.waypointType = dji_osdk_ros::DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  waypointV2.headingMode = dji_osdk_ros::DJIWaypointV2HeadingManual;//dji_osdk_ros::DJIWaypointV2HeadingGimbalYawFollow;//DJIWaypointV2HeadingWaypointCustom;//DJIWaypointV2HeadingModeAuto; // TBD: Change it acording to user needs
  waypointV2.config.useLocalCruiseVel = 0;
  waypointV2.config.useLocalMaxVel = 0;

  waypointV2.dampingDistance = 40;
  waypointV2.heading = 0;
  waypointV2.turnMode = dji_osdk_ros::DJIWaypointV2TurnModeClockwise;

  waypointV2.positionX = 0;
  waypointV2.positionY = 0;
  waypointV2.positionZ = 0;
  waypointV2.maxFlightSpeed= 10;
  waypointV2.autoFlightSpeed = 2;
}

std::vector<dji_osdk_ros::WaypointV2> generatePolygonWaypoints(ros::NodeHandle &nh, float32_t radius, uint16_t polygonNum)
{
  // Let's create a vector to store our waypoints in.
  std::vector<dji_osdk_ros::WaypointV2> waypointList;
  dji_osdk_ros::WaypointV2 startPoint;
  dji_osdk_ros::WaypointV2 waypointV2;

  startPoint.latitude  = gps_position_.latitude * C_PI / 180.0;
  startPoint.longitude = gps_position_.longitude * C_PI / 180.0;
  startPoint.relativeHeight = 15;
  setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);

  // Iterative algorithm
  for (int i = 0; i < polygonNum; i++) {
    float32_t angle = i * 2 * M_PI / polygonNum;
    setWaypointV2Defaults(waypointV2);
    float32_t X = radius * cos(angle);
    float32_t Y = radius * sin(angle);
    waypointV2.latitude = Y/EARTH_RADIUS + startPoint.latitude;
    waypointV2.longitude = X/(EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
    waypointV2.relativeHeight = startPoint.relativeHeight ;
    waypointList.push_back(waypointV2);
  }
  waypointList.push_back(startPoint);

  return waypointList;
}

bool initWaypointV2Setting(ros::NodeHandle &nh, int &actionIDCounter)
{
    waypointV2_init_setting_client = nh.serviceClient<dji_osdk_ros::InitWaypointV2Setting>("dji_osdk_ros/waypointV2_initSetting");
    initWaypointV2Setting_.request.polygonNum = 6;
    initWaypointV2Setting_.request.radius = 6;
    initWaypointV2Setting_.request.actionNum = actionNumber;//2*gpsList_global.size();//TBD: Change it acording to the number of actions given by the user

    /*! Generate actions*/
    //generateWaypointV2Actions(nh, initWaypointV2Setting_.request.actionNum);
    //generateGimbalActions(nh, initWaypointV2Setting_.request.actionNum);
    //generateHeadingV2Actions(nh, initWaypointV2Setting_.request.actionNum);

    // We rock here
    generateWaypointV2AllActions_(nh, initWaypointV2Setting_.request.actionNum, actionIDCounter);
    //generateWaypointV2AllActionsKylie(nh, initWaypointV2Setting_.request.actionNum);

    // Configure General Init Settings
    initWaypointV2Setting_.request.waypointV2InitSettings.repeatTimes = 1;
    initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction = finish_action;//initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionFinishedGoHome;
    initWaypointV2Setting_.request.waypointV2InitSettings.maxFlightSpeed = velocity_range;//10;
    initWaypointV2Setting_.request.waypointV2InitSettings.autoFlightSpeed = idle_velocity;
    initWaypointV2Setting_.request.waypointV2InitSettings.exitMissionOnRCSignalLost = 1;
    initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode = initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
    //initWaypointV2Setting_.request.waypointV2InitSettings.mission = generatePolygonWaypoints(nh, initWaypointV2Setting_.request.radius, initWaypointV2Setting_.request.polygonNum);
    initWaypointV2Setting_.request.waypointV2InitSettings.mission = createWaypoints(nh, gpsList_global);
    
    initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen = initWaypointV2Setting_.request.waypointV2InitSettings.mission.size();

    waypointV2_init_setting_client.call(initWaypointV2Setting_);
    if (initWaypointV2Setting_.response.result)
    {
      ROS_INFO("Init mission setting successfully!\n");
    }
    else
    {
      ROS_ERROR("Init mission setting failed!\n");
    }

    return initWaypointV2Setting_.response.result;

}

bool uploadWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_upload_mission_client = nh.serviceClient<dji_osdk_ros::UploadWaypointV2Mission>("dji_osdk_ros/waypointV2_uploadMission");
    waypointV2_upload_mission_client.call(uploadWaypointV2Mission_);

    if(uploadWaypointV2Mission_.response.result)
    {
      ROS_INFO("Upload waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Upload waypoint v2 mission failed!\n");
    }

    return uploadWaypointV2Mission_.response.result;
}

bool uploadWaypointV2Action(ros::NodeHandle &nh)
{
    waypointV2_upload_action_client = nh.serviceClient<dji_osdk_ros::UploadWaypointV2Action>("dji_osdk_ros/waypointV2_uploadAction");
    waypointV2_upload_action_client.call(uploadWaypointV2Action_);

    if(uploadWaypointV2Action_.response.result)
    {
      ROS_INFO("Upload waypoint v2 actions successfully!\n");
    }
    else
    {
      ROS_ERROR("Upload waypoint v2 actions failed!\n");
    }

    return uploadWaypointV2Action_.response.result;
}

bool downloadWaypointV2Mission(ros::NodeHandle &nh, std::vector<dji_osdk_ros::WaypointV2> &mission)
{
    waypointV2_download_mission_client = nh.serviceClient<dji_osdk_ros::DownloadWaypointV2Mission>("dji_osdk_ros/waypointV2_downloadMission");
    waypointV2_download_mission_client.call(downloadWaypointV2Mission_);
    mission = downloadWaypointV2Mission_.response.mission;

    if(downloadWaypointV2Mission_.response.result)
    {
      ROS_INFO("Download waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Download waypoint v2 mission failed!\n");
    }

    return downloadWaypointV2Mission_.response.result; 
}



bool stopWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_stop_mission_client = nh.serviceClient<dji_osdk_ros::StopWaypointV2Mission>("dji_osdk_ros/waypointV2_stopMission");
    waypointV2_stop_mission_client.call(stopWaypointV2Mission_);

    if(stopWaypointV2Mission_.response.result)
    {
      ROS_INFO("Stop waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Stop waypoint v2 mission failed!\n");
    }

    return stopWaypointV2Mission_.response.result;
}

bool pauseWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_pause_mission_client = nh.serviceClient<dji_osdk_ros::PauseWaypointV2Mission>("dji_osdk_ros/waypointV2_pauseMission");
    waypointV2_pause_mission_client.call(pauseWaypointV2Mission_);

    if(pauseWaypointV2Mission_.response.result)
    {
      ROS_INFO("Pause waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Pause waypoint v2 mission failed!\n");
    }

    return pauseWaypointV2Mission_.response.result;
}

bool resumeWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_resume_mission_client = nh.serviceClient<dji_osdk_ros::ResumeWaypointV2Mission>("dji_osdk_ros/waypointV2_resumeMission");
    waypointV2_resume_mission_client.call(resumeWaypointV2Mission_);

    if(resumeWaypointV2Mission_.response.result)
    {
      ROS_INFO("Resume Waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Resume Waypoint v2 mission failed!\n");
    }

    return resumeWaypointV2Mission_.response.result;
}

bool generateWaypointV2Actions(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector;
    for (uint16_t i = 0; i < actionNum; i++)
    {
      actionVector.actionId  = i;
      actionVector.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
      actionVector.waypointV2CameraActuator.actuatorIndex = 0;
      actionVector.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto;
      generateWaypointV2Action_.request.actions.push_back(actionVector);
    }

    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}

bool setGlobalCruiseSpeed(ros::NodeHandle &nh, float32_t cruiseSpeed)
{
    waypointV2_set_global_cruisespeed_client = nh.serviceClient<dji_osdk_ros::SetGlobalCruisespeed>("dji_osdk_ros/waypointV2_setGlobalCruisespeed");
    setGlobalCruisespeed_.request.global_cruisespeed = cruiseSpeed;
    waypointV2_set_global_cruisespeed_client.call(setGlobalCruisespeed_);

    if(setGlobalCruisespeed_.response.result)
    {
      ROS_INFO("Current cruise speed is: %f m/s\n", cruiseSpeed);
    }
    else
    {
      ROS_ERROR("Set glogal cruise speed failed\n");
    }

    return setGlobalCruisespeed_.response.result;
}

float32_t getGlobalCruiseSpeed(ros::NodeHandle &nh)
{
    waypointV2_get_global_cruisespeed_client = nh.serviceClient<dji_osdk_ros::GetGlobalCruisespeed>("dji_osdk_ros/waypointV2_getGlobalCruisespeed");
    waypointV2_get_global_cruisespeed_client.call(getGlobalCruisespeed_);

    ROS_INFO("Current cruise speed is: %f m/s\n", getGlobalCruisespeed_.response.global_cruisespeed);

    return getGlobalCruisespeed_.response.global_cruisespeed;
}

bool runWaypointV2Mission(ros::NodeHandle &nh, int &actionIDCounter)
{
  int timeout = 1;
  bool result = false;

  // ROS Declarations
  get_drone_type_client = nh.serviceClient<dji_osdk_ros::GetDroneType>("get_drone_type");
  waypointV2_mission_state_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2Event>("dji_osdk_ros/waypointV2_subscribeMissionState");
  waypointV2_mission_event_push_client = nh.serviceClient<dji_osdk_ros::SubscribeWaypointV2State>("dji_osdk_ros/waypointV2_subscribeMissionEvent");

  waypointV2EventSub = nh.subscribe("dji_osdk_ros/waypointV2_mission_event", 10, &waypointV2MissionEventSubCallback);
  waypointV2StateSub = nh.subscribe("dji_osdk_ros/waypointV2_mission_state", 10, &waypointV2MissionStateSubCallback);

  subscribeWaypointV2Event_.request.enable_sub = true;
  subscribeWaypointV2State_.request.enable_sub = true;
 
  get_drone_type_client.call(drone_type);
  if (drone_type.response.drone_type != static_cast<uint8_t>(dji_osdk_ros::Dronetype::M300))
  {
      ROS_DEBUG("This node only works for the model DJI M300!\n");
      return false;
  }

  waypointV2_mission_state_push_client.call(subscribeWaypointV2State_);
  waypointV2_mission_event_push_client.call(subscribeWaypointV2Event_);

    /*! init mission */
    
  result = initWaypointV2Setting(nh,actionIDCounter);
  if(!result)
  {
    return false;
  }
  sleep(timeout);

  /*! upload mission */
  result = uploadWaypointV2Mission(nh);
  if(!result)
  {
    return false;
  }
  sleep(timeout);

 /*! download mission */
  std::vector<dji_osdk_ros::WaypointV2> mission;
  result = downloadWaypointV2Mission(nh, mission);
  if(!result)
  {
    return false;
  }
  sleep(timeout);

  /*! upload  actions */
  result = uploadWaypointV2Action(nh);
  if(!result)
  {
    return false;
  }
  sleep(timeout);
  std::vector<dji_osdk_ros::WaypointV2> resetMission;
  resetMission = resetWaypoints(nh, gpsList_global);

  
return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointV2_node");
  WaypointV2Node node;
 
  
  ros::spin();
  return 0;
}
