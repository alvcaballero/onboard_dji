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

// Our changes goes from here:
//Global Variables:
std::vector<sensor_msgs::NavSatFix> gpsList_global;
std_msgs::Float64MultiArray yaw_list_global;
std_msgs::Float64MultiArray gimbal_pitch_list_global;
int velocity_range;
int idle_velocity;
int finish_action;
int yaw_mode_global;
int actionNumber;

// Configuration of the mission obtained from the .YAML file
bool config_mission(aerialcore_common::ConfigMission::Request  &req,
         aerialcore_common::ConfigMission::Response &res){
  ROS_WARN("Received mission");

  ros::NodeHandle nodehandler;

  
  gpsList_global = req.waypoint; // WORKS
  yaw_list_global = req.yaw; // WORKS
  yaw_mode_global = req.yawMode; // TBD
  gimbal_pitch_list_global = req.gimbalPitch; // TEST:  to include gimbal pitch
  velocity_range = req.maxVel;
  idle_velocity = req.idleVel;
  finish_action = req.finishAction;
  
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
          // TBD
          break;
        case 2:// start recording
          // TBD
          break;
        case 3:// stop recording
          // TBD
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
  	
  ROS_WARN("Finish action: %d",finish_action);
  // if everything goes right we run the whole thing
  res.success = runWaypointV2Mission(nodehandler);
  return true;
}

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

bool generateWaypointV2AllActions_(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
    dji_osdk_ros::WaypointV2Action actionVector_camera;
    //dji_osdk_ros::WaypointV2Action actionVector_gimbal;
    //dji_osdk_ros::WaypointV2Action actionVector_heading;
    auto *action = new dji_osdk_ros::WaypointV2Action;
    int id=0;
    
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = 1;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);

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

      ROS_INFO("Heading action created with ID: %d at wp: %d and angle %d ", action->actionId, action->waypointV2SampleReachPointTrigger.waypointIndex, action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw); // add more info when advances come
      id+=1;  
      generateWaypointV2Action_.request.actions.push_back(*action);
      delete action;
      action = new dji_osdk_ros::WaypointV2Action;

    }
    
    for (uint16_t i = 0; i < gpsList_global.size(); i++)
    {
      
      // Gimbal control, we need to use different IDs for the actions obviously
      action->actionId  = id;//+ actionNum + 1;

      action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
      action->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised;
      action->waypointV2AssociateTrigger.waitingTime = 0;
      action->waypointV2AssociateTrigger.actionIdAssociated = i;

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
      ROS_INFO("Gimbal action created with ID: %d associated to action: %d and with angle %d", action->actionId, action->waypointV2AssociateTrigger.actionIdAssociated, action->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y); // add more info when advances come

      generateWaypointV2Action_.request.actions.push_back(*action);
      delete action;
      action = new dji_osdk_ros::WaypointV2Action;
      

    }
    //Stop recording video
    actionVector_camera.actionId  = id;
    actionVector_camera.waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
    actionVector_camera.waypointV2SampleReachPointTrigger.waypointIndex = 1;
    actionVector_camera.waypointV2SampleReachPointTrigger.terminateNum = 0;
    actionVector_camera.waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
    actionVector_camera.waypointV2CameraActuator.actuatorIndex = 0;
    actionVector_camera.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo;
    generateWaypointV2Action_.request.actions.push_back(actionVector_camera);

    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}
// Another test with the idea implemented in the examples of Kylie
bool generateWaypointV2AllActionsKylie(ros::NodeHandle &nh, uint16_t actionNum)
{
    waypointV2_generate_actions_client = nh.serviceClient<dji_osdk_ros::GenerateWaypointV2Action>("dji_osdk_ros/waypointV2_generateActions");
     
    auto *action = new dji_osdk_ros::WaypointV2Action;
    auto *gimbalAction = new dji_osdk_ros::WaypointV2Action;
    int id=0;
    
    for (uint16_t i = 0; i <= gpsList_global.size(); i++)
    {
      // Heading control
      action->actionId  = id;//*2 + 1;
      action->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint; // Good for now
      action->waypointV2SampleReachPointTrigger.waypointIndex = i;
      action->waypointV2SampleReachPointTrigger.terminateNum = 0;

      action->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl;
      // Config of the aircraft control (our case yaw angles)
      action->waypointV2AircraftControlActuator.actuatorIndex = 0;
      action->waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType = dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw;
      action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.isRelative = 0;
      action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw = yaw_list_global.data[i]; // works with manual mode

      ROS_INFO("Heading action created with ID: %d at wp: %d and angle %d ", action->actionId, action->waypointV2SampleReachPointTrigger.waypointIndex, action->waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw); // add more info when advances come
      id+=1;  
      generateWaypointV2Action_.request.actions.push_back(*action);
      
      delete action;
      auto *gimbalAction = new dji_osdk_ros::WaypointV2Action;

         
      // Gimbal control, we need to use different IDs for the actions obviously
      gimbalAction->actionId  = id;//+ actionNum + 1;

      gimbalAction->waypointV2ActionTriggerType  = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated;
      gimbalAction->waypointV2AssociateTrigger.actionAssociatedType = dji_osdk_ros::WaypointV2AssociateTrigger::DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised;
      gimbalAction->waypointV2AssociateTrigger.waitingTime = 0;
      gimbalAction->waypointV2AssociateTrigger.actionIdAssociated = id-1;

      
      gimbalAction->waypointV2ACtionActuatorType = dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeGimbal;
      // We are gonna rotate the gimbal somehow so we need to set this operation type

      gimbalAction->waypointV2GimbalActuator.DJIWaypointV2ActionActuatorGimbalOperationType = dji_osdk_ros::WaypointV2GimbalActuator::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal;
      gimbalAction->waypointV2GimbalActuator.actuatorIndex = 0;
      // Gimbal Parameters
      // Gimbal roll angle
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.x = 0; 
      // Gimbal pitch angle
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y = 10*gimbal_pitch_list_global.data[i]; // TBD: Change it acording to user needs -> gimbal_pitch_list_global.data[i];
      // Gimbal yaw angle
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.z = 0;//10*yaw_list_global.data[i]; 

      // Gimbal Control mode
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.ctrl_mode = 0; // 0: absolute angle, 1: relative angle

      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.rollCmdIgnore = 1;
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.pitchCmdIgnore = 0;
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.yawCmdIgnore = 1;

      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.absYawModeRef = 1; //0: relative to the aircraft, 1: relative to North
   
      // Gimbal Control speed
      gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.duationTime = 20; // rotate time
      id+=1;  
      ROS_INFO("Gimbal action created with ID: %d associated to action: %d and with angle %d", gimbalAction->actionId, gimbalAction->waypointV2AssociateTrigger.actionIdAssociated, gimbalAction->waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y); // add more info when advances come

      generateWaypointV2Action_.request.actions.push_back(*gimbalAction);
      
      delete gimbalAction;
      action = new dji_osdk_ros::WaypointV2Action;
      

    }
    
    waypointV2_generate_actions_client.call(generateWaypointV2Action_);

    return generateWaypointV2Action_.response.result;
}
// Following changes: Do it with smart pointers


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

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
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
  waypointV2.maxFlightSpeed= 9;
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

bool initWaypointV2Setting(ros::NodeHandle &nh)
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
    generateWaypointV2AllActions_(nh, initWaypointV2Setting_.request.actionNum);
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

bool startWaypointV2Mission(ros::NodeHandle &nh)
{
    waypointV2_start_mission_client = nh.serviceClient<dji_osdk_ros::StartWaypointV2Mission>("dji_osdk_ros/waypointV2_startMission");
    waypointV2_start_mission_client.call(startWaypointV2Mission_);

    if(startWaypointV2Mission_.response.result)
    {
      ROS_INFO("Start waypoint v2 mission successfully!\n");
    }
    else
    {
      ROS_ERROR("Start waypoint v2 mission failed!\n");
    }

    return startWaypointV2Mission_.response.result;
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

bool runWaypointV2Mission(ros::NodeHandle &nh)
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
    
  result = initWaypointV2Setting(nh);
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

  /*! start mission 
  result = startWaypointV2Mission(nh);
  if(!result)
  {
    return false;
  }*/
  /*sleep(20);

  /*! set global cruise speed /
  result = setGlobalCruiseSpeed(nh, 1.5);
  if(!result)
  {
    return false;
  }
  sleep(timeout);

  /*! get global cruise speed /
  float32_t globalCruiseSpeed = 0;
  globalCruiseSpeed = getGlobalCruiseSpeed(nh);
  sleep(timeout);

  /*! pause the mission/
  result = pauseWaypointV2Mission(nh);
  if(!result)
  {
    return false;
  }
  sleep(5);

  /*! resume the mission/
  result = resumeWaypointV2Mission(nh);
  if(!result)
  {
    return false;
  }
  sleep(20); */

return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointV2_node");
  ros::NodeHandle nh;
  //nodehandler = nh;
  ros::Subscriber gpsPositionSub = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
    "obtain_release_control_authority");
  
  //if you want to fly without rc ,you need to obtain ctrl authority.Or it will enter rc lost.
  dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  // Our changes goes from here:
  ros::ServiceServer service_config_mission = nh.advertiseService("dji_control/configure_mission", config_mission);
  //ros::ServiceServer service_run_mission = nh.advertiseService("dji_control/start_mission", run_mission);
  //ros::ServiceServer service_send_bags = nh.advertiseService("dji_control/send_bags", sendFiles);

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //runWaypointV2Mission(nh);

  ros::waitForShutdown();
}
