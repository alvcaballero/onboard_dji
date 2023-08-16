# Problems and solutions during the development

## Camera Stream issue

With a DJI Matrice 210 V2 RTK, we have some problems to obtain the video streaming with the advance sensing examples. Our onboard computer is a Raspberry pi 4 with ubuntu 18.04 (which was updated to 20.04) we used the last version of the OSDK (4.1) and we try the 3.9 too, but we cannot get the video streaming in neither of them .

The procedures that we followed were the following:
https://developer.dji.com/onboard-sdk/documentation/quickstart/run-the-sample.html

https://developer.dji.com/onboard-sdk/documentation/M210-Docs/aircraft-checklist.html

https://sdk-forum.dji.net/hc/en-us/articles/360062675633--Summary-of-the-problem-of-connecting-the-aircraft-to-DJI-assistant-2

https://sdk-forum.dji.net/hc/en-us/articles/360060633874-When-getting-the-data-of-dji-osdk-ros-fpv-camera-images-the-corresponding-data-output-is-not-obtained

We can obtain response if we do ping to the IP of the ethernet over USB network created by the connection of the USB showed in the links, but if we run the examples (camera-stream-poll-sample and camera-stream-callback-sample) we obtain this:

So we sent an email to the dji support dev@dji.com telling them our problem. The response they give us was absolutely accurate and solve the problem: 

*Based on the log, it seems that the USB link is currently unavailable. Please make sure you have a double-A USB connection. Then, you can check the RNDIS network port IP by using the command "ifconfig" on your onboard computer. After that, try pinging the IP address 192.168.42.2 (which is the drone IP). If the ping fails, you can manually set the IP of the RNDIS network port to 192.168.42.3. This will ensure that you can successfully ping the IP of the drone. Once you've done that, you can run the OSDK to get the video stream.*

We change manually the ip with [netplan](https://www.linux.com/topic/distributions/how-use-netplan-network-configuration-tool-linux/) and everything works with the fpv camera and a Zenmuse XT2.

## Obtaining Camera files
We try to execute the example given by the OSDK called [download_sample.cpp](https://github.com/dji-sdk/Onboard-SDK/blob/118e2825a347499efb8ed253146552c5b9b10779/sample/platform/linux/payloads/download_sample.cpp) but it does not work, so we contact the support team again. 

The response was "*We regret to inform you that the M210 does not support the OSDK for downloading media files from the camera SD card. However, we are pleased to let you know that the M300, M350, M30/T, and M3 Enterprise drones do support this feature.*"

## Actions
In order to perform a mission in a proper way, we needed to control the yaw and the gimbal pitch angle. Even though the mission itself allows you specify this parameters in each waypoint, we explore another option because the angles were interpolated between two waypoints. 


The solution was as simple as include a vector in the configuration file (.yaml) with the actions parameters needed and include them into the ros node. This parameters format are defined in the following links related to MissionWaypoint V1 type of mission:

- [MissionWaypoint.msg](https://github.com/dji-sdk/Onboard-SDK-ROS/blob/master/msg/MissionWaypoint.msg)
- [MissionWaypointTask.msg](https://github.com/dji-sdk/Onboard-SDK-ROS/blob/master/msg/MissionWaypointTask.msg) 
- [dji_mission_type.hpp](https://github.com/dji-sdk/Onboard-SDK/blob/118e2825a347499efb8ed253146552c5b9b10779/osdk-core/api/inc/dji_mission_type.hpp#L126)


