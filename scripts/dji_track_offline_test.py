#!/usr/bin/env python
# -*- coding: utf-8 -*- 
# Original code: https://github.com/grvcTeam/grvc-ual/blob/master/uav_abstraction_layer/scripts/track_waypoints.py

import argparse, sys, yaml, rospy, rospkg

from inspector.srv import ConfigMission
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool

mission_type = "waypoint"
radius = 0
maxVel = 2                          # Maximum speed joystick input(2~15m)
idleVel = 1                         # Cruising Speed (without joystick input, no more than vel_cmd_range)

# # constant for yaw_mode
# uint8 YAW_MODE_AUTO     = 0       # auto mode (point to next waypoint)
# uint8 YAW_MODE_LOCK     = 1       # lock as an initial value
# uint8 YAW_MODE_RC       = 2       # controlled by RC
# uint8 YAW_MODE_WAYPOINT = 3       # use waypoint's yaw(tgt_yaw)
yawMode = 2

# # constant for trace_mode
# uint8 TRACE_POINT       = 0       # point to point, after reaching the target waypoint hover, complete waypt action (if any), then fly to the next waypt
# uint8 TRACE_COORDINATED = 1       # 1: Coordinated turn mode, smooth transition between waypts, no waypts task
traceMode = 0

# # constant for action_on_finish
# uint8 FINISH_NO_ACTION       = 0  # no action
# uint8 FINISH_RETURN_TO_HOME  = 1  # return to home
# uint8 FINISH_AUTO_LANDING    = 2  # auto landing
# uint8 FINISH_RETURN_TO_POINT = 3  # return to point 0
# uint8 FINISH_NO_EXIT         = 4  # infinite mode， no exit
finishAction = 0


# [ namespace, srv to get authority, srv to send mission, srv to start mission ]
config_m210 = ["","/dji_control/get_control","/dji_control/configure_mission"]
config_m600 = ["/m600","/dji_control/recover_control","/dji_control/configure_mission","/dji_control/start_mission"]

def track_waypoints():
    global finishAction
    # Parse arguments
    parser = argparse.ArgumentParser(description='Track waypoints defined in a yaml file')
    parser.add_argument('-pp','--plan_package', type=str, default='inspector',
                        help='Name of the package where plan to track is stored')
    parser.add_argument('-pf','--plan_file', type=str, default='wp_default.yaml',
                        help='Name of the file inside plan_package/plans')
    parser.add_argument('-w','--wait_for', action='store_true', help='Wait for human response', default='true')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('waypoint_tracker')

    file_name = args.plan_file
    # Autocomplete file extension
    if not file_name.endswith('.yaml'):
        file_name = file_name + '.yaml'

    file_url = rospkg.RosPack().get_path(args.plan_package) + '/plans/' + file_name
    with open(file_url, 'r') as wp_file:
        wp_data = yaml.load(wp_file, Loader=yaml.SafeLoader)

    for uav_id in range(1,wp_data["uav_n"]+1):
        config = ["uav_"+str(uav_id),"/dji_control/get_control",""] 
        
        if 'frame_id' not in wp_data:
            rospy.logerr("Must specify frame_id in waypoints file")  # TODO: default to ''?
            return

        if(uav_id == 1):
            config = config_m210
        elif(uav_id == 2):
            config = config_m600
            
        print "\n-- UAV "+config[0]+" --"

        missionMsg = ConfigMission()
        wp_list = []
        for wp_id in range(wp_data["uav_"+str(uav_id)]["wp_n"]):       
            print 'WP '+ str(wp_id)
            wp_raw = wp_data["uav_"+str(uav_id)]["wp_"+str(wp_id)]

            wp = NavSatFix()
            wp.latitude = wp_raw[0]
            wp.longitude = wp_raw[1]
            wp.altitude = wp_raw[2]
            wp_list.append(wp)
            
            print "latitude: "+ str(wp.latitude)
            print "longitude: "+str(wp.longitude)
            print "altitude: "+str(wp.altitude)

            print config[0]+config[2]
            rospy.wait_for_service(config[0]+config[2])
            try:
                load_mission = rospy.ServiceProxy(config[0]+config[2], ConfigMission)
                # TODO: Check we're flying!
                print "Ready to track " + str(wp_data["uav_"+str(uav_id)]["wp_n"]) + " waypoints to UAV " + str(uav_id)
                if args.wait_for:
                    answer = raw_input("Continue? (y/N): ").lower().strip()
                    if answer == 'y' or answer == 'yes':
                        if(uav_id==2):
                            finishAction = 1
                        res = load_mission(mission_type,wp_list,radius,maxVel,idleVel,yawMode,traceMode,finishAction)
                        print "Mission ACK:"+str(res.success)
                        if(res.success and uav_id == 2):
                            rospy.wait_for_service(config[0]+config[3])
                            try:    
                                start_mission = rospy.ServiceProxy(config[0]+config[3], SetBool)
                                res = start_mission(True)
                                if(res.success):
                                    print("Mission running: Take Off!")
                            except rospy.ServiceException as e:
                                print("Mission start service call failed: %s"%e)
                        else:
                            print("Mission running: Take Off!")
                    else:
                        print "Mission aborted"
                else:
                    res = load_mission(mission_type,wp_list,radius,maxVel,idleVel,yawMode,traceMode,finishAction)
                    print "Mission ACK:"+str(res.success)

            except rospy.ServiceException as e:
                print("Mission service call failed: %s"%e)

if __name__ == "__main__":
    track_waypoints()
