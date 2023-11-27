#!/usr/bin/env python3

'''
    This is a test file to test the themal camera's tool exiftool
'''

import sys
import cv2
import numpy as np
import sys, rospy, rospkg, os

# to work with dates
from datetime import datetime
from onboard_dji.srv import ProcessImg, ProcessImgResponse
from sensor_msgs.msg import NavSatFix

# Decoding the thermal image
from thermal_base import ThermalImage

## general porpuse functions definition
def show_temp(x,y,temp_image):
    
    # print the value over the image showed with openCV
    #cv2.putText(temp_image, str(thermal_np[x,y]), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.circle(temp_image, (x, y), 2, (0, 0, 0), -1)
    cv2.putText(temp_image," {:3.2f} C".format(thermal_np[y,x]),(x - 80, y - 15), cv2.FONT_HERSHEY_PLAIN, 1,(255,0,0),2)
    print("The temperature at point ({},{}) is {:.2f} º C".format(x,y,thermal_np[y,x])) # Convert to ROS_INFO or ROS_WARN

    

def find_max_temp(thermal_np):
    # Encontrar el valor máximo en el array
    max_temp = np.max(thermal_np)

    # Encontrar los índices del valor máximo
    max_idx = np.argwhere(thermal_np == max_temp)

    print("Máximum temperature:", max_temp) # Convert to ROS_INFO or ROS_WARN
    print("Indexes:", max_idx[0]) # Convert to ROS_INFO or ROS_WARN
    print(type(max_idx[0])) # Convert to ROS_INFO or ROS_WARN
    #show_temp(max_idx[0][1],max_idx[0][0],temp_image)
    cv2.circle(temp_image, (max_idx[0][1], max_idx[0][0]), 2, (0, 0, 0), -1)
    cv2.putText(temp_image,"Max: {:.2f} C".format(thermal_np[max_idx[0][0],max_idx[0][1]]),(max_idx[0][1] - 80, max_idx[0][0] - 15), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)

########################################################################################################################################
'''
Handle function for the service
'''
def handle_process_thm_img(req):
    print("Processing the imgs between ", req.initDate, " and ", req.FinishDate)
    home_path = os.path.expanduser("~")
    rospy.loginfo("The home path is {}".format(home_path))
    root_path = home_path +"/uav_media"
    # At first we list the directories in the root_path
    #print(os.listdir(root_path))

    for root, dirs, files in os.walk(root_path):
        for dir in dirs:
            # directories must have the template format: mission_YYYY-MM-DD_HH:MM and we need to obtain the date and time to compare with the requested
            # mission
            print(dir)
            dir_date = dir.split("_")[1]
            try:
                dir_date_std = datetime.strptime(dir_date, '%Y-%m-%d_%H-%M')
                #Debugging
                
            except ValueError:
                rospy.logerr("The folder {} is not in the correct format".format(dir))
                continue
            rospy.loginfo("The date of the folder {} is {}".format(dir, dir_date_std))
            

            # Converting the string to datetime
            init_date = datetime.strptime(req.initDate, '%Y-%m-%d %H:%M')
            finish_date = datetime.strptime(req.FinishDate, '%Y-%m-%d %H:%M')
            # do the comparison
            if dir_date >= init_date and dir_date <= finish_date:
                rospy.loginfo("The mission folder {} is between the requested dates".format(dir))
                # now we need to check if the mission has thermal images
                for name in files:
                    if name.endswith(("THRM.jpg")):
                        # debug
                        print(os.path.join(root,dir,name))
            else:
                rospy.logwarn("The mission folder {} is NOT between the requested dates".format(dir))
                return ProcessImgResponse(False)
            
    return ProcessImgResponse(True)
'''       
    path_to_image = "/home/ubuntugrvc/thermal_cameras/examples/H20T/DJI_202307211037_002/DJI_20230721104850_0004_T.JPG"
    image = ThermalImage(image_path=path_to_image, camera_manufacturer="dji")
    thermal_np = image.thermal_np           # The temperature matrix as a np array
    raw_sensor_np = image.raw_sensor_np     # The raw thermal sensor excitation values as a np array
    meta = image.meta                       # Any other metadata that exiftool picked up
    colorbar = image.generate_colorbar(cmap = cv2.COLORMAP_HOT)   # Returns: np.ndarray: A colourbar of the required height with temperature values labelled

    print(thermal_np)
    print("the shape of the thermal image is:", thermal_np.shape)
    #show the colorbar image which is in the form of np array

    cv2.imshow("colorbar", colorbar)


    # Thermal Image annotation
    from thermal_base import ThermalImageAnnotation

    img_ann = ThermalImageAnnotation()

    # Manipulation of the image
    from thermal_base import utils

    #print(dir(utils))                                   # View manipulation tools available
    #thermal_np = utils.change_emissivity_for_roi(...)   # Sample: Change the emissivity of an RoI

    temp_image = utils.get_temp_image(thermal_np, colormap=cv2.COLORMAP_HOT)     
    print("Thermal image as numpy image:", type(temp_image))
    print(temp_image)



    ## As we have the thermal image and a matrix of the same size with the temperature values, we can now do some image processing
    # 1. We can apply a threshold to the image to get the areas that are above a certain temperature
    # 2. We can apply a gaussian filter to the image to smooth it out
    # 3. We can apply a median filter to the image to smooth it out
    # 4. We can apply a bilateral filter to the image to smooth it out
    # 5. We can apply a morphological filter to the image to smooth it out
    # 6. We can apply a canny filter to the image to smooth it out
    # 7. We can apply a laplacian filter to the image to smooth it out
    # 8. We can apply a sobel filter to the image to smooth it out
    # 9. We can apply a scharr filter to the image to smooth it out
    # 10. We can apply a prewitt filter to the image to smooth it out
    # 11. We can apply a roberts filter to the image to smooth it out
    # 12. We can apply a hessian filter to the image to smooth it out
    # 13. We can apply a frangi filter to the image to smooth it out
    # 14. We can apply a meijering filter to the image to smooth it out

    # or simply show the temperature in a single point over the image given the coordinates of the point


    show_temp(326,198,temp_image) # temperature of my head
    show_temp(344,244,temp_image) # temperature of the floor
    show_temp(141,78,temp_image) # temperature of the RTK

    find_max_temp(thermal_np)


    # Show the image
    cv2.imshow("temp_image", temp_image)
    cv2.waitKey(0)
'''




########################################################################################################################################
def process_thm_img_server():
    rospy.init_node('process_thm_img_node')
    # TBD: create the service
    s = rospy.Service('dji_extra/process_thermal_image', ProcessImg, handle_process_thm_img)
    print("Ready to process the thermal images that you need")
    rospy.spin()


if __name__ == "__main__":
    process_thm_img_server()