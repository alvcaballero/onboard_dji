# onboard_dji
GRVC repository including the software developed for the operation of DJI UAVs.
## Usage
The purpose is to install this package in a companion computer connected to the DJI Matrice series drone.



Of course you will have to communicate with the companion computer which is onboard your UAV, in our case we have an ubiquiti based local network and we connect to the onboard computer with ssh protocol:
```
# DJI uav_1 autonomous landing based on vision
ssh grvc@10.42.0.99 #pswd: grvc1234
# DJI uav_2 
ssh ubuntu@10.42.0.42 #pswd: ubuntu

```

We defined an alias inside the *.bashrc* in order to configure the drone for a simulated flight or a real flight:
- *M210-real*: \<path-to-OSDK\>/Onboard-SDK/utility/bin/armv8/64-bit/M210ConfigTool --usb-port /dev/ttyACM0 --config-file \<path-to-UserConfig\>/UserConfig.txt --usb-connected-flight on
- *M210-sim-on*:<path-to-OSDK>/Onboard-SDK/utility/bin/armv8/64-bit/M210ConfigTool --usb-port /dev/ttyACM0 --config-file \<path-to-UserConfig\>/UserConfig.txt--simulation on --latitude 37.53395183 --longitude -6.30202933 --usb-connected-flight on

**NOTE: the aliases were done thanks to DJI Developer's documentation provided by official DJI websites such as [M210 Aircraft Checklist ](https://developer.dji.com/onboard-sdk/documentation/M210-Docs/aircraft-checklist.html),[Correction with newer versions](https://sdk-forum.dji.net/hc/en-us/articles/360062675633--Summary-of-the-problem-of-connecting-the-aircraft-to-DJI-assistant-2)**
### real
If we want to flight in a real environment we just need two terminals and execute the following commands:
```
#Launch the alias M210-real
M210-real

# Launch the mission node
roslaunch onboard_dji atlas.launch
```

To have the whole ROS access from GCS computer we need to execute the multimaster node too:

```
roslaunch onboard_dji multimaster.launch
```


### simulation
Here we have a Hardware In The Loop (HITL) kind of simulation, and we proceed as follows:

```
#Launch the alias M210-sim-on
M210-sim-on
```

Then we perform as in real flights:

```
# Launch the mission node
roslaunch onboard_dji atlas.launch
```

And finally in another terminal:

```
roslaunch onboard_dji multimaster.launch
```
