# Summary

This package is the ros package for teleoperating multiple chairbots via one central machine.
This package acts as the ros master and can connect to joysticks to control multiple chairbots.
It uses fiducials on the chairs along with anchor fiducials to localize robots

This is the server script, the robot script lives [in this repo](https://github.com/stoddabr/chairbot_robot)

# Quickstart

## Setup chairbots

### SSH into Raspi
Make sure that the raspi is set up to ssh properly and is active on the same interent connection

To find the ip address of the raspi, use a tool like nmap `sudo nmap -sS -p 22 192.168.10.0/24`
Or manually set it to a static value.

By default ours is set static to `192.168.1.73`

SSH into the raspi using the command `sudo ssh chairbot01@192.168.1.73`

On default raspi's the password will be `raspberry`

You now have terminal access to the chairbot Raspi

### Install ROS on Raspi

coming soon...

### Set up custom code
Copy the code from the `robots` folder into the raspi using scp. Eg `/home/chairbot01/catkin_ws/src/chairbot_neato/chairbot_neato_node`

Or clone the raspi code in this repo and only use the `robots` folder

Set it up to run with ros

`roslaunch chairbot_neato_node ui.launch`

## Setup server

`roslaunch central_server start_ui_server.launch`

## ...
coming soon...


# Features
  - Support for various screen sizes (including mobile via localhost)
  - Controllable via a joystick
  - Can control one or multiple chairbots
  - Fiducial tracking via webcam

# Planned changes
  - Collision avoidance
  - Arrangement memory and recall

# External Links
Chirbot Wiki for more information on getting started with the chairs https://github.com/abhiagni11/ChairBots_ROS_system/wiki
Charisma Lab Github for other chairbot projects

Snow white project that uses fiducials similarly https://github.com/charisma-lab/neato_localization/
  - image is processed in `tracking_aruco_markers.py`
  - then `localizing_tracked_markers.py` updates marker locations and publishes poses where applicable

# contributors
  - after 4/2020 Brett Stoddard  stoddardbrett@gmail.com
  - 1/2019-6/2019 Samarendra Hedaoo hedaoos@oregonstate.edu
  - 6/2018-4/2020 Abrar Fallatah fallataaa@oregonstate.edu
  - 6/2018-9/2018 Brett Alteria altenab@oregonstate.edu
  - 9/2017-6/2019 Abhijeet Agnihotri agnihota@oregonstate.edu
