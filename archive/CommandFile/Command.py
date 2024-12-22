#Shell
#Preset
#four terminal
#
cd workspace2/team3_ws/
source install/setup.bash
ros2 run send_script img_sub

#set the arm & code connector
source install/setup.bash
cd ~/colcon_ws
ros2 run tm_driver tm_driver 192.168.0.69

##our controlling code    send signal once and will run once
ros2 run send_script send_script

#Viewer(not sure what for)
ros2 run tm_get_status image_talker

