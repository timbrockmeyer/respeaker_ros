echo "Starting snips" &&
sudo systemctl start snips-audio-server.service &&
sudo systemctl start snips-asr.service &&
cd ~/catkin_ws &&
source ~/catkin_ws/devel/setup.bash &&
roslaunch respeaker_ros respeaker_snips.launch
