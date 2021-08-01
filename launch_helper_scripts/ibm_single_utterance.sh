echo "Killing snips, because it is hogging the microphone channels if it's running" &&
sudo systemctl stop snips-asr.service &&
sudo systemctl stop snips-dialogue.service &&
sudo systemctl stop snips-injection.service &&
sudo systemctl stop snips-skill-server.service &&
sudo systemctl stop snips-audio-server.service &&
sudo systemctl stop snips-hotword.service &&
sudo systemctl stop snips-nlu.service &&
sudo systemctl stop snips-tts.service &&
echo "Snips successfully shut down"
cd ~/catkin_ws &&
source ~/catkin_ws/devel/setup.bash &&
roslaunch respeaker_ros respeaker_ibm_single_utterance.launch
