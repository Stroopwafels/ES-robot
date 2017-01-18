# ES-robot
All packages for the line follower robot

Zorg ervoor dat je ook OpenCV voor ros hebt geïnstalleerd:
	sudo apt-get install ros-indigo-cv-bridge

Zorg er voor het opstarten van roscore ervoor dat de IP-instellingen van de android telefoon en ROS goed zijn ingesteld:

	1. In terminal: ifconfig
		om je ip-adres te achterhalen.

	2. In terminal: export ROS_IP=<je eigen IP>
					export ROS_MASTER_URI=http://$ROS_IP:11311
		om in ROS de juiste IP-instellingen aan te zetten

	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/

Startup sequence:
	
	1. In terminal: source devel/setup.bash
	2. In terminal: roscore
	3. Connect op Android telefoon
	4. In terminal: rosrun linefollower_34 linefollower_34_node image:=camera/image _image_transport:=compressed

	eventueel:
	1. In terminal: rosrun image_view image_view image:=linefollower_34/output_video _image_transport:=compressed
	2. In terminal: rosrun image_view image_view image:=camera/image _image_transport:=compressed

	OF

	1. source devel/setup.bash
	2. In terminal: roslaunch linefollower_34 default.launch
	