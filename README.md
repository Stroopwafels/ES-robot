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
	
	1. In terminal: roscore
	2. Android telefoon aanzetten
	3. In terminal: rosrun linefollower_34 linefollower_34_node image:=camera/image _image_transport:=compressed

	eventueel:
	1. In terminal: image_view image_view image:=camera/image _image_transport:=compressed