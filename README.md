# ES-robot
All packages for the line follower robot

Zorg ervoor dat je ook OpenCV voor ros hebt geïnstalleerd:
	sudo apt-get install ros-indigo-cv-bridge

Zorg er voor het opstarten van roscore ervoor dat de IP-instellingen van de android telefoon en ROS goed zijn ingesteld (vervang <je eigen ip> inclusief haakjes):

	1. In terminal: ifconfig
		om je ip-adres te achterhalen.

	2. In terminal: export ROS_IP=<je eigen IP>
					export ROS_MASTER_URI=http://$ROS_IP:11311
		om in ROS de juiste IP-instellingen aan te zetten

	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
