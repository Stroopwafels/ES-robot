# ES-robot
All packages for the line follower robot

Zorg ervoor dat je ook OpenCV voor ros hebt geïnstalleerd:
	sudo apt-get install ros-indigo-cv-bridge

## Startup sequence
	
	1. in terminal: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in andere terminal tab: source devel/setup.bash
	6. in terminal: roslaunch linefollower_34 default.launch
	
OF
	
	1. in terminal: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in andere terminal tab: source devel/setup.bash
	6. In terminal: rosrun linefollower_34 linefollower_34_node image:=camera/image _image_transport:=compressed

	met eventueel:
	1. In terminal: rosrun image_view image_view image:=linefollower_34/output_video _image_transport:=compressed
	2. In terminal: rosrun image_view image_view image:=camera/image _image_transport:=compressed
