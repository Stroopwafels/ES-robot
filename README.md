# ES-robot
All packages for the line follower robot

Zorg ervoor dat je ook OpenCV voor ros hebt geïnstalleerd:
	sudo apt-get install ros-indigo-cv-bridge

## Startup sequence
	
	1. in terminal 1: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal 1: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in terminal 2: source devel/setup.bash
	6. in terminal 2: roslaunch linefollower_34 default.launch
	
OF
	
	1. in terminal 1: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal 1: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in terminal 2: source devel/setup.bash
	6. In terminal 2: rosrun linefollower_34 linefollower_34_node image:=camera/image _image_transport:=compressed

	met eventueel:
	7. In terminal 3: rosrun image_view image_view image:=linefollower_34/output_video _image_transport:=compressed
	8. In terminal 4: rosrun image_view image_view image:=camera/image _image_transport:=compressed
