# ES-robot
All packages for the line follower robot

Zorg ervoor dat je ook OpenCV voor ros hebt geïnstalleerd:
	sudo apt-get install ros-indigo-cv-bridge

## Standaard Startup
	
	1. in terminal 1: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal 1: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in terminal 2: source devel/setup.bash
	6. in terminal 2: roslaunch linefollower_34 default.launch port:=/dev/ttyUSB0
	
OF
	
	1. in terminal 1: source <locatie van git repo>/resources/ipset.bash 
	2. in terminal 1: roscore
	3. Voer op de android-app bij Robot URI in: http://<je eigen IP>:11311/
	4. Connect op Android telefoon
	5. in terminal 2: source devel/setup.bash
	6. In terminal 2: rosrun linefollower_34 linefollower_34_node image:=camera/image _image_transport:=compressed
	7. In terminal 3: rosrun rosserial_python serial_node.py /dev/ttyUSB0

	met eventueel:
	8. In terminal 3: rosrun image_view image_view image:=linefollower_34/output_video _image_transport:=compressed
	9. In terminal 4: rosrun image_view image_view image:=camera/image _image_transport:=compressed

## Arduino test startup
	1. Terminal 1: roscore
	2. Terminal 2: rosrun turtlesim turtlesim_node
	3. Terminal 3: rosrun turtlesim turtle_teleop_key
	4. Terminal 4: rosrun rosserial_python serial_node.py /dev/ttyUSB0

	bij de laatste /dev/ttyUSB0 hangt af van je arduino port natuurlijk

OF
	1. Terminal 1: roscore
	2. Terminal 2: source devel/setup.bash
	3. Terminal 2: roslaunch linefollower_34 arduino_test port:=/dev/ttyUSB0