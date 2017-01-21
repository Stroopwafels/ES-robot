#!/bin/bash

# sets ROS ip environment variables to your own IP-adress
ipadress=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_IP=$ipadress
export ROS_MASTER_URI=http://$ROS_IP:11311
echo "Your IP adress = $ipadress"