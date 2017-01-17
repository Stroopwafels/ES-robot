// C++ includes
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Image handling and openCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/improc/improc.hpp>
//#include <opencv2/highgui/highgui.hpp>

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	ros::Publisher twist_pub_;

public:
	ImageConverter():
	it_(nh_) {
		// Subscribe to the camera/image topic
		image_sub_ = it_.subscribe("camera/image",1, &ImageConverter::imageCallback, this);

		// Publish Twist messages on cmd_vel
		twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
		

	}
}

int main(int argc, char const *argv[]) {
	ros::init(argc, argv, "linefollower_34");


	ros::spin();
	return 0;
}