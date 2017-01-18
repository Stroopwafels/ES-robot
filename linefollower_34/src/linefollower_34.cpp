// C++ includes
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Image handling and openCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CONTOURS

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::Publisher twist_pub_;

public:
	ImageConverter():
	img_height(768),
	img_width(1280),
	it_(nh_) {
		// Subscribe to the camera/image topic
		image_sub_ = it_.subscribe("camera/image", 1, &ImageConverter::imageCallback, this);

		// Publish generated images on linefollower_34/output_video
		image_pub_ = it_.advertise("linefollower_34/output_video", 1);

		// Publish Twist messages on cmd_vel
		twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1000);

		assert(calc_style = "contours" | calc_style = "hough_lines")
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
		// Convert ros-img to OpenCV
		cv_bridge::CvImagePtr cv_ptr;

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		// Get image dimensions in pixels
		img_height = cv_ptr->image.rows; 
		img_width = cv_ptr->image.cols;

		ROS_DEBUG("img_height = %d, img_width = %d\n", img_height, img_width);
		

	}

private:
	int img_height; // Nexus 5: img_height = 768px
	int img_width; // Nexus 5: img_width = 1280px
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "linefollower_34");
	ImageConverter ic;

	ros::spin();
	return 0;
}