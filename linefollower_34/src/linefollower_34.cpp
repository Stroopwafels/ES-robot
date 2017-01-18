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
		
		cv::Mat image_canny, image_lines;

		// Select ROI
		cv::Mat imageROI = cv_ptr->image;

		// Image filters

		// Edge detection
		cv::Canny(cv_ptr->image, image_canny, 50, 200, 3);

		// Probabilistic Hough Line transform
		std::vector<Vec4i> lines;
		cv::HoughLinesP(image_canny, lines, 1, CV_PI/180, 50, 50, 10 );

		// Line analysis
		for(size_t i = 0; i<lines.size(); i++) {
  			Vec4i l = lines[i];
  			line( image_lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
		}
	}

private:
	int img_height; // Nexus 5: img_height = 768px
	int img_width; // Nexus 5: img_width = 1280px
	
	// ROI boundaries 
	// float roi_up;
	// float roi_down;
	// float roi_left;
	// float roi_right;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "linefollower_34");
	ImageConverter ic;

	ros::spin();
	return 0;
}