// C++ includes
#include <math.h>
#include <assert.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Image handling and openCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter {
	

public:
	ImageConverter():
	img_height(768),
	img_width(1280),
	roi_up(0.7), roi_down(1), roi_left(0), roi_right(1),
	turn_area_width(0.4),
	threshold_val(100),
	it_(nh_) {
		// Subscribe to the camera/image topic
		image_sub_ = it_.subscribe("camera/image", 1, &ImageConverter::imageCallback, this);

		// Publish generated images on linefollower_34/output_video
		image_pub_ = it_.advertise("linefollower_34/output_video", 1);

		// Publish Twist messages on cmd_vel
		twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1000);

		// Asserts
		assert(roi_up >= 0 && roi_up <=1);
		assert(roi_down >= 0 && roi_down <=1);
		assert(roi_left >= 0 && roi_left <=1);
		assert(roi_right >= 0 && roi_right <=1);
		assert(turn_area_width >= 0 && turn_area_width <=0.5);
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
		// Convert ros-img to OpenCV
		cv_bridge::CvImagePtr cv_ptr;

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
		
		cv::Mat image_rot, image_roi, image_grey, image_blur, image_threshold, image_canny, image_lines;

		cv::transpose(cv_ptr->image, image_rot);  // rotate 90 deg clockwise
    	cv::flip(image_rot, image_rot, 1);

    	// image_lines = image_rot;

    	// Get image dimensions in pixels
		img_height = image_rot.rows; 
		img_width = image_rot.cols;

		ROS_DEBUG("img_height = %d, img_width = %d\n", img_height, img_width);

		// Select ROI
		cv::Rect roi(roi_left*img_width, roi_up*img_height, (roi_right-roi_left) *img_width, (roi_down-roi_up)*img_height);
		image_roi = image_rot(roi);
		image_lines = image_roi;

		// Image filters
		cv::cvtColor(image_roi, image_grey, CV_RGB2GRAY);
		//cv::GaussianBlur(image_grey, image_blur, cv::Size(3, 3), 0, 0);
		cv::blur(image_grey, image_blur, cv::Size(3,3), cv::Point(-1,-1));
		cv::threshold(image_blur, image_threshold, threshold_val, 255, CV_THRESH_BINARY);
		
		// Edge detection
		cv::Canny(image_threshold, image_canny, 50, 200, 3);

		// Probabilistic Hough Line transform
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(image_canny, lines, 1, CV_PI/180, 50, 50, 10 );

		// Line analysis
		int min_r = img_width / 2;
		int min_cx = img_width / 2;
		int min_cy = 0;

		if (lines.size() == 0) {
			ROS_WARN("I see no lines");
		}

		for(size_t i = 0; i<lines.size(); i++) {
  			cv::Vec4i l = lines[i];
  			cv::line( image_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);

  			// Pick the line with the smallest horizontal distance to the center of the image
  			int cx = (l[0] + l[2]) / 2;
  			int cy = (l[1] + l[3]) / 2;
  			int r = abs( cx - img_width / 2);
  			if(r < min_r) {
  				min_cx = cx;
  				min_cy = cy;
  				min_r = r;
  			}
		}

		// Generate Twist message
		geometry_msgs::Twist twist_msg;

		if (min_cx < img_width*turn_area_width) {
			ROS_INFO("< Turning left");
			
		} else if ( min_cx > img_width*(1 - turn_area_width) ) {
			ROS_INFO("> Turning right");
			
		} else {
			ROS_INFO("^ Going Straight");

		}
		
		// Create output image
		cv::line(image_lines, cv::Point(0, min_cy), cv::Point(img_width, min_cy), cv::Scalar(255,0,0), 3, CV_AA);
		cv::line(image_lines, cv::Point(min_cx, 0), cv::Point(min_cx, img_height), cv::Scalar(255,0,0), 3, CV_AA);

		cv::line(image_lines, cv::Point(img_width*turn_area_width, 0), cv::Point(img_width*turn_area_width, img_height), cv::Scalar(255,255,0), 3, CV_AA);
		cv::line(image_lines, cv::Point(img_width*(1 - turn_area_width), 0), cv::Point(img_width*(1 - turn_area_width), img_height), cv::Scalar(255,255,0), 3, CV_AA);

		cv_ptr->image = image_roi; //image_lines;
		image_pub_.publish(cv_ptr->toImageMsg());
		ROS_DEBUG("Publishing new image\n");
		//TODO: Publish image_lines
	}

private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::Publisher twist_pub_;


	int img_height; // Nexus 5: img_height = 768px
	int img_width; // Nexus 5: img_width = 1280px

	// ROI boundaries 
	float roi_up; // 0 = upper boundary of image; 1 = lower boundary
	float roi_down;
	float roi_left; // 0 = left boundary of image; 1 = right boundary
	float roi_right;

	// Turning area width
	float turn_area_width; // 0 = No turning areas; 0.5 = turning areas go all the way to the horizontal center of the image

	// Filter variables
	float threshold_val;
};

int main(int argc, char **argv) {
	ROS_INFO("Starting up image converter\n");
	ros::init(argc, argv, "linefollower_34");
	ImageConverter ic;

	ros::spin();
	return 0;
}