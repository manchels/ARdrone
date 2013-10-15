#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <iomanip>

#define HUE_CHANNEL 0
#define SATURATION_CHANNEL 2
#define LIGHTNESS_CHANNEL 1


namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv ;

static const char WINDOW_H[] = "Channel Hue";
static const char WINDOW_L[] = "Channel Lightness";
static const char WINDOW_S[] = "Channel Saturation";
static const char WINDOW_R[] = "Image filtered";
image_transport::Publisher pub;

int H_min = 164 ;
int H_max = 255 ;
int S_min = 233 ;
int S_max = 255 ;

int test = 0 ;

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	
	/* Initialisation of the two images : source and destination (image processed) */
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_chg;

	Mat hlsChannels[3];
	Mat hlsChannelsThreshold[3];

	Mat imageFiltered ;
	vector<vector<Point> > contours;
	vector<Vec4i> contoursHierarchy;
	vector<vector<Point> >::iterator iteratorContours ;

	Mat erodeElement  = getStructuringElement( MORPH_RECT, Size ( 3, 3) );
	Mat dilateElement = getStructuringElement( MORPH_RECT, Size ( 3, 3) );
	Point2f center ;
	float radius ;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
		cv_chg = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}


	/* OpenCV algorithm for ball detection  */
	cv::cvtColor ( cv_ptr->image, cv_chg->image, CV_BGR2HLS ) ;

	/* Application of thresholds and noise cancelation */
	inRange(cv_chg->image, cv::Scalar(H_min, 0, S_min), cv::Scalar(H_max, 255, S_max), imageFiltered);
	cv::imshow(WINDOW_R, imageFiltered) ;
	erode ( imageFiltered, imageFiltered, erodeElement );
	erode ( imageFiltered, imageFiltered, erodeElement );
	dilate ( imageFiltered, imageFiltered, dilateElement );
	dilate ( imageFiltered, imageFiltered, dilateElement );


	/* Contour determination */
	Canny ( imageFiltered, imageFiltered, 100, 200 ) ;
  	findContours( imageFiltered, contours, contoursHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	cout << "New image" << endl ;
	for ( iteratorContours = contours.begin() ; iteratorContours != contours.end() ; iteratorContours++)
	{	
		minEnclosingCircle ( *iteratorContours, center, radius ) ;
		cout << center << endl << radius << endl << endl ;
	}


	/* Decomposing HSL image into channels, for visualization */
	cv::split ( cv_chg->image, hlsChannels ) ;
	cv::inRange ( hlsChannels[HUE_CHANNEL], cv::Scalar(H_min), cv::Scalar(H_max), hlsChannelsThreshold[HUE_CHANNEL]) ; 
	cv::inRange ( hlsChannels[SATURATION_CHANNEL], cv::Scalar(S_min), cv::Scalar(S_max), hlsChannelsThreshold[SATURATION_CHANNEL]) ; 


	/* Result display */
	cv::imshow(WINDOW_H, hlsChannelsThreshold[HUE_CHANNEL]);
	cv::imshow(WINDOW_L, hlsChannels[LIGHTNESS_CHANNEL]);
	cv::imshow(WINDOW_S, hlsChannelsThreshold[SATURATION_CHANNEL]);
	

	cv::waitKey(1);
	/*_______________________OPENCV ENDS HERE__________________________*/

	pub.publish(cv_chg->toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_images");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	cv::namedWindow(WINDOW_H, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_S, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_L, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW_R, CV_WINDOW_AUTOSIZE);
	image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
	cv::destroyWindow(WINDOW_H);
	cv::destroyWindow(WINDOW_S);
	cv::destroyWindow(WINDOW_L);
	cv::destroyWindow(WINDOW_R);
	pub = it.advertise("camera/image_processed", 1);
	ros::spin();

	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}

