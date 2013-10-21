#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <red_ball_tracker/TrackerMsg.h>

#include <cmath>
#include <iostream>
#include <iomanip>

#define HUE_CHANNEL 0
#define SATURATION_CHANNEL 2
#define LIGHTNESS_CHANNEL 1
#define PX2RAD 0.002
#define RBALLE 0.035

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv ;

static const char WINDOW_H[] = "Channel Hue";
static const char WINDOW_L[] = "Channel Lightness";
static const char WINDOW_S[] = "Channel Saturation";
static const char WINDOW_R[] = "Image filtered";
static const char WINDOW_C[] = "Configuration";
image_transport::Publisher pub;

int H_min = 164 ;
int H_max = 255 ;
int S_min = 233 ;
int S_max = 255 ;
int L_min = 0 ;
int L_max = 255 ;

/* Callback for image treatment */
void imageCallback ( const sensor_msgs::ImageConstPtr& original_image )
{
	ros::NodeHandle n;
	ros::Publisher trackerTopic = n.advertise<red_ball_tracker::TrackerMsg>("red_ball_tracker/tracking", 1);	
  red_ball_tracker::TrackerMsg teleopMsg ;	

	/* Initialisation of the two images : source and destination (image processed) */
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_chg;

	Mat hlsChannels[3];
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
		ROS_ERROR("Exception trying to copy image : %s", e.what());
		return;
	}


	/* Conversion BGR to HLS  */
	cvtColor ( cv_ptr->image, cv_chg->image, CV_BGR2HLS ) ;

	/* Application of thresholds */
	inRange(cv_chg->image, Scalar(H_min, L_min, S_min), Scalar(H_max, L_max, S_max), imageFiltered);

	/* Decomposing HSL image into channels, for visualization */
	split ( cv_chg->image, hlsChannels ) ;

	/* Noise cancelation */
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

    teleopMsg.distance = RBALLE / ( tan ( radius * PX2RAD ) ) ;
    teleopMsg.alphax = PX2RAD * ( center.x - 320 ) ;
    teleopMsg.alphay = PX2RAD * ( - center.y + 240 ) ;

    trackerTopic.publish(teleopMsg) ;		
	}

	/* Result display */
	imshow(WINDOW_H, hlsChannels[HUE_CHANNEL]);
	moveWindow(WINDOW_H, 0, 0) ;
	imshow(WINDOW_L, hlsChannels[LIGHTNESS_CHANNEL]);
	moveWindow(WINDOW_L, 320, 0) ;
	imshow(WINDOW_S, hlsChannels[SATURATION_CHANNEL]);
	moveWindow(WINDOW_S, 640, 0) ;
	imshow(WINDOW_R, imageFiltered) ;
	moveWindow(WINDOW_R, 320, 300) ;

	waitKey(1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "red_ball_tracker");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	namedWindow(WINDOW_H, CV_WINDOW_AUTOSIZE);
	namedWindow(WINDOW_S, CV_WINDOW_AUTOSIZE);
	namedWindow(WINDOW_L, CV_WINDOW_AUTOSIZE);
	namedWindow(WINDOW_R, CV_WINDOW_AUTOSIZE);

	namedWindow(WINDOW_C, 0);
	createTrackbar("H min", WINDOW_C, &H_min, 255 ) ;
	createTrackbar("H max", WINDOW_C, &H_max, 255 ) ;


	image_transport::Subscriber subImage = it.subscribe("/ardrone/image_raw", 1, imageCallback);
	destroyAllWindows();
	ros::spin();

	ROS_INFO("Program ended correctly");
}

