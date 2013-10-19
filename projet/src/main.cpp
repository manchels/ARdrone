#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include <iostream>
#include <iomanip>

#define HUE_CHANNEL 0
#define SATURATION_CHANNEL 2
#define LIGHTNESS_CHANNEL 1

#define JOY_INIT_SWITCH 1
#define JOY_INIT_TAKEOFF 2
#define JOY_INIT_LAND 4
#define JOY_INIT_MOVE_FORWARD 8
#define JOY_INIT_MOVE_RIGHT 16
#define JOY_INIT_MOVE_UP 32
#define JOY_INIT_TURN_RIGHT 64
#define JOY_CMD_SWITCH 0
#define JOY_CMD_TAKEOFF 1
#define JOY_CMD_LAND 2

#define AUTO_CONTROL 0
#define MANUAL_CONTROL 1

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

int initJoy = 0 ;
int joyCmd[10] ;
int typeControl = AUTO_CONTROL ;

/* Callback for joystick commands treatment */
void joyCallback ( const sensor_msgs::Joy::ConstPtr& joyMessage)
{
	int i = 0 ;
	ros::NodeHandle n;
	ros::Publisher takeoffTopic = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	ros::Publisher landTopic = n.advertise<std_msgs::Empty>("ardrone/land", 1);
	ros::Publisher emergencyTopic = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist cmd ;

	if ( initJoy != ( JOY_INIT_SWITCH | JOY_INIT_TAKEOFF | JOY_INIT_LAND | JOY_INIT_MOVE_FORWARD | JOY_INIT_MOVE_RIGHT | JOY_INIT_MOVE_UP | JOY_INIT_TURN_RIGHT ) )
	{
		for ( i = 0 ; i < 30 && joyMessage->buttons[i] == 0 ; i++ ) { }			

		if ( ( initJoy & JOY_INIT_SWITCH ) != JOY_INIT_SWITCH && i != 30 && joyMessage->buttons[i] == 1 )
		{
			initJoy = initJoy | JOY_INIT_SWITCH ;
			joyCmd[JOY_CMD_EMERGENCY] = i ;
			cout << "Button assigned for switching between auto control and manual control is set to button " << i << endl ;

			cout << "Please press and hold button to take off (press any key when ready)"  ;  
			cin.get() ; 
		}
		else if ( ( initJoy & JOY_INIT_TAKEOFF ) != JOY_INIT_TAKEOFF && i != 30 && joyMessage->buttons[i] == 1 )
		{
			initJoy = initJoy | JOY_INIT_TAKEOFF ;
			joyCmd[JOY_CMD_TAKEOFF] = i ;			
			cout << "Button assigned to take off is set to button " << i << endl ;

			cout << "Please press and hold button to land (press any key when ready)"  ;  
			cin.get() ; 
		}
		else if ( ( initJoy & JOY_INIT_LAND ) != JOY_INIT_LAND && i != 30 && joyMessage->buttons[i] == 1 )
		{
			initJoy = initJoy | JOY_INIT_LAND ;
			joyCmd[JOY_CMD_LAND] = i ;
			cout << "Button assigned to land is set to button " << i << endl ;
		}
		else if ( ( initJoy & JOY_INIT_MOVE_FORWARD ) != JOY_INIT_MOVE_FORWARD && i != 30 && joyMessage->axes[i] == 1 )
		{
			initJoy = initJoy | JOY_INIT_MOVE_FORWARD ;
			joyCmd[JOY_CMD_MOVE_FORWARD] = i ;
			cout << "Axe assigned to move forward is set to axes " << i << endl ;
		}
		else if ( ( initJoy & JOY_INIT_MOVE_RIGHT ) != JOY_INIT_MOVE_RIGHT && i != 30 && joyMessage->buttons[i] == 1 )
		{
			initJoy = initJoy | JOY_INIT_MOVE_RIGHT ;
			joyCmd[JOY_CMD_MOVE_RIGHT] = i ;
			cout << "Button assigned to land is set to button " << i << endl ;
		}
	}
	else
	{
		if ( joyMessage->buttons[joyCmd[JOY_CMD_SWITCH]] == 1 )
		{
			if ( typeControl == AUTO_CONTROL )
				typeControl = MANUAL_CONTROL ;
			else
			 	typeControl = AUTO_CONTROL ;			
		}
		else if ( typeControl == MANUAL_CONTROL )
		{
			if ( joyMessage->buttons[joyCmd[JOY_CMD_LAND]] == 1 )			
			{
				landTopic.publish(emptyMsg) ;		
			}
			else if ( joyMessage->buttons[joyCmd[JOY_CMD_TAKEOFF]] == 1 )
			{
				takeoff.publish(emptyMsg) ;		
			}
			else
			{
				
			}
		}
	}	
}

/* Callback for image treatment */
void imageCallback ( const sensor_msgs::ImageConstPtr& original_image)
{
	
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
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}


	/* OpenCV algorithm for ball detection  */
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
	ros::init(argc, argv, "process_images");
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
	cout << "Please press and hold button to switch between auto control and manual control mode (press anykey when ready)"  ;   
	cin.get() ; 
	ros::Subscriber subJoy = nh.subscribe<sensor_msgs::Joy>("/joy", 1, joyCallback);

	destroyAllWindows();

	ros::spin();

	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}

