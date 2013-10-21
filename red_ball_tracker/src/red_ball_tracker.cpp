#include "red_ball_tracker.h"

namespace rbt {


cv::Scalar const red_ball_tracker::_hsl_min = cv::Scalar(164,0,233);
cv::Scalar const red_ball_tracker::_hsl_max = cv::Scalar(255,255,255);
float const red_ball_tracker::_ball_radius = 0.035f;
float const red_ball_tracker::_pixel_to_rad = 0.002f;


red_ball_tracker::red_ball_tracker(void)
  : _private_node(ros::NodeHandle(std::string("~")))
  , _image_transport(_public_node)
  , _need_display_img(false)
  , _is_init(true)
  , _prev_distance(-1.f)
{
  ROS_INFO("Create red_ball_tracker object.");

  _private_node.param(
        "image", _image_topic_name,
        std::string("ardrone/image_raw"));
  ROS_INFO("Image topic : %s", _image_topic_name.c_str());

  _tracking_topic_name =
      _image_topic_name
      + "/red_ball_tracking";
  ROS_INFO("Tracking topic : %s", _tracking_topic_name.c_str());

  _image_subscriber =
      _image_transport.subscribe(
        _image_topic_name, 1,
        boost::bind(&red_ball_tracker::image_callback, this, _1));

  _tracking_publisher =
      _public_node.advertise< ::red_ball_tracker::TrackerMsg>(
        _tracking_topic_name, 100);
}


red_ball_tracker::~red_ball_tracker(void)
{
  ROS_INFO("Destroy red_ball_tracker object.");
}


void red_ball_tracker::spinOnce(void) {}


void red_ball_tracker::spin(void)
{
  while (ros::ok())
  {
    ros::Rate rate(50);
    spinOnce();
    ros::spinOnce();
    rate.sleep();
  }
}


void red_ball_tracker::image_callback(
    sensor_msgs::Image::ConstPtr img)
{
  // Initialisation of the two images :
  //   source and destination (image processed).
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr cv_chg;

  // Result message.
  ::red_ball_tracker::TrackerMsg teleopMsg;

  cv::Mat imageFiltered;

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> contoursHierarchy;
  std::vector<std::vector<cv::Point> >::iterator iteratorContours;

  cv::Mat erodeElement =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat dilateElement =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  cv::Point2f center;
  float radius;
  float dist = 1000;
  float diff = 1000;
  bool ballFound = false;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv_chg = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Exception trying to copy image : %s", e.what());
    return;
  }

  // Conversion BGR to HLS.
  cv::cvtColor(cv_ptr->image, cv_chg->image, CV_BGR2HLS);

  // Application of thresholds.
  cv::inRange(
        cv_chg->image, _hsl_min, _hsl_max, imageFiltered);

  // Noise cancelation.
  cv::erode(imageFiltered, imageFiltered, erodeElement);
  cv::erode(imageFiltered, imageFiltered, erodeElement);
  cv::dilate(imageFiltered, imageFiltered, dilateElement);
  cv::dilate(imageFiltered, imageFiltered, dilateElement);


  // Contour determination.
  cv::Canny(imageFiltered, imageFiltered, 100, 200);
  cv::findContours(
        imageFiltered, contours, contoursHierarchy,
        CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  for (iteratorContours = contours.begin();
       iteratorContours != contours.end() ;
       iteratorContours++)
  {
    ballFound = true;

    cv::minEnclosingCircle ( *iteratorContours, center, radius ) ;

    if (_is_init)
    {
      if (dist > std::abs(_ball_radius / (std::tan(radius * _pixel_to_rad))))
      {
        dist = std::abs(_ball_radius / (std::tan(radius * _pixel_to_rad)));

        teleopMsg.distance = _ball_radius / (std::tan(radius * _pixel_to_rad));
        teleopMsg.alphax = _pixel_to_rad * (center.x - 320);
        teleopMsg.alphay = _pixel_to_rad * (-center.y + 240);
      }
      _is_init = false;
    }
    else
    {
      if (std::abs(_prev_distance - _ball_radius /
                   (std::tan(radius * _pixel_to_rad)))
          < diff)
      {
        diff = std::abs(
              _prev_distance - _ball_radius /
              (std::tan(radius * _pixel_to_rad)));

        teleopMsg.distance = _ball_radius / (std::tan(radius * _pixel_to_rad));
        teleopMsg.alphax = _pixel_to_rad * (center.x - 320);
        teleopMsg.alphay = _pixel_to_rad * (-center.y + 240);
      }
    }
  }

  _prev_distance = teleopMsg.distance;

  if (ballFound) _tracking_publisher.publish(teleopMsg);
}


void red_ball_tracker::display_callback(void)
{
  // TODO: Display images on good windows.
}


void red_ball_tracker::display_connection_callback(void)
{
  bool prev_need = _need_display_img;
  _need_display_img = (_image_display_publisher.getNumSubscribers() > 0);

  if (_need_display_img && !prev_need)
  {
    // TODO: Create cv::window.
  }

  if (!_need_display_img && prev_need)
  {
    // TODO: Destroy cv::window.
  }
}


}
