#include "red_ball_tracker.h"


// Static const members definitions.
namespace rbt {


cv::Scalar const red_ball_tracker::_hsl_min = cv::Scalar(164,0,233);
cv::Scalar const red_ball_tracker::_hsl_max = cv::Scalar(255,255,255);
float const red_ball_tracker::_ball_radius = 0.035f;
float const red_ball_tracker::_pixel_to_rad = 0.002f;
std::string const red_ball_tracker::_hue_window = "hue channel";
std::string const red_ball_tracker::_lightness_window = "lightness channel";
std::string const red_ball_tracker::_saturation_window = "saturation channel";
std::string const red_ball_tracker::_filtered_window = "filtered image";
cv::Mat const red_ball_tracker::_erode_element =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::Mat const red_ball_tracker::_dilate_element =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));


} // rbt


// Methods definitions.
namespace rbt {


red_ball_tracker::red_ball_tracker(void)
  : _private_node(ros::NodeHandle(std::string("~")))
  , _image_transport(_public_node)
  , _need_display_img(false)
  , _is_init(false)
  , _prev_distance(-1.f)
{
  ROS_INFO("Create red_ball_tracker object.");

  _private_node.param(
        "image", _image_topic_name, std::string("ardrone/image_raw"));
  ROS_INFO("Image topic : %s", _image_topic_name.c_str());

  _tracking_topic_name = _image_topic_name + "/red_ball_tracking";
  ROS_INFO("Tracking topic : %s", _tracking_topic_name.c_str());

  _display_topic_name = _image_topic_name + "/display_tracking";
  ROS_INFO("Display topic : %s", _display_topic_name.c_str());

  ROS_INFO("To display the image filtering, please run the "
           "display_tracking_node");

  _image_subscriber =
      _image_transport.subscribe(
        _image_topic_name, 1,
        boost::bind(&red_ball_tracker::image_callback, this, _1));

  _tracking_publisher =
      _public_node.advertise< ::red_ball_tracker::TrackerMsg>(
        _tracking_topic_name, 100);

  _image_display_publisher =
      _public_node.advertise<std_msgs::Empty>(
        _display_topic_name, 1,
        boost::bind(&red_ball_tracker::display_connection_callback, this),
        boost::bind(&red_ball_tracker::display_connection_callback, this));

  // Init display if needed.
  display_connection_callback();
}


red_ball_tracker::~red_ball_tracker(void)
{
  ROS_INFO("Destroy red_ball_tracker object.");
}


void red_ball_tracker::spin_once(void)
{
  if (_need_display_img && _image_origin && _image_filtered)
    display_callback();

  // To force disconnection when it has to happen (otherwise huge latency).
  _image_display_publisher.publish(std_msgs::Empty());

  cv::waitKey(1);
}


void red_ball_tracker::spin(void)
{
  while (ros::ok())
  {
    ros::Rate rate(50);
    spin_once();
    ros::spinOnce();
    rate.sleep();
  }
}


void red_ball_tracker::image_callback(sensor_msgs::Image::ConstPtr img)
{
  cv::Point2f center;
  float radius;
  float dist = 1000;
  float diff = 1000;
  bool ballFound = false;

  if (!_is_init)
  {
    _image_center.y = img->height/2;
    _image_center.x = img->width/2;
  }

  try
  {
    _image_origin =
        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    _image_filtered =
        cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Exception while trying to copy image: %s", e.what());
    return;
  }

  // Conversion BGR to HLS.
  cv::cvtColor(_image_origin->image, _image_filtered->image, CV_BGR2HLS);

  // Application of thresholds.
  cv::inRange(_image_filtered->image, _hsl_min, _hsl_max, _image_tmp);

  // Noise cancelation.
  cv::erode(_image_tmp, _image_tmp, _erode_element);
  cv::erode(_image_tmp, _image_tmp, _erode_element);
  cv::dilate(_image_tmp, _image_tmp, _dilate_element);
  cv::dilate(_image_tmp, _image_tmp, _dilate_element);

  // Contour determination.
  std::vector<std::vector<cv::Point> > contours;
  {
    std::vector<cv::Vec4i> dummy;
    cv::Canny(_image_tmp, _image_tmp, 100, 200);
    cv::findContours(
          _image_tmp, contours, dummy,
          CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }

  // Result message.
  ::red_ball_tracker::TrackerMsg teleopMsg;

  typedef std::vector<std::vector<cv::Point> >::iterator contours_iterator;
  for (contours_iterator iteratorContours = contours.begin();
       iteratorContours != contours.end(); iteratorContours++)
  {
    ballFound = true;

    cv::minEnclosingCircle(*iteratorContours, center, radius);

    if (!_is_init)
    {
      if (dist > std::abs(_ball_radius / (std::tan(radius * _pixel_to_rad))))
      {
        dist = std::abs(_ball_radius / (std::tan(radius * _pixel_to_rad)));

        teleopMsg.distance = _ball_radius / (std::tan(radius * _pixel_to_rad));
        teleopMsg.alphax = _pixel_to_rad * (center.x - _image_center.x);
        teleopMsg.alphay = _pixel_to_rad * (_image_center.y - center.y);
      }
      _is_init = true;
    }
    else
    {
      if (std::abs(_prev_distance - _ball_radius /
                   (std::tan(radius * _pixel_to_rad))) < diff)
      {
        diff = std::abs(
              _prev_distance - _ball_radius /
              (std::tan(radius * _pixel_to_rad)));

        teleopMsg.distance = _ball_radius / (std::tan(radius * _pixel_to_rad));
        teleopMsg.alphax = _pixel_to_rad * (center.x - _image_center.x);
        teleopMsg.alphay = _pixel_to_rad * (_image_center.y - center.y);
      }
    }
  }

  _prev_distance = teleopMsg.distance;

  if (ballFound)
  {
    _tracking_publisher.publish(teleopMsg);

    ROS_DEBUG("CtrX,CtrY,Rad: [%10.5f, %10.5f, %10.5f]",
              center.x, center.y, radius);
    ROS_DEBUG("Image:         [%f,%f]", _image_center.x, _image_center.y);
    ROS_DEBUG("Teleop msg:    [%10.5f, %10.5f, %10.5f]",
              teleopMsg.alphax, teleopMsg.alphay, teleopMsg.distance);
  }
}


void red_ball_tracker::display_callback(void)
{
  cv::Mat hlsChannels[3];

  cv::split(_image_filtered->image, hlsChannels);

  cv::imshow(_hue_window, hlsChannels[0]);
  cv::imshow(_lightness_window, hlsChannels[1]);
  cv::imshow(_saturation_window, hlsChannels[2]);
  cv::imshow(_filtered_window, _image_filtered->image);
}


void red_ball_tracker::display_connection_callback(void)
{
  bool prev_need = _need_display_img;
  _need_display_img = (_image_display_publisher.getNumSubscribers() > 0);

  if (_need_display_img && !prev_need)
  {
    ROS_INFO("Subscribtion.");
    cv::namedWindow(_hue_window);
    cv::namedWindow(_saturation_window);
    cv::namedWindow(_lightness_window);
    cv::namedWindow(_filtered_window);

    cv::moveWindow(_hue_window, 0, 0);
    cv::moveWindow(_saturation_window, 320, 0);
    cv::moveWindow(_lightness_window, 640, 0);
    cv::moveWindow(_filtered_window, 320, 300);
  }

  if (!_need_display_img && prev_need)
  {
    ROS_INFO("Unsubscribtion.");
    cv::destroyAllWindows();
  }
}


} // rbt
