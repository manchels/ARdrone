#include "red_ball_tracker.h"

#include <initializer_list>


#define LOW_PASS_FILTER { 0.5f, 0.25f, 0.25f }

// Static const members definitions.
namespace rbt {


float const red_ball_tracker::_ball_radius = 0.035f;
float const red_ball_tracker::_pixel_to_rad = 0.002f;
std::string const red_ball_tracker::_hue_window = "hue channel";
std::string const red_ball_tracker::_lightness_window = "lightness channel";
std::string const red_ball_tracker::_saturation_window = "saturation channel";
std::string const red_ball_tracker::_filtered_window = "filtered image";
std::string const red_ball_tracker::_contour_window = "contour image";
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
  , _idle_counter(0)
  , _alphax_filter(LOW_PASS_FILTER)
  , _alphay_filter(LOW_PASS_FILTER)
  , _distance_filter(LOW_PASS_FILTER)
  , _prev_distance(0.f)
  , _hsl_min(130,70,210)
  , _hsl_max(200,220,255)
{
  // Get Parameters.
  {
    _private_node.param("image", _image_topic_name, std::string("ardrone/image_raw"));

    // HSL parameters.
    {
      int min_h, min_s, min_l, max_h, max_s, max_l;

      _private_node.param("min_hue"       , min_h, 130);
      _private_node.param("min_saturation", min_s, 70 );
      _private_node.param("min_lightness" , min_l, 210);
      _private_node.param("max_hue"       , max_h, 200);
      _private_node.param("max_saturation", max_s, 220);
      _private_node.param("max_lightness" , max_l, 255);

      _hsl_min = cv::Scalar(min_h, min_s, min_l);
      _hsl_max = cv::Scalar(max_h, max_s, max_l);
    }
  }

  // Set topic names.
  {
    _tracking_topic_name = _image_topic_name + "/red_ball_tracking";
    _display_topic_name  = _image_topic_name + "/display_tracking";
  }

  // Subscribers.
  {
    _image_subscriber =
        _image_transport.subscribe(
          _image_topic_name, 1,
          boost::bind(&red_ball_tracker::image_callback, this, _1));
  }

  // Publishers.
  {
    _tracking_publisher =
        _public_node.advertise< ::red_ball_tracker::TrackerMsg>(
          _tracking_topic_name, 100);

    _image_display_publisher =
        _public_node.advertise<std_msgs::Empty>(
          _display_topic_name, 1,
          boost::bind(&red_ball_tracker::display_connection_callback, this),
          boost::bind(&red_ball_tracker::display_connection_callback, this));
  }

  // Init display if needed.
  display_connection_callback();

  // Info.
  {
    sleep(2);
    ROS_INFO("Create red_ball_tracker object.");
    ROS_INFO("Image topic : %s", _image_topic_name.c_str());
    ROS_INFO("Tracking topic : %s", _tracking_topic_name.c_str());
    ROS_INFO("Display topic : %s", _display_topic_name.c_str());
    ROS_INFO("To display the image filtering, please run the "
             "display_tracking node");
    ROS_INFO("Min HSL: [ %.5f, %.5f, %.5f ]\n"
             "Max HSL: [ %.5f, %.5f, %.5f ]",
             _hsl_min[0], _hsl_min[1], _hsl_min[2],
             _hsl_max[0], _hsl_max[1], _hsl_max[2]);
  }
}


red_ball_tracker::~red_ball_tracker(void)
{ ROS_INFO("Destroy red_ball_tracker object."); }


void red_ball_tracker::spin_once(void)
{
  // Display images if needed.
  if (_need_display_img && _image_origin && _image_filtered)
  {
    display_callback();

    cv::waitKey(1);
  }

  // To force disconnection when it has to happen (otherwise huge latency).
  _image_display_publisher.publish(std_msgs::Empty());

  // Increase idle counter.
  ++_idle_counter;
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
  float diff = 1000;
  bool ballFound = false;

  if (!_is_init)
  {
    _image_center.y = img->height/2;
    _image_center.x = img->width/2;
    _is_init = false;

    try
    {
      _image_filtered =
          cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Exception while trying to copy image: %s", e.what());
      return;
    }
  }

  try
  {
    _image_origin =
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
          CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }

  if (_need_display_img)
    cv::drawContours(_image_tmp, contours, -1, 255);

  // Result message.
  typedef std::vector<std::vector<cv::Point> >::iterator contours_iterator;
  for (contours_iterator iteratorContours = contours.begin();
       iteratorContours != contours.end(); ++iteratorContours)
  {
    cv::minEnclosingCircle(*iteratorContours, center, radius);
    if (_need_display_img)
      cv::circle(_image_tmp, center, radius, 125, 2);

    if (radius > 8.f && // Avoid too small circles.
        std::abs(_prev_distance - _ball_radius /
                 (std::tan(radius * _pixel_to_rad))) < diff)
    {
      ballFound = true;

      diff = std::abs(
               _prev_distance -
               _ball_radius / (std::tan(radius * _pixel_to_rad)));

      _ball_center_in_image = center;
      _ball_radius_in_image = radius;
    }
  }

  _prev_distance = _teleop_msg.distance;

  if (ballFound)
  {
    set_teleop_msg(
          _pixel_to_rad * (_ball_center_in_image.x - _image_center.x),
          _pixel_to_rad * (_image_center.y - _ball_center_in_image.y),
          _ball_radius / (std::tan(_ball_radius_in_image * _pixel_to_rad)));
    _tracking_publisher.publish(_teleop_msg);

    if (_need_display_img)
      cv::circle(_image_tmp, _ball_center_in_image,
                 _ball_radius_in_image, 125, 4);

    ROS_DEBUG("CtrX,CtrY,Rad: [ %10.5f, %10.5f, %10.5f ]",
              center.x, center.y, radius);
    ROS_DEBUG("Teleop msg:    [ %10.5f, %10.5f, %10.5f ]",
              _teleop_msg.alphax, _teleop_msg.alphay, _teleop_msg.distance);
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
  cv::imshow(_contour_window, _image_tmp);
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
    cv::namedWindow(_contour_window);

    cv::moveWindow(_hue_window, 0, 0);
    cv::moveWindow(_saturation_window, 320, 0);
    cv::moveWindow(_lightness_window, 640, 0);
    cv::moveWindow(_filtered_window, 160, 300);
    cv::moveWindow(_contour_window, 480, 300);
  }

  if (!_need_display_img && prev_need)
  {
    ROS_INFO("Unsubscribtion.");
    cv::destroyAllWindows();
  }
}


void red_ball_tracker::set_teleop_msg(
    float alphax, float alphay, float distance)
{
  if (_idle_counter < 5)
  {
    _teleop_msg.alphax   = _alphax_filter(alphax);
    _teleop_msg.alphay   = _alphay_filter(alphay);
    _teleop_msg.distance = _distance_filter(distance);
  }
  else
  {
    _teleop_msg.alphax   = _alphax_filter.reset(alphax);
    _teleop_msg.alphay   = _alphay_filter.reset(alphay);
    _teleop_msg.distance = _distance_filter.reset(distance);
  }

  _idle_counter = 0;
}


} // rbt
