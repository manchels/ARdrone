#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include <red_ball_tracker/TrackerMsg.h>


namespace rbt {

class red_ball_tracker
{
public:

  red_ball_tracker(void);
  ~red_ball_tracker(void);

  void spin_once(void);
  void spin(void);

private:

  static const cv::Scalar _hsl_min, _hsl_max;
  static const float _pixel_to_rad, _ball_radius;
  static const std::string
  _hue_window, _saturation_window, _lightness_window, _filtered_window;
  static const cv::Mat _erode_element, _dilate_element;

  void image_callback(sensor_msgs::Image::ConstPtr img);

  void display_callback(void);
  void display_connection_callback(void);

  ros::NodeHandle _public_node;
  ros::NodeHandle _private_node;

  image_transport::ImageTransport _image_transport;
  image_transport::Subscriber _image_subscriber;

  ros::Publisher _tracking_publisher;
  ros::Publisher _image_display_publisher;

  std::string _image_topic_name, _tracking_topic_name, _display_topic_name;

  bool _need_display_img;
  bool _is_init;

  float _prev_distance;

  cv::Mat _image_tmp;
  cv::Point2f _image_center;
  cv_bridge::CvImagePtr _image_origin;
  cv_bridge::CvImagePtr _image_filtered;
};

}
