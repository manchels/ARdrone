#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <red_ball_tracker/TrackerMsg.h>


namespace teleop {

class teleop
{
public:

  teleop(void);
  ~teleop(void);

  void spin_once(void);
  void spin(void);

private:

  void ball_position_callback(red_ball_tracker::TrackerMsg::ConstPtr position);
  void joy_command_callback(sensor_msgs::Joy::ConstPtr joy);
  void navigation_callback(ardrone_autonomy::Navdata::ConstPtr navdata);

  inline void publish_velocity(void);

  ros::NodeHandle _public_node;
  ros::NodeHandle _private_node;

  ros::Publisher _ardrone_reset_publisher;
  ros::Publisher _ardrone_land_publisher;
  ros::Publisher _ardrone_takeoff_publisher;
  ros::Publisher _ardrone_velocity_publisher;

  ros::Subscriber _ball_position_subscriber;
  ros::Subscriber _joy_command_subscriber;
  ros::Subscriber _ardrone_navigation_subscriber;

  std::string
  _ardrone_reset_topic_name,
  _ardrone_land_topic_name,
  _ardrone_takeoff_topic_name,
  _ardrone_velocity_topic_name,
  _ardrone_navigation_topic_name,
  _ball_position_topic_name,
  _joy_command_topic_name;

  int _altitude_mm;
  bool _controlled_by_joy, _last_toggle_control;
  bool _start;
  bool _is_flying, _toggle_flying, _last_toggle_flying;
  bool _toggle_emergency, _last_toggle_emergency;
  bool _activate, _last_activate;
  float _scale;
  geometry_msgs::Twist _control;

  struct p_corrector
  {
    p_corrector(float dx, float dz, float tz)
      : dist_x(dx), dist_z(dz), theta_z(tz) {}

    float dist_x, dist_z, theta_z;
  }
  _coeff;
};

}
