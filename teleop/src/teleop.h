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

  enum led_color
  {
    BLINK_GREEN_RED      = 0 ,
    BLINK_GREEN          = 1 ,
    BLINK_RED            = 2 ,
    BLINK_ORANGE         = 3 ,
    SNAKE_GREEN_RED      = 4 ,
    FIRE                 = 5 ,
    STANDARD             = 6 ,
    RED                  = 7 ,
    GREEN                = 8 ,
    RED_SNAKE            = 9 ,
    BLANK                = 10,
    LEFT_GREEN_RIGHT_RED = 11,
    LEFT_RED_RIGHT_GREEN = 12,
    BLINK_STANDARD       = 13
  };

  void ball_position_callback(red_ball_tracker::TrackerMsg::ConstPtr position);
  void joy_command_callback(sensor_msgs::Joy::ConstPtr joy);
  void blink_led(led_color color, float frequency, uint8_t duration);

  ros::NodeHandle _public_node;
  ros::NodeHandle _private_node;

  ros::Publisher _ardrone_reset_publisher;
  ros::Publisher _ardrone_land_publisher;
  ros::Publisher _ardrone_takeoff_publisher;
  ros::Publisher _ardrone_velocity_publisher;

  ros::Subscriber _ball_position_subscriber;
  ros::Subscriber _joy_command_subscriber;

  ros::ServiceClient _led_client;

  std::string
  _ardrone_reset_topic_name,
  _ardrone_land_topic_name,
  _ardrone_takeoff_topic_name,
  _ardrone_velocity_topic_name,
  _ardrone_led_service_name,
  _ball_position_topic_name,
  _joy_command_topic_name;

  bool _controlled_by_joy, _last_toggle_control;
  bool _move, _toggle_move, _last_toggle_move;
  bool _is_flying, _toggle_flying, _last_toggle_flying;
  bool _toggle_emergency, _last_toggle_emergency;
  bool _activate, _last_activate;
  float _scale;
  float _user_distance;
  int _ball_miss;
  bool _ball_detected;

  geometry_msgs::Twist _control;
  float _x_prev;
  float _x_p, _x_i, _x_d;
  int _rate_ms;

  struct p_corrector
  {
    p_corrector(
        float dxp, float dxi, float dxd,
        float dz,
        float tz)
      : dist_x_p(dxp), dist_x_i(dxp), dist_x_d(dxp)
      , dist_z_p(dz), theta_z_p(tz) {}

    float
    dist_x_p, dist_x_i, dist_x_d,
    dist_z_p, theta_z_p;
  }
  _coeff;
};

}
