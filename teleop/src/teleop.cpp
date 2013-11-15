#include "teleop.h"
#include <cassert>

#define STRINGIZE(args...) __STRING(args)

#define START            buttons[0]
#define TOGGLE_CONTROL   buttons[1]
#define TOGGLE_FLYING    buttons[2]
#define TOGGLE_EMERGENCY buttons[3]
#define LATERAL          axes[0]
#define FORWARD          axes[1]
#define YAW              axes[2]
#define ALTITUDE         axes[3]


// Methods definitions.
namespace teleop {


teleop::teleop(void)
  : _private_node(ros::NodeHandle(std::string("~")))
  , _altitude_mm(0)
  , _controlled_by_joy(false)
  , _last_toggle_control(false)
  , _start(false)
  , _is_flying(false)
  , _toggle_flying(false)
  , _last_toggle_flying(false)
  , _toggle_emergency(false)
  , _last_toggle_emergency(false)
  , _scale(1.f)
  , _x_prev(0.f)
  , _x_p(0.f), _x_i(0.f), _x_d(0.f)
  , _rate_ms(50)
  // Coefficient found by Ziegler-Nichols method.
  , _coeff(.3f, 1.f, 0.25f,
            10.f, 10.f)
{
  ROS_INFO("Create teleop object.");

  // Parameters.
  {
    _private_node.param(
          "reset", _ardrone_reset_topic_name, std::string("ardrone/reset"));
    ROS_INFO("Ardrone reset topic : %s", _ardrone_reset_topic_name.c_str());

    _private_node.param(
          "land", _ardrone_land_topic_name, std::string("ardrone/land"));
    ROS_INFO("Ardrone land topic : %s", _ardrone_land_topic_name.c_str());

    _private_node.param(
          "takeoff", _ardrone_takeoff_topic_name,
          std::string("ardrone/takeoff"));
    ROS_INFO("Ardrone reset topic : %s", _ardrone_reset_topic_name.c_str());

    _private_node.param(
          "velocity", _ardrone_velocity_topic_name, std::string("cmd_vel"));
    ROS_INFO("Ardrone velocity topic : %s",
             _ardrone_velocity_topic_name.c_str());

    _private_node.param(
          "navigation", _ardrone_navigation_topic_name,
          std::string("ardrone/navdata"));
    ROS_INFO("Ardrone navigation topic : %s",
             _ardrone_navigation_topic_name.c_str());

    _private_node.param(
          "ball", _ball_position_topic_name,
          std::string("ardrone/image_raw/red_ball_tracking"));
    ROS_INFO("Ball position topic : %s", _ball_position_topic_name.c_str());

    _private_node.param(
          "joy", _joy_command_topic_name, std::string("joy"));
    ROS_INFO("Joy command topic : %s", _joy_command_topic_name.c_str());
  }

  // Subscribers.
  {
    _ball_position_subscriber =
        _public_node.subscribe<red_ball_tracker::TrackerMsg>(
          _ball_position_topic_name, 1,
          boost::bind(&teleop::ball_position_callback, this, _1));

    _joy_command_subscriber =
        _public_node.subscribe<sensor_msgs::Joy>(
          _joy_command_topic_name, 1,
          boost::bind(&teleop::joy_command_callback, this, _1));

//    _ardrone_navigation_subscriber =
//        _public_node.subscribe<ardrone_autonomy::Navdata>(
//          _ardrone_navigation_topic_name, 1,
//          boost::bind(&teleop::navigation_callback, this, _1));
  }

  // Publishers.
  {
    _ardrone_land_publisher =
        _public_node.advertise<std_msgs::Empty>(
          _ardrone_land_topic_name, 10);

    _ardrone_takeoff_publisher =
        _public_node.advertise<std_msgs::Empty>(
          _ardrone_takeoff_topic_name, 10);

    _ardrone_reset_publisher =
        _public_node.advertise<std_msgs::Empty>(
          _ardrone_reset_topic_name, 10);

    _ardrone_velocity_publisher =
        _public_node.advertise<geometry_msgs::Twist>(
          _ardrone_velocity_topic_name, 10);
  }

  ROS_INFO(
        "Joystick commands are:\n"
        "\tStart:            " STRINGIZE(START           ) "\n"
        "\tToggle control:   " STRINGIZE(TOGGLE_CONTROL  ) "\n"
        "\tToggle emergency: " STRINGIZE(TOGGLE_EMERGENCY) "\n"
        "\tToggle flying:    " STRINGIZE(TOGGLE_EMERGENCY) "\n"
        "\tLateral axe:      " STRINGIZE(LATERAL         ) "\n"
        "\tForward axe:      " STRINGIZE(FORWARD         ) "\n"
        "\tYaw axe:          " STRINGIZE(YAW             ) "\n"
        "\tAltitude axe:     " STRINGIZE(ALTITUDE        ) "\n");
}


teleop::~teleop(void)
{ ROS_INFO("Destroy teleop object."); }


void teleop::spin_once(void)
{
  // Stabilize when not controlled.
  {
    _control.angular.x *= 0.5f;
    _control.angular.y *= 0.5f;
    _control.angular.z *= 0.5f;

    _control.linear.x *= 0.5f;
    _control.linear.y *= 0.5f;
    _control.linear.z *= 0.5f;
  }

  // Update values.
  {
    _x_i += (_control.linear.x - _x_prev) * _rate_ms / 1000.f;
    _x_d  = 1000.f * (_control.linear.x - _x_prev) / _rate_ms;

    _x_prev = _control.linear.x;
  }

  publish_velocity();
}


void teleop::spin(void)
{
  while (ros::ok())
  {
    ros::Rate rate(_rate_ms);
    spin_once();
    ros::spinOnce();
    rate.sleep();
  }
}


void teleop::ball_position_callback(
    red_ball_tracker::TrackerMsg::ConstPtr position)
{
  // Autonomous mode description.
  if (_start && !_controlled_by_joy)
  {
    if (_is_flying)
    {
      _x_p = 3.f - position->distance;
      _control.linear.x =
          -(_coeff.dist_x_p * _x_p +
            _coeff.dist_x_i * _x_i/* +
            _coeff.dist_x_d * _x_d*/);
      _control.linear.y = 0;
      _control.linear.z = 0; //-_coeff.dist_z * position->alphay;

      _control.angular.x = 0;
      _control.angular.y = 0;
      _control.angular.z = -(_coeff.theta_z * position->alphax);

      ROS_INFO("[ %10.5f, %10.5f, %10.5f ]", _x_p, _x_i, _x_d);
    }
    else
    {
      ROS_INFO("Taking off!");
      _ardrone_takeoff_publisher.publish(std_msgs::Empty());
      _is_flying = true;
    }
  }
}


void teleop::joy_command_callback(sensor_msgs::Joy::ConstPtr joy)
{
  if (joy->START && !_start)
  {
    ROS_INFO("Starting.");
    _start = true;
  }

  if (joy->TOGGLE_CONTROL && !_last_toggle_control)
  {
    _controlled_by_joy = !_controlled_by_joy;
    ROS_INFO("Set control: %s.",
             (_controlled_by_joy)
             ? "Joystick"
             : "Autonomous");
  }
  _last_toggle_control = joy->TOGGLE_CONTROL;

  // Emergency state.
  {
    _last_toggle_emergency = _toggle_emergency;
    _toggle_emergency = joy->TOGGLE_EMERGENCY;

    if (_toggle_emergency && !_last_toggle_emergency)
    {
      ROS_INFO("Changing emergency status.");
      _ardrone_reset_publisher.publish(std_msgs::Empty());
    }
  }

  // Joy controlled mode description.
  if (_start && _controlled_by_joy)
  {
    _control.linear.x  = _scale * joy->FORWARD;  // forward, backward
    _control.linear.y  = _scale * joy->LATERAL;  // left, right
    _control.linear.z  = _scale * joy->ALTITUDE; // up, down
    _control.angular.z = _scale * joy->YAW;      // yaw

    // Flying state.
    {
      _last_toggle_flying = _toggle_flying;
      _toggle_flying = joy->TOGGLE_FLYING;

      if (_toggle_flying && !_last_toggle_flying)
      {
        if (!_is_flying)
        {
          ROS_INFO("Taking off!");
          _ardrone_takeoff_publisher.publish(std_msgs::Empty());
          _is_flying = true;
        }
        else
        {
          ROS_INFO("Landing.");
          _ardrone_land_publisher.publish(std_msgs::Empty());
          _is_flying = false;
        }
      }

    }
  }
}


void teleop::navigation_callback(ardrone_autonomy::Navdata::ConstPtr navdata)
{ _altitude_mm = navdata->altd; }


void teleop::publish_velocity(void)
{ _ardrone_velocity_publisher.publish(_control); }


} // teleop
