#include "teleop.h"

#include <cassert>


#define STRINGIZE(args...) __STRING(args)

#define TOGGLE_MOVE      buttons[0]
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
  , _controlled_by_joy(false)
  , _last_toggle_control(false)
  , _move(false)
  , _toggle_move(false)
  , _last_toggle_move(false)
  , _is_flying(false)
  , _toggle_flying(false)
  , _last_toggle_flying(false)
  , _toggle_emergency(false)
  , _last_toggle_emergency(false)
  , _scale(1.f)
  , _x_prev(0.f)
  , _x_p(0.f), _x_i(0.f), _x_d(0.f)
  , _rate_ms(50)
  , _coeff(0.f,0.f,0.f,0.f,0.f)
{
  // Parameters.
  {
    // Topics.
    {
      _private_node.param("reset"     , _ardrone_reset_topic_name     , std::string("ardrone/reset")                      );
      _private_node.param("land"      , _ardrone_land_topic_name      , std::string("ardrone/land")                       );
      _private_node.param("takeoff"   , _ardrone_takeoff_topic_name   , std::string("ardrone/takeoff")                    );
      _private_node.param("velocity"  , _ardrone_velocity_topic_name  , std::string("cmd_vel")                            );
      _private_node.param("navigation", _ardrone_navigation_topic_name, std::string("ardrone/navdata")                    );
      _private_node.param("ball"      , _ball_position_topic_name     , std::string("ardrone/image_raw/red_ball_tracking"));
      _private_node.param("joy"       , _joy_command_topic_name       , std::string("joy")                                );
    }

    // Coefficient found by Ziegler-Nichols method.
    {
      double dxp, dxi, dxd, dzp, tzp;

      _private_node.param("dist_x_p" , dxp, 3.0 );
      _private_node.param("dist_x_i" , dxi, 1.0 );
      _private_node.param("dist_x_d" , dxd, 0.25);
      _private_node.param("dist_z_p" , dzp, 10.0);
      _private_node.param("theta_z_p", tzp, 2.0 );

      _coeff.dist_x_p  = dxp;
      _coeff.dist_x_i  = dxi;
      _coeff.dist_x_d  = dxd;
      _coeff.dist_z_p  = dzp;
      _coeff.theta_z_p = tzp;
    }

    // Distence to user.
    {
      double user_dist;

      _private_node.param("user_distance", user_dist, 1.5);
      _user_distance = user_dist;
    }
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

  // Info.
  {
    sleep(2);
    ROS_INFO("Create teleop object.");
    ROS_INFO("Ardrone reset topic      : %s", _ardrone_reset_topic_name.c_str()     );
    ROS_INFO("Ardrone land topic       : %s", _ardrone_land_topic_name.c_str()      );
    ROS_INFO("Ardrone reset topic      : %s", _ardrone_reset_topic_name.c_str()     );
    ROS_INFO("Ardrone velocity topic   : %s", _ardrone_velocity_topic_name.c_str()  );
    ROS_INFO("Ardrone navigation topic : %s", _ardrone_navigation_topic_name.c_str());
    ROS_INFO("Ball position topic      : %s", _ball_position_topic_name.c_str()     );
    ROS_INFO("Joy command topic        : %s", _joy_command_topic_name.c_str()       );
    ROS_INFO(
          "Joystick commands are:\n"
          "\tToggle move:      " STRINGIZE(TOGGLE_MOVE     ) "\n"
          "\tToggle control:   " STRINGIZE(TOGGLE_CONTROL  ) "\n"
          "\tToggle emergency: " STRINGIZE(TOGGLE_EMERGENCY) "\n"
          "\tToggle flying:    " STRINGIZE(TOGGLE_EMERGENCY) "\n"
          "\tLateral axe:      " STRINGIZE(LATERAL         ) "\n"
          "\tForward axe:      " STRINGIZE(FORWARD         ) "\n"
          "\tYaw axe:          " STRINGIZE(YAW             ) "\n"
          "\tAltitude axe:     " STRINGIZE(ALTITUDE        ) "\n");
    ROS_INFO(
          "Controller parameters:\n"
          "\tDistance.X PID: [ %10.5f, %10.5f, %10.5f ]\n"
          "\tDistance.Z P:   [ %10.5f ]\n"
          "\tAngle.Z    P:   [ %10.5f ]\n",
          _coeff.dist_x_p, _coeff.dist_x_i, _coeff.dist_x_d,
          _coeff.dist_z_p,
          _coeff.theta_z_p);
    ROS_INFO("User distance: %f", _user_distance);
  }
}


teleop::~teleop(void) { ROS_INFO("Destroy teleop object."); }


void teleop::spin_once(void)
{
  // Stabilize when not controlled.
  {
    _control.angular.x *= 0.9f;
    _control.angular.y *= 0.9f;
    _control.angular.z *= 0.9f;

    _control.linear.x *= 0.9f;
    _control.linear.y *= 0.9f;
    _control.linear.z *= 0.9f;
  }

  // Update values.
  {
    _x_i += (_control.linear.x - _x_prev) * _rate_ms / 1000.f;
    _x_d  = 1000.f * (_control.linear.x - _x_prev) / _rate_ms;

    _x_prev = _control.linear.x;
  }

  // Publish velocity.
  _ardrone_velocity_publisher.publish(_control);
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
  if (_move && !_controlled_by_joy)
  {
    if (_is_flying)
    {
      _x_p = _user_distance - position->distance;
      _control.linear.x =
          -(_coeff.dist_x_p * _x_p +
            _coeff.dist_x_i * _x_i +
            _coeff.dist_x_d * _x_d);
      _control.linear.y = 0;
      _control.linear.z = 0; //-_coeff.dist_z * position->alphay;

      _control.angular.x = 0;
      _control.angular.y = 0;
      _control.angular.z = -(_coeff.theta_z_p * position->alphax);

      ROS_DEBUG("[ x_p, x_i, x_d ] = [ %10.5f, %10.5f, %10.5f ]",
                _x_p, _x_i, _x_d);
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
  if (joy->TOGGLE_MOVE && !_last_toggle_move)
  {
    _move = !_move;
    ROS_INFO("%s moving.", _move ? "Start" : "Stop");
  }
  _last_toggle_move = joy->TOGGLE_MOVE;

  if (joy->TOGGLE_CONTROL && !_last_toggle_control)
  {
    _controlled_by_joy = !_controlled_by_joy;
    ROS_INFO("Set control: %s.",
             _controlled_by_joy
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
  if (_move && _controlled_by_joy)
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


} // teleop
