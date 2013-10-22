#include "red_ball_tracker.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball_tracker");

  rbt::red_ball_tracker tracker;
  tracker.spin();

  return EXIT_SUCCESS;
}
