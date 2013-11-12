#include "teleop.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");

  teleop::teleop teleop;
  teleop.spin();

  return EXIT_SUCCESS;

}
