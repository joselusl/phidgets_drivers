#include "phidgets_gps/gps_ros_i.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsGps");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::GpsAsyncRosI gps(nh, nh_private);
  gps.initDevice();
  ros::spin();
  return 0;
}
