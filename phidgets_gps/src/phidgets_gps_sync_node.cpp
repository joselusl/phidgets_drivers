#include "phidgets_gps/gps_ros_i.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsGps");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::GpsSyncRosI gps(nh, nh_private);
  gps.initDevice();

  while(ros::ok())
  {
      ros::spinOnce();
      gps.runStep();
  }
  return 0;
}
