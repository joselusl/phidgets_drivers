#include "phidgets_gps/phidgets_gps_nodelet.h"

typedef phidgets::PhidgetsGpsNodelet PhidgetsGpsNodelet;

PLUGINLIB_DECLARE_CLASS (phidgets_gps, PhidgetsGpsNodelet, PhidgetsGpsNodelet, nodelet::Nodelet);

void PhidgetsGpsNodelet::onInit()
{
  NODELET_INFO("Initializing Phidgets GPS Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  gps_ = new GpsAsyncRosI(nh, nh_private);
}
