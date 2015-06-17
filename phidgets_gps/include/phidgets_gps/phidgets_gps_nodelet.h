#ifndef PHIDGETS_GPS_PHIDGETS_GPS_NODELET_H
#define PHIDGETS_GPS_PHIDGETS_GPS_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "phidgets_gps/gps_ros_i.h"

namespace phidgets {

class PhidgetsGpsNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    Gps * gps_;  // FIXME: change to smart pointer
};

} // namespace phidgets

#endif // PHIDGETS_GPS_PHIDGETS_GPS_NODELET_H
