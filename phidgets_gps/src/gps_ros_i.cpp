#include "phidgets_gps/gps_ros_i.h"

namespace phidgets
{

GpsRosI::GpsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
    Gps(),
    nh_(nh),
    nh_private_(nh_private),
    initialized_(false)
{
  ROS_INFO ("Starting Phidgets GPS");

  // **** get parameters


    // Frame ID
  if (!nh_private_.getParam ("frame_id", frame_id_))
    frame_id_ = "gps";

    // Std Dev
  if (!nh_private_.getParam ("position_stdev", position_stdev_))
    position_stdev_ = 0.02 * (M_PI / 180.0); // 0.02 deg/s resolution, as per manual


  // Serial number
  if(!nh_private_.getParam ("serial_number", serial_number))
      serial_number = -1; // -1: means -> connect anyone

  // Advertised topic names
  if(!nh_private_.getParam ("nav_sat_fix_topic_name", NavSatFix_topic_name))
      NavSatFix_topic_name = "phidgets_gps/gps/nav_sat_fix";
  if(!nh_private_.getParam ("nav_sat_status_topic_name", NavSatStatus_topic_name))
      NavSatStatus_topic_name = "phidgets_gps/gps/nav_sat_status";
  if(!nh_private_.getParam ("time_reference_topic_name", TimeReference_topic_name))
      TimeReference_topic_name = "phidgets_gps/time_reference";


  // **** advertise topics

  NavSatFix_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(NavSatFix_topic_name, 5);
  NavSatStatus_publisher_ = nh_.advertise<sensor_msgs::NavSatStatus>(NavSatStatus_topic_name, 5);
  TimeReference_publisher_ = nh_.advertise<sensor_msgs::TimeReference>(TimeReference_topic_name, 5);

  // **** initialize variables and device

  // Fill message constants and init values

  // NavSatStatus_msg
  NavSatStatus_msg.status=sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  NavSatStatus_msg.service=sensor_msgs::NavSatStatus::SERVICE_GPS;


  // NavSatFix_msg

  NavSatFix_msg.header.frame_id=frame_id_;

  NavSatFix_msg.status=NavSatStatus_msg;

  NavSatFix_msg.latitude=std::numeric_limits<double>::quiet_NaN();;
  NavSatFix_msg.longitude=std::numeric_limits<double>::quiet_NaN();;
  NavSatFix_msg.altitude=std::numeric_limits<double>::quiet_NaN();;

  // covarinace knowledge
  NavSatFix_msg.position_covariance_type=sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  // build covariance matrices
  double position_var = position_stdev_ * position_stdev_;
  for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
      {
        int idx = j*3 +i;

        if (i == j)
        {
          NavSatFix_msg.position_covariance[idx]    = position_var;
        }
        else
        {
          NavSatFix_msg.position_covariance[idx]    = 0.0;
        }
  }

  // TimeReference_msg
  TimeReference_msg.source="phidgets_gps_s_n_"+std::to_string(serial_number);

  // Init device -> NO!!
  //initDevice();


  //
  return;
}





//void GpsRosI::positionChangeHandler(double latitude, double longitude, double altitude,
//                                           double heading,
//                                           double velocity,
//                                           GPSDate date, GPSTime time,
//                                           NMEAData NMEAdata)
//{
//    processNavSatFixData(latitude, longitude, altitude);
//    processTimeReferenceData(date, time);
//    return;
//}

void GpsRosI::positionHandler(double latitude, double longitude, double altitude)
{
    this->processNavSatFixData(latitude, longitude, altitude);
    return;
}

void GpsRosI::headingHandler(double heading)
{
    return;
}

void GpsRosI::velocityHandler(double velocity)
{
    return;
}

void GpsRosI::dateAndTimeHandler(GPSDate date, GPSTime time)
{
    this->processTimeReferenceData(date, time);
    return;
}

void GpsRosI::nmeaDataHandler(NMEAData NMEAdata)
{
    return;
}


void GpsRosI::positionFixStatusChangeHandler(int status)
{
    this->processNavSatStatusData(status);
    return;
}



void GpsRosI::processNavSatFixData(double latitude, double longitude, double altitude)
{
    NavSatFix_msg.header.stamp=ros::Time::now();

    NavSatFix_msg.status=NavSatStatus_msg;

    NavSatFix_msg.latitude=latitude;
    NavSatFix_msg.longitude=longitude;
    NavSatFix_msg.altitude=altitude;

    NavSatFix_publisher_.publish(NavSatFix_msg);

    return;
}

void GpsRosI::processTimeReferenceData(GPSDate date, GPSTime time)
{

    TimeReference_msg.header.stamp=ros::Time::now();

    // TODO Fix!!
    TimeReference_msg.time_ref.sec=(time.tm_hour*24+time.tm_min)*60+time.tm_sec;
    TimeReference_msg.time_ref.nsec=time.tm_ms*1000;

    TimeReference_publisher_.publish(TimeReference_msg);

    return;
}

void GpsRosI::processNavSatStatusData(int status)
{
    NavSatStatus_msg.status=status;

    NavSatStatus_publisher_.publish(NavSatStatus_msg);

    return;
}





GpsAsyncRosI::GpsAsyncRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
    GpsAsync(),
    GpsRosI(nh,nh_private)
{

    return;
}


GpsSyncRosI::GpsSyncRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
    GpsSync(),
    GpsRosI(nh,nh_private)
{

    return;
}


} // namespace phidgets

