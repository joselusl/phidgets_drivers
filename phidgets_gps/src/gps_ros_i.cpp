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

  // Init device
  initDevice();
}

void GpsRosI::initDevice()
{
	ROS_INFO("Opening device");
    open(serial_number);

	ROS_INFO("Waiting for IMU to be attached...");
	int result = waitForAttachment(10000);
	if(result)
	{
	  const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_FATAL("Problem waiting for IMU attachment: %s Make sure the USB cable is connected and you have executed the phidgets_c_api/setup-udev.sh script.", err);
	}


}



void GpsRosI::positionChangeHandler(double latitude, double longitude, double altitude,
                                           double heading,
                                           double velocity,
                                           GPSDate date, GPSTime time,
                                           NMEAData NMEAdata)
{
    processNavSatFixData(latitude, longitude, altitude);
    processTimeReferenceData(date, time);
    return;
}

void GpsRosI::positionFixStatusChangeHandler(int status)
{
    processNavSatStatusData(status);
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


/*
void GpsRosI::processImuData(CPhidgetSpatial_SpatialEventDataHandle* data, int i)
{
  // **** calculate time from timestamp
  ros::Duration time_imu(data[i]->timestamp.seconds + 
                         data[i]->timestamp.microseconds * 1e-6);

  ros::Time time_now = time_zero_ + time_imu;

  double timediff = time_now.toSec() - ros::Time::now().toSec();
  if (fabs(timediff) > 0.1)
  {
    ROS_WARN("IMU time lags behind by %f seconds, resetting IMU time offset!", timediff);
    time_zero_ = ros::Time::now() - time_imu;
    time_now = ros::Time::now();
  }

  // **** initialize if needed

  if (!initialized_)
  { 
    last_imu_time_ = time_now;
    initialized_ = true;
  }

  // **** create and publish imu message

  boost::shared_ptr<ImuMsg> imu_msg = 
    boost::make_shared<ImuMsg>(imu_msg_);

  imu_msg->header.stamp = time_now;

  // set linear acceleration
  imu_msg->linear_acceleration.x = - data[i]->acceleration[0] * G;
  imu_msg->linear_acceleration.y = - data[i]->acceleration[1] * G;
  imu_msg->linear_acceleration.z = - data[i]->acceleration[2] * G;

  // set angular velocities
  imu_msg->angular_velocity.x = data[i]->angularRate[0] * (M_PI / 180.0);
  imu_msg->angular_velocity.y = data[i]->angularRate[1] * (M_PI / 180.0);
  imu_msg->angular_velocity.z = data[i]->angularRate[2] * (M_PI / 180.0);

  imu_publisher_.publish(imu_msg);

  // **** create and publish magnetic field message

  boost::shared_ptr<MagMsg> mag_msg = 
    boost::make_shared<MagMsg>();
  
  mag_msg->header.frame_id = frame_id_;
  mag_msg->header.stamp = time_now;

  if (data[i]->magneticField[0] != PUNK_DBL)
  {
    mag_msg->vector.x = data[i]->magneticField[0];
    mag_msg->vector.y = data[i]->magneticField[1];
    mag_msg->vector.z = data[i]->magneticField[2];
  }
  else
  {
    double nan = std::numeric_limits<double>::quiet_NaN();

    mag_msg->vector.x = nan;
    mag_msg->vector.y = nan;
    mag_msg->vector.z = nan;
  }
   
  mag_publisher_.publish(mag_msg);
}

void GpsRosI::dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  for(int i = 0; i < count; i++)
    processImuData(data, i);
}
*/


} // namespace phidgets

