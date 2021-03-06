#include "phidgets_imu/imu_ros_i.h"

namespace phidgets {

ImuRosI::ImuRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
  Imu(),
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false)
{
  ROS_INFO ("Starting Phidgets IMU");

  // **** get parameters

    // Perido -> rate
  if (!nh_private_.getParam ("period", period_))
    period_ = 8; // 8 ms

    // Frame ID
  if (!nh_private_.getParam ("frame_id", frame_id_))
    frame_id_ = "imu";

    // Std Dev
  if (!nh_private_.getParam ("angular_velocity_stdev", angular_velocity_stdev_))
    angular_velocity_stdev_ = 0.02 * (M_PI / 180.0); // 0.02 deg/s resolution, as per manual
  if (!nh_private_.getParam ("linear_acceleration_stdev", linear_acceleration_stdev_))
    linear_acceleration_stdev_ = 300.0 * 1e-6 * G; // 300 ug as per manual

  // Serial number
  if(!nh_private_.getParam ("serial_number", serial_number))
      serial_number = -1; // -1: means -> connect anyone

  // Advertised topic names
  if(!nh_private_.getParam ("imu_raw_data_topic_name", imu_raw_data_topic_name))
      imu_raw_data_topic_name = "phidgets_imu/imu/data_raw";
  if(!nh_private_.getParam ("mag_raw_data_topic_name", mag_raw_data_topic_name))
      mag_raw_data_topic_name = "phidgets_imu/mag/data_raw";
  if(!nh_private_.getParam ("imu_is_calibrated_topic_name", imu_is_calibrated_topic_name))
      imu_is_calibrated_topic_name = "phidgets_imu/is_calibrated";

  // Advertised service names
  if(!nh_private_.getParam ("imu_calibrate_service_name", imu_calibrate_service_name))
      imu_calibrate_service_name = "phidgets_imu/calibrate";

  // **** advertise topics

  imu_publisher_ = nh_.advertise<ImuMsg>(
    imu_raw_data_topic_name, 5);
  mag_publisher_ = nh_.advertise<MagMsg>(
    mag_raw_data_topic_name, 5);
  cal_publisher_ = nh_.advertise<std_msgs::Bool>(
    imu_is_calibrated_topic_name, 5);

  // **** advertise services

  cal_srv_ = nh_.advertiseService(
    imu_calibrate_service_name, &ImuRosI::calibrateService, this);

  // **** initialize variables and device
  
  imu_msg_.header.frame_id = frame_id_;

  // build covariance matrices

  double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;
  double lin_acc_var = linear_acceleration_stdev_ * linear_acceleration_stdev_;

  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
  {
    int idx = j*3 +i;

    if (i == j)
    {
      imu_msg_.angular_velocity_covariance[idx]    = ang_vel_var;
      imu_msg_.linear_acceleration_covariance[idx] = lin_acc_var;
    }
    else
    {
      imu_msg_.angular_velocity_covariance[idx]    = 0.0;
      imu_msg_.linear_acceleration_covariance[idx] = 0.0;
    }
  }

  initDevice();
}

void ImuRosI::initDevice()
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

	// set the data rate for the spatial events
  setDataRate(period_);

  // TODO -> check if we need this!
  // calibrate on startup
  calibrate();
}

bool ImuRosI::calibrateService(std_srvs::Empty::Request  &req,
                               std_srvs::Empty::Response &res)
{
  calibrate();
  return true;
}

void ImuRosI::calibrate()
{
  ROS_INFO("Calibrating IMU...");
  zero();
  ROS_INFO("Calibrating IMU done.");

  time_zero_ = ros::Time::now();

  // publish message
  std_msgs::Bool is_calibrated_msg;
  is_calibrated_msg.data = true;
  cal_publisher_.publish(is_calibrated_msg);
}

void ImuRosI::processImuData(CPhidgetSpatial_SpatialEventDataHandle* data, int i)
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


  // JL: Magnetic field info: http://www.phidgets.com/docs/1044_User_Guide
  if (data[i]->magneticField[0] != PUNK_DBL)
  {
    mag_msg->vector.x = data[i]->magneticField[0];
    mag_msg->vector.y = data[i]->magneticField[1];
    mag_msg->vector.z = data[i]->magneticField[2];

    mag_publisher_.publish(mag_msg);
  }
  else
  {
      // JL: Do not publish

//    double nan = std::numeric_limits<double>::quiet_NaN();

//    mag_msg->vector.x = nan;
//    mag_msg->vector.y = nan;
//    mag_msg->vector.z = nan;


  }
   
    //mag_publisher_.publish(mag_msg);

  return;
}

void ImuRosI::dataHandler(CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
  for(int i = 0; i < count; i++)
    processImuData(data, i);
}

} // namespace phidgets

