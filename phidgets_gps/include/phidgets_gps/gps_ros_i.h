#ifndef PHIDGETS_GPS_ROS_I_H
#define PHIDGETS_GPS_ROS_I_H

#include <ros/ros.h>

// Not needed
//#include <boost/thread/mutex.hpp>

// Not needed
//#include <ros/service_server.h>


// Not needed
//#include <tf/transform_datatypes.h>
//#include <sensor_msgs/Imu.h>
//#include <std_srvs/Empty.h>
//#include <std_msgs/Bool.h>
//#include <geometry_msgs/Vector3Stamped.h>

// GPS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
// Time reference
#include <sensor_msgs/TimeReference.h>
// Velocity -> TODO
#include <geometry_msgs/TwistStamped.h>
// Heading -> TODO
// TODO

#include <phidgets_api/gps.h>

#include <string>

namespace phidgets
{


class GpsRosI : public virtual Gps
{

    public:

        GpsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);


    private:
        // frame id
        std::string frame_id_;

        // NodeHandlers
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // Publishers
        ros::Publisher  NavSatFix_publisher_;
        ros::Publisher  NavSatStatus_publisher_;
        ros::Publisher  TimeReference_publisher_;

        // Messages
        sensor_msgs::NavSatFix NavSatFix_msg;
        sensor_msgs::NavSatStatus NavSatStatus_msg;
        sensor_msgs::TimeReference TimeReference_msg;

        // Topic names
        std::string NavSatFix_topic_name;
        std::string NavSatStatus_topic_name;
        std::string TimeReference_topic_name;

        // ?
        bool initialized_;

        // Time
        // ?
        ros::Time last_gps_time_;
        // ?
        ros::Time time_zero_;

        // std dev
        double position_stdev_;


        // Function event handlers
    protected:
        virtual void positionHandler(double latitude, double longitude, double altitude);
        virtual void headingHandler(double heading);
        virtual void velocityHandler(double velocity);
        virtual void dateAndTimeHandler(GPSDate date, GPSTime time);
        virtual void nmeaDataHandler(NMEAData NMEAdata);

        virtual void positionFixStatusChangeHandler(int status);


        // Function process data
    protected:
        void processNavSatFixData(double latitude, double longitude, double altitude);
        void processTimeReferenceData(GPSDate date, GPSTime time);
        void processNavSatStatusData(int status);


};


class GpsAsyncRosI : public GpsAsync, public GpsRosI
{

public:

    GpsAsyncRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);



};



class GpsSyncRosI : public GpsSync, public GpsRosI
{

public:

    GpsSyncRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);



};



} //namespace phidgets

#endif // PHIDGETS_GPS_ROS_I_H
