#ifndef PHIDGETS_API_GPS_H
#define PHIDGETS_API_GPS_H

#include "phidgets_api/phidget.h"

#include <iostream>

#include <thread>         // std::this_thread::sleep_until
#include <chrono>         // std::chrono::system_clock
#include <ctime>          // std::time_t, std::tm, std::localtime, std::mktime

#include <limits> // inf
#include <cmath> // nan




namespace phidgets 
{

// Virtual class for GPS
class Gps : public Phidget
{
        // Constuctor
    public:
        Gps();

        // Gps Handler
    protected:
        CPhidgetGPSHandle gps_handle_;

        // Private Handlers called on events
    protected:
        // Static
        static int PositionChange_HandlerStatic(CPhidgetGPSHandle gps, void *userptr, double latitude, double longitude, double altitude);
        static int PositionFixStatusChange_HandlerStatic(CPhidgetGPSHandle gps, void *userptr, int status);

    protected:
        // Virtual non-static -> Different for sync or async
        virtual int PositionChange_Handler(CPhidgetGPSHandle gps, double latitude, double longitude, double altitude)=0;
        virtual int PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, int status)=0;

        // User must define for the application
    protected:
        // Processing on position change
        virtual void positionHandler(double latitude, double longitude, double altitude)=0;
        virtual void headingHandler(double heading)=0;
        virtual void velocityHandler(double velocity)=0;
        virtual void dateAndTimeHandler(GPSDate date, GPSTime time)=0;
        virtual void nmeaDataHandler(NMEAData NMEAdata)=0;

        // On fix status change
        virtual void positionFixStatusChangeHandler(int status)=0;

    private:
        virtual int registerGpsHandlers();

};

// Asynchronous GPS that only makes actions when receive a new measurement (different than the previous one)
class GpsAsync: public virtual Gps
{
        // Constuctor
    public:
        GpsAsync();

        // Handlers called on events
    protected:
        // Virtual non-static
        int PositionChange_Handler(CPhidgetGPSHandle gps, double latitude, double longitude, double altitude);
        int PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, int status);

};




class GpsSync : public virtual Gps
{
        // Constuctor
    public:
        GpsSync();

        // Handlers called on events
    protected:
        // Virtual non-static
        int PositionChange_Handler(CPhidgetGPSHandle gps, double latitude, double longitude, double altitude);
        int PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, int status);

        // Rate of the measurements
    protected:
        double rate; // In Hz. Maximum = 10Hz
        std::chrono::duration<double> period_time; // Seconds
    public:
        int setRate(double rate);
    protected:
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::duration<double>> time_ref;

        // Values -> Last known values
    public:
        int initPoseValues();
    protected:
        // Pose and speed
        double latitude;
        double longitude;
        double altitude;
        double heading;
        double velocity;
        // Date and Time
    public:
        int initDateAndTimeValues();
    protected:
        GPSDate date;
        GPSTime time;
        // NMEA Data
    public:
        int initNMEAValues();
    protected:
        NMEAData NMEAdata;
        // Status
    public:
        int initStatusValues();
    protected:
        int status;

        // Flags
    protected:
        bool is_signal_acquired;
        bool is_position_fixed;
        bool is_position_acquired;

        // Run method
    public:
        int run();
};


class GpsTest : public virtual Gps
{
    public:
        // Processing on position change -> User must define for the application
        virtual void positionHandler(double latitude, double longitude, double altitude);
        virtual void headingHandler(double heading);
        virtual void velocityHandler(double velocity);
        virtual void dateAndTimeHandler(GPSDate date, GPSTime time);
        virtual void nmeaDataHandler(NMEAData NMEAdata);

        // On fix status change
        virtual void positionFixStatusChangeHandler(int status);

};


class GpsAsyncTest : public GpsAsync, public GpsTest
{

};

class GpsSyncTest : public GpsSync, public GpsTest
{

};


} //namespace phidgets

#endif // PHIDGETS_API_GPS_H
