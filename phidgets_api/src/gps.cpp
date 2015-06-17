#include "phidgets_api/gps.h"

namespace phidgets 
{

Gps::Gps():
    Phidget(),
    gps_handle_(0)
{
    // create the handle
    CPhidgetGPS_create(&gps_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)gps_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register gps data callback
    CPhidgetGPS_set_OnPositionChange_Handler(gps_handle_, PositionChange_Handler, this);
    CPhidgetGPS_set_OnPositionFixStatusChange_Handler(gps_handle_, PositionFixStatusChange_Handler, this);

}

int Gps::PositionChange_Handler(CPhidgetGPSHandle gps, void *userptr, double latitude, double longitude, double altitude)
{

    double heading, velocity;
    GPSDate date;
    GPSTime time;
    NMEAData NMEAdata;


    // Heading and Velocity
    // TODO check
    CPhidgetGPS_getHeading(gps, &heading);
    CPhidgetGPS_getVelocity(gps, &velocity);


    // Date and time
    // TODO check
    CPhidgetGPS_getDate(gps, &date);
    CPhidgetGPS_getTime(gps, &time);

    // NMEA data
    // TODO check
    CPhidgetGPS_getNMEAData(gps, &NMEAdata);


    // Function to use the data
#ifdef PHIDGETS_GPS_ON_POSITION_CHANGE_HANDLER_GROUPED
    ((Gps*)userptr)->positionChangeHandler(latitude, longitude, altitude, heading, velocity, date, time, NMEAdata);
#else
    ((Gps*)userptr)->positionHandler(latitude, longitude, altitude);
    ((Gps*)userptr)->headingHandler(heading);
    ((Gps*)userptr)->velocityHandler(velocity);
    ((Gps*)userptr)->dateAndTimeHandler(date, time);
    ((Gps*)userptr)->nmeaDataHandler(NMEAdata);
#endif


    return 0;
}

int Gps::PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, void *userptr, int status)
{
    ((Gps*)userptr)->positionFixStatusChangeHandler(status);
    return 0;
}


#ifdef PHIDGETS_GPS_ON_POSITION_CHANGE_HANDLER_GROUPED
void Gps::positionChangeHandler(double latitude, double longitude, double altitude,
                                           double heading,
                                           double velocity,
                                           GPSDate date, GPSTime time,
                                           NMEAData NMEAdata)
{
    std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Latitude="<<latitude<<" ";
    std::cout<<" Longitude="<<longitude<<" ";
    std::cout<<" Altitude="<<altitude<<std::endl;
    std::cout<<" Heading="<<heading<<" ";
    std::cout<<" Velocity="<<velocity<<std::endl;
    printf(" Date: %02d/%02d/%02d Time %02d:%02d:%02d.%03d\n", date.tm_mday, date.tm_mon, date.tm_year, time.tm_hour, time.tm_min, time.tm_sec, time.tm_ms);
    std::cout<<" NMEAData message needs to be processed"<<std::endl;
}

#else
void Gps::positionHandler(double latitude, double longitude, double altitude)
{
    std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Latitude="<<latitude<<" ";
    std::cout<<" Longitude="<<longitude<<" ";
    std::cout<<" Altitude="<<altitude<<std::endl;
}

void Gps::headingHandler(double heading)
{
    std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Heading="<<heading<<" ";
}

void Gps::velocityHandler(double velocity)
{
    std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Velocity="<<velocity<<std::endl;
}

void Gps::dateAndTimeHandler(GPSDate date, GPSTime time)
{
    std::cout<<"Position change event:"<<std::endl;
    printf(" Date: %02d/%02d/%02d Time %02d:%02d:%02d.%03d\n", date.tm_mday, date.tm_mon, date.tm_year, time.tm_hour, time.tm_min, time.tm_sec, time.tm_ms);
}

void Gps::nmeaDataHandler(NMEAData NMEAdata)
{
    std::cout<<"Position change event:"<<std::endl;
    printf("Empty data nmeaDataHandler");
}
#endif

void Gps::positionFixStatusChangeHandler(int status)
{
    printf("Fix change event: %d\n", status);
}

} //namespace phidgets

