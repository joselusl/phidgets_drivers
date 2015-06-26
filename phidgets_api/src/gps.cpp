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

    // Register Gps Handlers
    registerGpsHandlers();

    return;
}

int Gps::registerGpsHandlers()
{
    // register gps data callback
    CPhidgetGPS_set_OnPositionChange_Handler(gps_handle_, PositionChange_HandlerStatic, this);
    CPhidgetGPS_set_OnPositionFixStatusChange_Handler(gps_handle_, PositionFixStatusChange_HandlerStatic, this);
    return 1;
}

int Gps::PositionChange_HandlerStatic(CPhidgetGPSHandle gps, void *userptr, double latitude, double longitude, double altitude)
{
    ((Gps*)userptr)->PositionChange_Handler(gps, latitude, longitude, altitude);
    return 1;
}

int Gps::PositionFixStatusChange_HandlerStatic(CPhidgetGPSHandle gps, void *userptr, int status)
{
    ((Gps*)userptr)->PositionFixStatusChange_Handler(gps, status);
    return 1;
}




GpsAsync::GpsAsync():
    Gps()
{
    return;
}

int  GpsAsync::PositionChange_Handler(CPhidgetGPSHandle gps, double latitude, double longitude, double altitude)
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
    this->positionHandler(latitude, longitude, altitude);
    this->headingHandler(heading);
    this->velocityHandler(velocity);

    this->dateAndTimeHandler(date, time);
    this->nmeaDataHandler(NMEAdata);


    return 1;
}

int  GpsAsync::PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, int status)
{
    // Set data -> Change values
    switch(status)
    {
    case 0:
    default:
        status=-1; // No fix
        break;
    case 1:
        status=0; // Fix
        break;
    }

    this->positionFixStatusChangeHandler(status);

    return 1;
}





GpsSync::GpsSync():
    Gps(),
    rate(10.0),
    period_time(1.0/this->rate),
    is_signal_acquired(false),
    is_position_fixed(false),
    is_position_acquired(false)
{
    // Init values
    initPoseValues();
    initStatusValues();
    initNMEAValues();
    initDateAndTimeValues();

    return;
}

int  GpsSync::PositionChange_Handler(CPhidgetGPSHandle gps, double latitude, double longitude, double altitude)
{
    // Set time for reference
    if(!this->is_signal_acquired)
    {
        this->time_ref = std::chrono::high_resolution_clock::now();
        this->is_signal_acquired=true;
    }

    // Set data -> Update values
    this->latitude=latitude;
    this->longitude=longitude;
    this->altitude=altitude;

    // Heading and Velocity
    // TODO check! The first time it gives a bad value
    if(is_position_acquired)
    {
        CPhidgetGPS_getHeading(gps, &heading);
        CPhidgetGPS_getVelocity(gps, &velocity);
    }

    // Check
    if(!std::isnan(this->latitude) && !std::isnan(this->longitude) && !std::isnan(this->altitude))
    {
        is_position_acquired=true;
    }

    return 1;
}

int GpsSync::PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, int status)
{
    // Set time for reference
    if(!this->is_signal_acquired)
    {
        this->time_ref = std::chrono::high_resolution_clock::now();
        this->is_signal_acquired=true;
    }

    // Set data -> Change values
    switch(status)
    {
    case 0:
    default:
        this->status=-1; // No fix
        is_position_fixed=false;
        break;
    case 1:
        this->status=0; // Fix
        is_position_fixed=true;
        break;
    }


    return 1;
}



int GpsSync::setRate(double rate)
{
    this->rate=rate;

    //period_time=std::chrono::duration_cast<std::chrono::duration<double>>(1.0/this->rate);
    this->period_time=std::chrono::duration<double>(1.0/this->rate);

    return 1;
}

int GpsSync::initPoseValues()
{
    // Pose and speed
    latitude=std::nan("");
    longitude=std::nan("");
    altitude=std::nan("");
    heading=std::nan("");
    velocity=std::nan("");

    return 1;
}

int GpsSync::initDateAndTimeValues()
{
    // TODO
    // Date and time
//    GPSDate date;
//    GPSTime time;

    return 1;
}

int GpsSync::initNMEAValues()
{
    // TODO
    // NMEA Data
//    NMEAData NMEAdata;

    return 1;
}

int GpsSync::initStatusValues()
{
    // Status
    status=-1;

    return 1;
}

int GpsSync::run()
{
    // Run all the time
    while(1)
    {
        if(!this->is_signal_acquired)
        {
            // Reset status
            initStatusValues();

            // Do nothing - Sleep
            std::this_thread::sleep_for(this->period_time);
        }
        else
        {
            // Work

            // Date and time
            // TODO check
            CPhidgetGPS_getDate(this->gps_handle_, &date);
            CPhidgetGPS_getTime(this->gps_handle_, &time);

            // NMEA data
            // TODO check
            CPhidgetGPS_getNMEAData(this->gps_handle_, &NMEAdata);

            // Checks
            if(!is_position_fixed)
            {
                is_position_acquired=false;
                initPoseValues();
            }

            // Fix Status
            this->positionFixStatusChangeHandler(status);

            // Position change
            this->positionHandler(latitude, longitude, altitude);
            this->headingHandler(heading);
            this->velocityHandler(velocity);
            this->dateAndTimeHandler(date, time);
            this->nmeaDataHandler(NMEAdata);


            // Sleep
            this->time_ref = this->time_ref + period_time;
            std::this_thread::sleep_until(this->time_ref);
        }
    }

    return 1;
}




void GpsTest::positionHandler(double latitude, double longitude, double altitude)
{
    std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Latitude="<<latitude<<" ";
    std::cout<<" Longitude="<<longitude<<" ";
    std::cout<<" Altitude="<<altitude<<std::endl;
}

void GpsTest::headingHandler(double heading)
{
    //std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Heading="<<heading<<std::endl;
}

void GpsTest::velocityHandler(double velocity)
{
    //std::cout<<"Position change event:"<<std::endl;
    std::cout<<" Velocity="<<velocity<<std::endl;
}

void GpsTest::dateAndTimeHandler(GPSDate date, GPSTime time)
{
    //std::cout<<"Position change event:"<<std::endl;
    printf(" Date: %02d/%02d/%02d Time %02d:%02d:%02d.%03d\n", date.tm_mday, date.tm_mon, date.tm_year, time.tm_hour, time.tm_min, time.tm_sec, time.tm_ms);
}

void GpsTest::nmeaDataHandler(NMEAData NMEAdata)
{
    //std::cout<<"Position change event:"<<std::endl;
    std::cout<<" NMEAData message needs to be processed"<<std::endl;
}

void GpsTest::positionFixStatusChangeHandler(int status)
{
    printf("Fix change event: %d\n", status);
    return;
}


} //namespace phidgets

