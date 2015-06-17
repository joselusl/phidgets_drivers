#ifndef PHIDGETS_API_GPS_H
#define PHIDGETS_API_GPS_H

#include "phidgets_api/phidget.h"

#include <iostream>

#define PHIDGETS_GPS_ON_POSITION_CHANGE_HANDLER_GROUPED

namespace phidgets 
{

class Gps: public Phidget
{
    public:
        // Constuctor
        Gps();

        // Gps Handler
    protected:
        CPhidgetGPSHandle gps_handle_;

    protected:
        // On position change
#ifdef PHIDGETS_GPS_ON_POSITION_CHANGE_HANDLER_GROUPED
        virtual void positionChangeHandler(double latitude, double longitude, double altitude,
                                           double heading,
                                           double velocity,
                                           GPSDate date, GPSTime time,
                                           NMEAData NMEAdata);
#else
        virtual void positionHandler(double latitude, double longitude, double altitude);
        virtual void headingHandler(double heading);
        virtual void velocityHandler(double velocity);
        virtual void dateAndTimeHandler(GPSDate date, GPSTime time);
        virtual void nmeaDataHandler(NMEAData NMEAdata);
#endif

        // On fix status change
        virtual void positionFixStatusChangeHandler(int status);


    private:
        // Handlers
        static int PositionChange_Handler(CPhidgetGPSHandle gps, void *userptr, double latitude, double longitude, double altitude);
        static int PositionFixStatusChange_Handler(CPhidgetGPSHandle gps, void *userptr, int status);
};

} //namespace phidgets

#endif // PHIDGETS_API_GPS_H
