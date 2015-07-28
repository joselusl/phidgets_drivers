#include "phidgets_api/gps.h"

#include <iostream>
#include <string>

int main(void)
{

    int serial_number=-1;

    phidgets::GpsSyncTest MyGps;

    MyGps.initDevice(serial_number);

    MyGps.run();

    return 1;
}
