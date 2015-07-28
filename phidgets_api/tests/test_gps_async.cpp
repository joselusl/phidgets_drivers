#include "phidgets_api/gps.h"

#include <iostream>
#include <string>

int main(void)
{

    int serial_number=-1;

    phidgets::GpsAsyncTest MyGps;

    MyGps.initDevice(serial_number);

    std::cout<<"Press enter to finish"<<std::endl;
    std::string key;
    getline(std::cin, key);


    return 1;
}
