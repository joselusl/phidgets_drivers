#include "phidgets_api/gps.h"

#include <iostream>
#include <string>

int main(void)
{

    int serial_number=-1;

    phidgets::GpsAsyncTest MyGps;


    printf("Opening device");
    MyGps.open(serial_number);

    printf("Waiting for GPS to be attached...");
    int result = MyGps.waitForAttachment(10000);
    if(result)
    {
      const char *err;
        CPhidget_getErrorDescription(result, &err);
        printf("Problem waiting for GPS attachment: %s Make sure the USB cable is connected and you have executed the phidgets_c_api/setup-udev.sh script.", err);
        return 0;
    }

    std::cout<<"Press enter to finish"<<std::endl;
    std::string key;
    getline(std::cin, key);


    return 1;
}
