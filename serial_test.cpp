#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>

#include "SonicSole.h"

int main() {
    SonicSole* sole = new SonicSole();
    //int fd = serialOpen("/dev/ttyS0", 115200); 

    int fd = serialOpen("/dev/ttyAMA0", 115200);
    //int fd = serialOpen("/dev/ttyAMA0", 921600);

    if (fd < 0) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    while (true) {
        sole->readIMU();
        // if (serialDataAvail(fd)) {
        //     char c = serialGetchar(fd);
        //     std::cout << c;
        //     std::cout.flush();
        // } else {
        //     usleep(10000); 
        // }
    }

    return 0;
}
//g++ -o serial_test serial_test.cpp -lwiringPi
