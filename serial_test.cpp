#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>

#include "SonicSole.h"

int main() {
    SonicSole* sole = new SonicSole();
    //int fd = serialOpen("/dev/ttyS0", 115200); 

    int fd = serialOpen("/dev/ttyS0", 115200);
    //int fd = serialOpen("/dev/ttyAMA0", 921600);

    if (fd < 0) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    int cycle = 0;

    while (true) {
        struct timeval tv_start, tv_end;
	    gettimeofday(&tv_start,NULL);
        sole->readIMU();
        gettimeofday(&tv_end,NULL);
        long elapsed_microS = (tv_end.tv_sec - tv_start.tv_sec) * 1000000L + (tv_end.tv_usec - tv_start.tv_usec);
      

        double elapsed_seconds = elapsed_microS / 1e6;
        std::cout << "\ntime: " << elapsed_seconds << " s" << std::endl;
     // std::cout << "\ntime: " << elapsed_microS << " us" << std::endl;
     // double time = tv_end.tv_sec - tv_start.tv_sec;
     // cout << "\ntime: " << time << endl;
        cout << "Cycle: " << cycle << endl;
        cycle++;
        cout << "IMU Data (g) (ax, ay, az): " << sole->ax << ", " << sole->ay << ", " << sole->az << endl;

    }

    return 0;
}

