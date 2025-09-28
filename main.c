// gcc -Wextra -o test main.c -Wall libs/RPI-serial/RPIserial.c libs/NMEA-gps_utilities/NMEA-GPSutils.c -lm

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include "libs/NMEA-gps_utilities/NMEA-GPSutils.h"
#include "libs/RPI-serial/RPIserial.h"

#define GPS_MODULE "/dev/ttyUSB0"

int main(void){
    setupGpsDevice(GPS_MODULE, B230400);
    
    while(1){
        printRmcData(stdout, getNewRmcLine());
        sleep(0.5);
    }

    closeGpsDevice();
    return 0;
}