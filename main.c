#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include "libs/NMEA-GPSutils/NMEA-GPSutils.h"
#include "libs/RPI-serial/RPIserial.h"

#define GPS_MODULE ""

int main(void){
    setupGpsDevice(GPS_MODULE, B230400);
    
    while(1){
        printRmcData(stdout, getNewRmcLine());
    }

    closeGpsDevice();
    return 0;
}