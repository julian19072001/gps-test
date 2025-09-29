// gcc -Wextra -o test main.c -Wall libs/RPI-serial/RPIserial.c libs/NMEA-gps_utilities/NMEA-GPSutils.c -lm

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <vector>

extern "C" {
	#include "libs/NMEA-gps_utilities/NMEA-GPSutils.h"
    #include "libs/RPI-serial/RPIserial.h"
}

#define DEG2RAD(x) ((x) * M_PI / 180.0)

#define FORWARD_RANGE 	500.0f		// Measuring range in centimeters
#define SIDE_RANGE		250.0f		// Measuring range in centimeters
#define RESOLUTION 		2.0f		// Amount of centimeter per pixel

#define CANVAS_SIZE_X ((SIDE_RANGE * 2.0f) / RESOLUTION)
#define CANVAS_SIZE_Y (FORWARD_RANGE / RESOLUTION)

#define CAR_X_LOCATION (CANVAS_SIZE_X / 2)
#define CAR_Y_LOCATION CANVAS_SIZE_Y

#define GPS_MODULE "/dev/ttyUSB0"

RMC_t *path;
int numWaypoints;

pthread_mutex_t imageMutex;
pthread_t streamThread;
cv::Mat outputImg = cv::Mat::ones(CANVAS_SIZE_Y, CANVAS_SIZE_X, CV_8UC1) * 255;

cv::VideoWriter video("lidar_output.mp4",
    cv::VideoWriter::fourcc('a','v','c','1'), 10,  
    cv::Size(CANVAS_SIZE_X, CANVAS_SIZE_Y), true); 

bool newImage = false;

// Function to run on exit
void cleanup(void) {
    printf("Program is closing. Cleaning up...\n");
	video.release();

	closeGpsDevice();
}

// Signal handler for Ctrl+C or termination
void handle_signal(int sig) {
    exit(0); // Triggers atexit() functions
}

void* processStream(void* args){
    RMC_t currentLocation;
    int closestWaypoint;
    double course;

	cv::Mat img(CANVAS_SIZE_Y, CANVAS_SIZE_X, CV_8UC1, cv::Scalar(255));

    while (1){
		pthread_testcancel(); 
		currentLocation = getNewRmcLine();

        printRmcData(stdout, currentLocation);
	
		closestWaypoint = getClosestWaypointIdx(&currentLocation, path, numWaypoints);
        course =  getBearing(&path[closestWaypoint], &path[closestWaypoint + 1]);

		for (int i = closestWaypoint; i < numWaypoints; i++){
			float angle = DEG2RAD(getBearing(&currentLocation, &path[i]) - course);
            float distance = getDistance(&currentLocation, &path[i]) * 100.0f;

			if(angle < 0) angle += (M_PI * 2);
			if(angle > (M_PI * 2)) angle -= (M_PI * 2);

			if(angle < (M_PI / 2) || angle > (M_PI * 2)-(M_PI / 2)){
				angle += (M_PI / 2);
				int px = CAR_X_LOCATION - (int)(distance / RESOLUTION * cosf(angle));
				int py = CAR_Y_LOCATION - (int)(distance / RESOLUTION * sinf(angle));

				if (px>=0 && px < CANVAS_SIZE_X && py >= 0 && py < CANVAS_SIZE_Y)
					img.at<uchar>(py, px) = 0;
			} 
		}
        pthread_mutex_lock(&imageMutex);   
        outputImg = img.clone();
        newImage = true;
        pthread_mutex_unlock(&imageMutex); 

        img.setTo(cv::Scalar(255));
	} 
    return nullptr;
}

int main(void){
    // Register the cleanup function to run at exit
    atexit(cleanup);

    // Register signal handlers
    signal(SIGINT, handle_signal);  // Ctrl+C
    signal(SIGTERM, handle_signal); // Kill signal

    setupGpsDevice(GPS_MODULE, B230400);

    path = parseRmcFile("path.nmea", &numWaypoints);

    if (!video.isOpened()) {
        std::cerr << "Kon video niet openen voor schrijven!" << std::endl;
    }

	// Initialize the mutex
    if (pthread_mutex_init(&imageMutex, NULL) != 0) {
        fprintf(stderr, "Mutex init failed\n");
        return 1;
    }
	
	pthread_create(&streamThread, NULL, processStream, NULL);
    
    while (true){
		cv::Mat display;

		pthread_mutex_lock(&imageMutex);   
		if(!newImage){
			pthread_mutex_unlock(&imageMutex); 
			continue;
		}

		display = outputImg.clone();
		newImage = false;
		pthread_mutex_unlock(&imageMutex); 

		if (!display.empty())
            cv::imwrite("lidar_frame.png", display);

			cv::Mat colorFrame;
			cv::cvtColor(display, colorFrame, cv::COLOR_GRAY2BGR);

			video.write(colorFrame);
	}

    pthread_cancel(streamThread);
    pthread_join(streamThread, nullptr);
    pthread_mutex_destroy(&imageMutex);

    return 0;
}