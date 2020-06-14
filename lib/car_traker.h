#ifndef CARTRACKER_H
#define CARTRACKER_H

#include <stdio.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <string.h>
#include <errno.h>

//serial libraries
#include <wiringPi.h>
#include <wiringSerial.h>

//opencv libraries 
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//ptask library
extern "C" {
    #include "ptask.h"
    #include "tstat.h"
}

#include "../lib/camera.h"
#include "../lib/control_system.h"
#include "../lib/detection.h"

using namespace cv;
using namespace std;

//task defines
#define WAIT_BEFORE_CLOSE 500
#define NUM_TASKS 5
#define STORE_PER 120 
#define STORE_PRIO 70	//low priority
#define SENSOR_PER 10
#define SENSOR_PRIO 99	//max priority
#define FRAME_PER 80
#define FRAME_PRIO 99	//max priority
#define DETECT_PER 80
#define DETECT_PRIO 90	//high priority
#define MOVE_PER 80
#define MOVE_PRIO 80	//mid priority

//serial
#define SER_MESS_LENGTH 10
#define BAUD_RATE 9600

//physical components values
#define NOT_CHANGE -1
#define INPUT_SENSOR_LENGTH 10
#define MIN_OBSTACLE_DIST 25
#define ENGINE_ID 1
#define SERVO_ID 2
#define SERVO_RANGE 36 //80 is the maximum for the chassis limitations
#define SERVO_CENTER 90

//range for color detection
#define H_MIN 120
#define H_MAX 155
#define S_MIN 97
#define S_MAX 256
#define V_MIN 32
#define V_MAX 159

//default capture width and height
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
//max number of objects to be detected in frame
#define MAX_NUM_OBJECTS 50
//minimum and maximum object area
#define MIN_OBJECT_AREA 1 * 1 //depends on the background noise
#define MAX_OBJECT_AREA FRAME_HEIGHT * FRAME_WIDTH / 1.5

extern int fd_serial;

//camera handlers
extern VideoCapture cap;

//video streaming
extern VideoWriter out_capture;
extern VideoWriter threshold_debug;

extern int ret[NUM_TASKS];

extern struct handler_t {
    //to access the frame resource in mutex
    sem_t acc_frame;

    //frame taken from the camera
    Mat frame;

	Mat detect_frame;

	int newFrame;
	int newFrame2Det;
	int newDetection;
} handler;

//it handles the synchronization between the detection tasks
extern struct detection_handler_t {
    //to access the results of each detection part
    sem_t det_sem;
	sem_t priv_col;

	//syncronization variables
	int color_ready;

    //frame taken from the camera
    Mat color_thresh;

} detection_handler;

//values of the various components: sensors, motors, etc.
extern struct components_handler_t {
    //to access the values
    sem_t acc_serial_out;

    int sens_dist_val;
	int last_obstacle_detected;

} components_handler;

extern void init();
extern void init_handlers(struct handler_t *h, struct components_handler_t *c, struct detection_handler_t *d);
extern void close_app();
extern void app_error(char *f, char *msg);


extern void frame_acquisition();

extern void store_video();
extern void detect_color();


extern void sensor_bridge();
extern void check_move();



extern void create_tasks();

#endif