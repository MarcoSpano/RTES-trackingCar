#ifndef CARTRACKER_H
#define CARTRACKER_H

//------------------------------------------------------------------------------
//		This header file contains all the variables and defines used by
//		the application and the main functions application, like the
//		inits and the tasks creation.
//------------------------------------------------------------------------------

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

//application library
#include "../lib/camera.h"
#include "../lib/control_system.h"
#include "../lib/detection.h"

using namespace cv;
using namespace std;

//------------------------------------------------------------------------------
//						    TASK RELATED CONSTANTS
//------------------------------------------------------------------------------
#define WAIT_BEFORE_CLOSE 500
#define NUM_TASKS 5
#define STORE_PER 120 
#define STORE_PRIO 70           //low priority
#define SENSOR_PER 10
#define SENSOR_PRIO 95          //max priority
#define FRAME_PER 80
#define FRAME_PRIO 95           //max priority
#define DETECT_PER 80
#define DETECT_PRIO 90          //high priority
#define MOVE_PER 80
#define MOVE_PRIO 80            //mid priority

//------------------------------------------------------------------------------
//				        SERIAL RELATED CONSTANTS
//------------------------------------------------------------------------------
#define SER_MESS_LENGTH 10
#define BAUD_RATE 9600

//------------------------------------------------------------------------------
//				CONTROL SYSTEM RELATED CONSTANTS
//------------------------------------------------------------------------------
#define NOT_CHANGE -1
#define INPUT_SENSOR_LEN 10
#define MIN_OBSTACLE_DIST 25
#define ENGINE_ID 1
#define SERVO_ID 2
#define SERVO_RANGE 36          //80 is the maximum for the chassis limitations
#define SERVO_CENTER 90

//------------------------------------------------------------------------------
//					COLOR DETECTION CONSTANTS
//------------------------------------------------------------------------------
#define H_MIN 120
#define H_MAX 176
#define S_MIN 97
#define S_MAX 256
#define V_MIN 32
#define V_MAX 159

//------------------------------------------------------------------------------
//						CAMERA RELATED CONSTANTS
//------------------------------------------------------------------------------
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VIDEO_FRAMERATE 1000 / STORE_PER

//------------------------------------------------------------------------------
//						DETECTION RELATED CONSTANTS
//------------------------------------------------------------------------------
#define MAX_NUM_OBJECTS 50
#define MIN_OBJ 2 * 2                          //depends on the background noise
#define MAX_OBJ FRAME_HEIGHT * FRAME_WIDTH / 1.5

//------------------------------------------------------------------------------
//						VARIABLES
//------------------------------------------------------------------------------
extern int              fd_serial;          //file descriptor for the serial device
extern int              ret[NUM_TASKS];     //ptask ID for each task
extern VideoCapture     cap;                //camera instantiation
extern VideoWriter      video_camera;       //video streaming of the onboard camera
extern VideoWriter      video_processed;    //video streaming of the processed frames

//------------------------------------------------------------------------------
//						STRUCTURES
//------------------------------------------------------------------------------
extern struct camera_h {
    sem_t   acc_frame;      //to lock the frame resources
    
    Mat     frame;          //frame taken from the camera
	Mat     detect_frame;   //black&white color filtered frame
	int     newFrame;       //tells if the current frame is already stored
	int     newFrame2Det;   //tells if the current frame is already filtered
	int     newDetection;   //tells if the detected frame is already stored
} camera;

extern struct detection_h {
    sem_t   det_sem;        //to lock the image result of the detection part
	sem_t   priv_col;       //private semaphore that tells if the detected frame is ready
	
	int     color_ready;    //tells if the detected frame was already consumed
    Mat     color_thresh;   //black&white frame with the filtered image

} detection;

extern struct control_h {
    sem_t   acc_serial_out;             //to lock the serial and sensor data

    int     sens_dist_val;              //latest value of the ultrasonic sensor
	int     last_obstacle_detected;     //tells if an obstacle is found

} control;

//------------------------------------------------------------------------------
//						FUNCTION DECLARATION
//------------------------------------------------------------------------------
extern void init();
extern void close_app();
extern void app_error(char *f, char *msg);

void init_strc(struct camera_h *h, struct control_h *c, struct detection_h *d);
void init_resources();

extern void frame_acquisition();
extern void store_video();
extern void detect_color();
extern void sensor_bridge();
extern void check_move();

extern void create_tasks();

#endif