//default libraries
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

//ptask library
extern "C" {
    #include "ptask.h"
}

//opencv libraries 
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

long int get_time_ms();

void init();

void init_handlers(struct handler_t *h, struct comp_val_t *c);
void get_frame(struct handler_t *h);
void frame_acquisition();
void store_frame(struct handler_t *h);
void store_video();
void detect_track();
void preproc_detect(struct handler_t *h, struct comp_val_t *c);
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);
void morphOps(Mat &thresh);
void check_move();
void calc_movement(struct comp_val_t *c, int& servo_val, int& engine_val);
void send_movement(int componentId, int componentValue);
void receive_sensor_data(int& componentId, int& componentValue);

void create_tasks();