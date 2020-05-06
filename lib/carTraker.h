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

void init_handlers(struct handler_t *h, struct components_handler_t *c, struct detection_handler_t *d);
void get_frame(struct handler_t *h);
void frame_acquisition();
void store_frame(struct handler_t *h);
void store_video();
void detect_color();
void preproc_detect(struct handler_t *h, detection_handler_t *d);
void detect_circles();
void circles_detection(struct handler_t *h, struct detection_handler_t *d);
void trackFilteredObject(int &x, int &y, Mat threshold);
void morphOps(Mat &thresh);
void sensor_bridge();
void check_move();
void calc_movement(struct detection_handler_t *d, int& servo_val, int& engine_val);
void send_movement(struct components_handler_t *c, int servo_val, int engine_val);
void serial_send(int componentId, int componentValue);
void serial_receive(struct components_handler_t *c);

void create_tasks();