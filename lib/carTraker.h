#include <stdio.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>

extern "C" {
    #include "ptask.h"
}


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

long int get_time_ms();

void init();

void init_handler(struct handler_t *h);
void get_frame(struct handler_t *h);
void frame_acquisition();
void store_frame(struct handler_t *h);
void store_video();
void detect_track();
void preproc_detect(struct handler_t *h);
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);
void morphOps(Mat &thresh);

void create_tasks();