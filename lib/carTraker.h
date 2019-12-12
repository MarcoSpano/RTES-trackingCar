#include <stdio.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdlib.h>

extern "C" {
    #include "ptask.h"
}


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void init();

void init_handler(struct handler_t *h);
void get_frame(struct handler_t *h);
void frame_acquisition();
void store_frame(struct handler_t *h);
void store_video();

void create_tasks();