#ifndef DETECTION_H
#define DETECTION_H

using namespace cv;

//------------------------------------------------------------------------------
//		This file contains all the function declaration 
//		related to detection.cpp
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//						FUNCTION DECLARATION
//------------------------------------------------------------------------------

extern void preproc_detect(struct camera_h *h, 
    struct detection_h *d);
extern bool calc_movement(struct detection_h *d, 
    int& servo_val, int& engine_val);

void morphOps(Mat &thresh);
bool trackFilteredObject(int &x, int &y, Mat threshold);

#endif