#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

//------------------------------------------------------------------------------
//		This file contains all the functions related to the detection
//		part of the a frame, to find the target object and its position
//------------------------------------------------------------------------------

/**
 * Modifications of the detected frame in order to remove background noise
 * @param	: Mat &thresh;		detected frame to be modified
 */
void morphOps(Mat &thresh) {
	Mat		erodeElement;		//structuring element that erodes the frame
	Mat 	dilateElement;		//structuring element that dilates the frame

	erodeElement = getStructuringElement(MORPH_RECT, Size(4, 4));
	dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	//removes noise and gives a better shape to the frame's element
	erode(thresh, thresh, erodeElement);
	//dilates to make sure object is nicely visible
	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

/**
 * Detects the biggest object in a filtered frame and outputs its position
 * inside the image
 * @param	: int &x;			X value of the biggest object of the image
 * @param	: int &y;			Y value of the biggest object of the image
 * @param	: Mat threshold;	filtered frame
 * @return	: bool;				true if an object is detected, false instead
 */
bool trackFilteredObject(int &x, int &y, Mat threshold) {
	Mat						temp;			//temporary frame to copy the filtered frame
	vector<vector<Point>>	contours; 		//detected contours of the image
	vector<Vec4i>			hierarchy;		//topology of the detected contours
	int						numObjects;		//number of the objects detected
	double					refArea = 0;	//area of the biggest object found
	double					area;			//temporary value for an object area
	int						index;			//loop index
	bool					objectFound;	//true if an object is detected, false instead

	objectFound = false;
	threshold.copyTo(temp);

	//find contours of the image and checks if at least one object is present
	findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	if (hierarchy.size() > 0) {
		numObjects = hierarchy.size();
        //if the number of objects is greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects < MAX_NUM_OBJECTS) {
			for (index = 0; index >= 0; index = hierarchy[index][0]) {
				Moments moment = moments((Mat) contours[index]);
				area = moment.m00;

				//if a bigger object is found we save it
                if (area > MIN_OBJ && area < MAX_OBJ && area > refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
				}
			}
		} 
		else printf("Too many objects detected, please adjust the filter\n");
	}

	return objectFound;
}

/**
 * Creates a black&white frame image filtering the color from the current
 * frame.
 * @param	: struct camera_h *h;		pointer to the camera struct
 * @param	: struct detection_h *d;	pointer to the detection struct
 */
void preproc_detect(struct camera_h *h, struct detection_h *d) {
	int		newFrame2Det;	//tells if the current frame is already filtered
	Mat		local_frame;	//temporary variable to save the current frame
	Mat		HSV;			//temporary variable to save the HSV converted frame
	Mat		color_thresh;	//black&white output frame with the filtered image
	Mat		gray;			//black&white output frame converted for the video streaming

	sem_wait(&h->acc_frame);
	newFrame2Det = h->newFrame2Det;
	h->newFrame2Det = 0;
	if (newFrame2Det == 1) h->frame.copyTo(local_frame);
	sem_post(&h->acc_frame);

	if (newFrame2Det == 0) printf("frame is not ready,waiting for a new one\n");
	else {
		//convert frame from BGR to HSV colorspace
		cvtColor(local_frame, HSV, COLOR_BGR2HSV);
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), 
			Scalar(H_MAX, S_MAX, V_MAX), color_thresh);	//filtering part
		cvtColor(color_thresh, gray, COLOR_GRAY2BGR);	//convertion for the video streaming

		//passing the frame to the video storing task
		sem_wait(&h->acc_frame);
		gray.copyTo(h->detect_frame);
		h->newDetection = 1;
		sem_post(&h->acc_frame);

		sem_wait(&d->det_sem);
		color_thresh.copyTo(d->color_thresh);	
		if (d->color_ready == 0) {
			//filtered frame is ready, waking up the task that calculates movement
			d->color_ready = 1;
			sem_post(&d->priv_col);
		}
		sem_post(&d->det_sem);
	}
}

/**
 * Calculates the actuators values from the position of the detected object
 * inside the frame
 * @param	: struct detection_h *d;	pointer to the detection struct
 * @param	: int& servo_val;			value of the steering part
 * @param	: int& engine_val;			value of the engine part
 * @return	: bool;						true if an object is detected, false instead
 */
bool calc_movement(struct detection_h *d, int& servo_val, int& engine_val) {

	int		x, y;					//X and Y values of the detected object
	bool	objectFound = false;	//true if an object is detected, false instead
	Mat		local_col_thresh;		//black&white frame with the filtered image

	//private sempahore to wait filtered frame to be ready
	sem_wait(&d->priv_col);

	sem_wait(&d->det_sem);

	d->color_ready = 0;
	
	d->color_thresh.copyTo(local_col_thresh);

	sem_post(&d->det_sem);
	
	if (local_col_thresh.empty()) 
			app_error((char *) "calc_movement",
				(char *) "ERROR! threshold frame is blank");
	
	morphOps(local_col_thresh);
	objectFound = trackFilteredObject(x, y, local_col_thresh);
	
	if (objectFound) {
		servo_val = (x * SERVO_RANGE) / FRAME_WIDTH;
		//move the val in a range of -(range/2) | +(range/2)
		servo_val -= (SERVO_RANGE / 2); 
		servo_val += SERVO_CENTER;		//offset to center the value for the servo
		engine_val = 255;
		printf("object found in X: %d, moving servo to %d\n", x, servo_val);
	} else 
		printf("object not found\n");
	
	return objectFound;
}