#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(4, 4));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);

}

bool trackFilteredObject(int &x, int &y, Mat threshold){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}
			}
		} 
		else printf("Too many objects detected, please adjust the filter\n");
	}

	return objectFound;
}

//preprocess of the frame
void preproc_detect(struct handler_t *h, struct detection_handler_t *d) {
	int newFrame2Det;

	Mat local_frame, HSV, color_thresh, gray;

	sem_wait(&h->acc_frame);

	newFrame2Det = h->newFrame2Det;
	h->newFrame2Det = 0;

	if(newFrame2Det == 1)
		h->frame.copyTo(local_frame);

	sem_post(&h->acc_frame);

	if(newFrame2Det == 0) 
		printf("frame is not ready, waiting for a new one\n");
	else
	{
		//convert frame from BGR to HSV colorspace
		cvtColor(local_frame, HSV, COLOR_BGR2HSV);
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), color_thresh);
		
		cvtColor(color_thresh, gray, COLOR_GRAY2BGR);

		sem_wait(&h->acc_frame);

		gray.copyTo(h->detect_frame);
		h->newDetection = 1;

		sem_post(&h->acc_frame);

		sem_wait(&d->det_sem);

		color_thresh.copyTo(d->color_thresh);
		
		if(d->color_ready == 0) {
			//my part is ready, waking up the task that calculates movement
			d->color_ready = 1;
			sem_post(&d->priv_col);
		}

		sem_post(&d->det_sem);
	}
}

//calculate the values of the servo 
bool calc_movement(struct detection_handler_t *d, int& servo_val, int& engine_val) {

	int x = FRAME_WIDTH / 2, y;
	bool objectFound = false;
	Mat local_col_thresh;

	//waits for color threshold to be ready
	sem_wait(&d->priv_col);

	sem_wait(&d->det_sem);

	d->color_ready = 0;
	
	d->color_thresh.copyTo(local_col_thresh);

	sem_post(&d->det_sem);
	
	if(local_col_thresh.empty()) 
			app_error((char *) "calc_movement", (char *) "ERROR! blank threshold retrieved");
	
	morphOps(local_col_thresh);
	objectFound = trackFilteredObject(x, y, local_col_thresh);
	
	if(objectFound) {
		servo_val = (x * SERVO_RANGE) / FRAME_WIDTH;
		servo_val -= (SERVO_RANGE / 2); //move the val in a range of -(range/2) | +(range/2)
		servo_val += SERVO_CENTER;
		engine_val = 255;
		printf("object found in X: %d, moving servo to %d\n", x, servo_val);
	} else 
		printf("object not found\n");
	
	return objectFound;
}