#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

//get a frame from the camera and stores it in matrix 'frame'
void get_frame(struct handler_t *h) {

	Mat tmp;

	cap.read(tmp);

	sem_wait(&h->acc_frame);

	tmp.copyTo(h->frame);
	h->newFrame = 1;
	h->newFrame2Det = 1;

	// check if we succeeded
	if (h->frame.empty()) {		
		app_error((char *) "get_frame", (char *) "ERROR! blank frame grabbed");
	}


	sem_post(&h->acc_frame);
}


//store the frame acquired inside the video
void store_frame(struct handler_t *h) {

	Mat local_frame;
	Mat local_det_frame;
	int newFrame;
	int newDetection;

	sem_wait(&h->acc_frame);

	//consume the new frame, if it is present
	newFrame = h->newFrame;
	h->newFrame = 0;
	newDetection = h->newDetection;
	h->newDetection = 0;

	if(newFrame == 1)
		h->frame.copyTo(local_frame);
	if(newDetection == 1)
		h->detect_frame.copyTo(local_det_frame);

	sem_post(&h->acc_frame);
	
	if(newFrame == 1)
		out_capture.write(local_frame);
	else
		printf("frame vecchio, salto %d\n", newFrame);
	if(newDetection == 1)
		threshold_debug.write(local_det_frame);
	else
		printf("frame vecchio debug, salto\n");

}