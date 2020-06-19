#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

//------------------------------------------------------------------------------
//		This file contains all the functions related to the frame
//		management, as the frame acquisition from the camera and the
//		frame storing inside videos
//------------------------------------------------------------------------------

/**
 * Gets the current frame from the camera and saves it
 * @param	: struct camera_h *h;	pointer to the camera struct
 */
void get_frame(struct camera_h *h) {
	Mat		tmp;		//temporary variable to save the current frame

	cap.read(tmp);

	sem_wait(&h->acc_frame);

	tmp.copyTo(h->frame);
	h->newFrame = 1;
	h->newFrame2Det = 1;

	if (h->frame.empty()) {		
		app_error((char *) "get_frame", (char *) "ERROR! blank frame grabbed");
	}

	sem_post(&h->acc_frame);
}

/**
 * Stores the frames acquired inside video streamings
 * @param	: struct camera_h *h;	pointer to the camera struct
 */
void store_frame(struct camera_h *h) {
	Mat		local_frame;		//temporary variable to save the current frame
	Mat		local_det_frame;	//temporary variable to save the detected frame
	int		newFrame;			//tells if the current frame is already stored
	int		newDetection;		//tells if the detected frame is already stored

	sem_wait(&h->acc_frame);

	//consumes the new frames, if they are present
	newFrame = h->newFrame;
	h->newFrame = 0;
	newDetection = h->newDetection;
	h->newDetection = 0;

	if (newFrame == 1)
		h->frame.copyTo(local_frame);
	if (newDetection == 1)
		h->detect_frame.copyTo(local_det_frame);

	sem_post(&h->acc_frame);
	
	if (newFrame == 1)
		video_camera.write(local_frame);
	else
		printf("current frame already stored, I move on\n");
	
	if (newDetection == 1) video_processed.write(local_det_frame);
	else
		printf("detected frame already stored, I move on\n");

}