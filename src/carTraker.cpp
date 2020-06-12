#include "../lib/carTraker.h"

using namespace std;
using namespace cv;

//task defines
#define WAIT_BEFORE_CLOSE 500
#define NUM_TASKS 5
#define STORE_PER 120 
#define STORE_PRIO 70	//low priority
#define SENSOR_PER 10
#define SENSOR_PRIO 99	//max priority
#define FRAME_PER 80
#define FRAME_PRIO 99	//max priority
#define DETECT_PER 80
#define DETECT_PRIO 90	//high priority
#define MOVE_PER 80
#define MOVE_PRIO 80	//mid priority

//serial
int fd_serial;
#define SER_MESS_LENGTH 10
#define BAUD_RATE 9600

//physical components values
#define NOT_CHANGE -1
#define INPUT_SENSOR_LENGTH 10
#define MIN_OBSTACLE_DIST 25
#define ENGINE_ID 1
#define SERVO_ID 2
#define SERVO_RANGE 36 //80 is the maximum for the chassis limitations
#define SERVO_CENTER 90

//range for color detection
int H_MIN = 120;
int H_MAX = 169;
int S_MIN = 127;
int S_MAX = 256;
int V_MIN = 32;
int V_MAX = 102;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 1*1; //depends on the background noise
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

//camera handlers
VideoCapture cap;

//video streaming
VideoWriter out_capture;
VideoWriter threshold_debug;

int ret[NUM_TASKS];

struct handler_t {
    //to access the frame resource in mutex
    sem_t acc_frame;

    //frame taken from the camera
    Mat frame;

	Mat detect_frame;

	int newFrame;
	int newFrame2Det;
	int newDetection;
} handler;

//it handles the synchronization between the detection tasks
struct detection_handler_t {
    //to access the results of each detection part
    sem_t det_sem;
	sem_t priv_col;

	//syncronization variables
	int color_ready;

    //frame taken from the camera
    Mat color_thresh;

} detection_handler;

//values of the various components: sensors, motors, etc.
struct components_handler_t {
    //to access the values
    sem_t acc_serial_out;

    int sens_dist_val;
	int last_obstacle_detected;

} components_handler;

/**
 * 
 * 
 */
void init() {
	Mat tmp;

    ptask_init(SCHED_FIFO, GLOBAL, PRIO_CEILING);
	
	init_handlers(&handler, &components_handler, &detection_handler);

	//--- open the camera
    cap.open(0);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        exit(-1);
    }
	cap.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	cap.set(CAP_PROP_BUFFERSIZE, 1); // internal buffer will now store only 1 frame

	out_capture.open("4video_car.avi", VideoWriter::fourcc('M','J','P','G'), 1000 / STORE_PER, Size(FRAME_WIDTH, FRAME_HEIGHT));
	threshold_debug.open("4debug.avi", VideoWriter::fourcc('M','J','P','G'), 1000 / STORE_PER, Size(FRAME_WIDTH, FRAME_HEIGHT));

	//opens and initializes serial connection
	if ((fd_serial = serialOpen ("/dev/ttyUSB0", BAUD_RATE)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		exit(-1);
	}

	if (wiringPiSetup () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
		exit(-1);
	}

	//sync of frame capture
	cap.read(tmp);

	//let's flush before start
	serialFlush(fd_serial) ;
	fflush(stdin);
	fflush(stdout);
}

/* inizializzazione della struttura condivisa */
void init_handlers(struct handler_t *h, struct components_handler_t *c, struct detection_handler_t *d) {

    /* mutua esclusione */
    sem_init(&h->acc_frame,0,1);
	sem_init(&c->acc_serial_out,0,1);
	sem_init(&d->det_sem,0,1);
	sem_init(&d->priv_col,0,0);

	h->newFrame = 0;
	h->newFrame2Det = 0;
	h->newDetection = 0;

	d->color_ready = 0;

	c->sens_dist_val = 0;
	c->last_obstacle_detected = 1;
}

void app_error(char *f, char *msg) {

	struct components_handler_t *c = &components_handler;

	//put the actuators to default values
	serial_send(ENGINE_ID, 0);
	serial_send(SERVO_ID, SERVO_CENTER);

	ptask_syserror(f, msg);
}

void close_app() {
	int i;
	tspec wc_et[NUM_TASKS];
	struct handler_t *h = &handler;
	struct components_handler_t *c = &components_handler;
	struct detection_handler_t *d = &detection_handler;

	//take all the semaphore to block the tasks
	
	sem_wait(&c->acc_serial_out);
	//put the actuators to default values
	serial_send(ENGINE_ID, 0);
	serial_send(SERVO_ID, SERVO_CENTER);

	sem_wait(&h->acc_frame);
	sem_wait(&d->det_sem);

	// //wait some time to block all the tasks
	usleep(WAIT_BEFORE_CLOSE);

	//close the videos
	out_capture.release();
	threshold_debug.release();
	
	printf("Time measurements approximated\n");
	for(i = 0; i < NUM_TASKS; i++) {
		wc_et[i] = ptask_get_wcet(ret[i]);
		printf("\nTask id %d:\nWorst case execution time: %ld\n", i, tspec_to(&wc_et[i], MILLI));
	}
	fflush(stdout);
}

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

//periodic task to take frames from the camera 
void frame_acquisition() {
	
	while(1) {

		get_frame(&handler);

		ptask_wait_for_period();
	}
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

//take all the frames and store them in a video
void store_video() {

	while(1) {
		
		store_frame(&handler);

		ptask_wait_for_period();
	}
}

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

	//DEBUG gray
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

void detect_color() {

	while(1) {

		preproc_detect(&handler, &detection_handler);

		ptask_wait_for_period();
	}
}

void serial_receive(struct components_handler_t *c) {
	char input_sensor [INPUT_SENSOR_LENGTH];
	char next_char = 0;
	char *eptr;
	long int sensor_value = 0;
	int i = 0;
	int current_obst_det = 0;

	input_sensor[INPUT_SENSOR_LENGTH - 1] = 0;

	sem_wait(&c->acc_serial_out);

	//clean up the serial from bad data
	while(next_char == 0 || next_char == '\n') {
		next_char = serialGetchar(fd_serial);
	}

	//populate the input array with the values from our serial
	while((next_char >= '0' && next_char <= '9') && i < INPUT_SENSOR_LENGTH - 1) {
		input_sensor[i] = next_char;

		next_char = serialGetchar(fd_serial);
		i++;

		if(i >= INPUT_SENSOR_LENGTH - 1) {
			printf("Errore nell'acquisizione del sensore, il messaggio supera il buffer\n");
		}
	}
	
	input_sensor[i] = 0; //close the string
	
	sensor_value = strtol(input_sensor, &eptr, 10);

	//insert and elaborate
	c->sens_dist_val = (int) sensor_value;

	printf("sensor value: %d\n", sensor_value);

	if(sensor_value <= MIN_OBSTACLE_DIST)
		current_obst_det = 1; //true
	else 
		current_obst_det = 0; //false

	//block the car if object is found
	if(current_obst_det == 1 && c->last_obstacle_detected == 0) {
		printf("BLOCKING THE CAR!!!\n");
		serial_send(ENGINE_ID, 0);
		serial_send(SERVO_ID, NOT_CHANGE);
	}

	//update object detected
	c->last_obstacle_detected = current_obst_det;

	sem_post(&c->acc_serial_out);
}

void sensor_bridge() {
	
	while(1) {

		if(serialDataAvail(fd_serial) == -1) {
			app_error((char *) "sensor_bridge", (char *) "ERROR! Can't access to the serial communication");
		} else {
			while (serialDataAvail(fd_serial) >= SER_MESS_LENGTH) {
				serial_receive(&components_handler);
			}
		}

		ptask_wait_for_period();
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

void serial_send(int componentId, int componentValue) {
	char ser_message[SER_MESS_LENGTH];

	//with a negative number we keep the component unchanged
	if(componentValue >= 0) {
		sprintf(ser_message, "<%d:%d>", componentId, componentValue);
		serialPuts(fd_serial, ser_message);
	}
}

void send_movement(struct components_handler_t *c, int servo_val, int engine_val) {

	sem_wait(&c->acc_serial_out);

	//update the movement path (only if an object is not detected)
	if(c->last_obstacle_detected == 0) {
		serial_send(ENGINE_ID, engine_val);
		
		if(servo_val >= SERVO_CENTER - (SERVO_RANGE / 2) && servo_val <= SERVO_CENTER + (SERVO_RANGE / 2)) {
			printf("moving servo to: %d\n", servo_val);
			serial_send(SERVO_ID, servo_val);	
		}
		else
			printf("Warning: servo value not valid %d\n", servo_val);

	} else
		printf("Can't move the car, it is blocked!\n");

	sem_post(&c->acc_serial_out);
}

//periodic task - moves the car using the parameters calculated by the other tasks
void check_move() {

	long int current, start;
	bool objectFound;
	int servo_val = 0, engine_val = 0;

	while(1) {

		objectFound = calc_movement(&detection_handler, servo_val, engine_val);

		if(objectFound) {
			send_movement(&components_handler, servo_val, engine_val);
		} else {
			send_movement(&components_handler, SERVO_CENTER, 0);
		}

		ptask_wait_for_period();
	}
}

void create_tasks() {
    tpars params[NUM_TASKS];
	int i;

    init();

	for(i = 0; i < NUM_TASKS; i++) {
		params[i] = TASK_SPEC_DFL;
		params[i].measure_flag = 1;
		params[i].act_flag = DEFERRED;
	}

	//store video
	params[0].period = tspec_from(FRAME_PER, MILLI);
	params[0].priority = FRAME_PRIO;

	//sensor bridge
	params[1].period = tspec_from(SENSOR_PER, MILLI);
	params[1].priority = SENSOR_PRIO;

	//frame acquisition
	params[2].period = tspec_from(STORE_PER, MILLI);
	params[2].priority = STORE_PRIO;

	//object detection
	params[3].period = tspec_from(DETECT_PER, MILLI);
	params[3].priority = DETECT_PRIO;

	//check & move
	params[4].period = tspec_from(MOVE_PER, MILLI);
	params[4].priority = MOVE_PRIO;

	ret[0] = ptask_create_param(frame_acquisition, &params[0]);
	if(ret[0] == -1)
		app_error((char *) "create_tasks", (char *) "Error during the creation of the tasks");

	ret[1] = ptask_create_param(sensor_bridge, &params[1]);
	if(ret[1] == -1)
		app_error((char *) "create_tasks", (char *) "Error during the creation of the tasks");

	ret[2] = ptask_create_param(store_video, &params[2]);
	if(ret[2] == -1)
		app_error((char *) "create_tasks", (char *) "Error during the creation of the tasks");

	ret[3] = ptask_create_param(detect_color, &params[3]);
	if(ret[3] == -1)
		app_error((char *) "create_tasks", (char *) "Error during the creation of the tasks");

	ret[4] = ptask_create_param(check_move, &params[4]);
	if(ret[4] == -1)
		app_error((char *) "create_tasks", (char *) "Error during the creation of the tasks");

	//activate the tasks
	for(i = 0; i < NUM_TASKS; i++) {
		ptask_activate(ret[i]);
	}
}