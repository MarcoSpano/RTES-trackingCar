//TO-DO mettere gli include dentro un file .h
//CERCARE TUTTI I TO-DO e DEBUG NEL CODICE E IMPLEMETARLI PRIMA DI ELIMINARE QUESTO MESSAGGIO!!

#include "../lib/carTraker.h"

using namespace std;
using namespace cv;

//ptask defines
#define PER 100
#define DREL 100
#define PRIO 80

//opencv defines
#define CANNY_MIN_THRESH 50
#define CANNY_MAX_THRESH 150
#define MIN_CIRCLE_DIST 15
#define CIRCLE_THRESH 16
#define MIN_RADIUS 4
#define MAX_RADIUS 40

//serial
int fd_serial;
#define SER_MESS_LENGTH 10

//physical components values
#define NOT_CHANGE -1
#define INPUT_SENSOR_LENGTH 10
#define MIN_OBSTACLE_DIST 10
#define ENGINE_ID 1
#define SERVO_ID 2
#define SERVO_RANGE 80
#define SERVO_CENTER 90

//range for color detection
int H_MIN = 120;
int H_MAX = 155;
int S_MIN = 97;
int S_MAX = 256;
int V_MIN = 32;
int V_MAX = 159;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

//camera handlers
VideoCapture cap;
VideoWriter out_capture;

//DEBUG
VideoWriter threshold_debug;
// VideoWriter circles_debug;

long int start;

long int get_time_ms()
{
	// std::time_t t = std::time(0);  // t is an integer type
    // std::cout << t << " seconds since 01-Jan-1970\n";

	struct timeval tp;
	gettimeofday(&tp, NULL);
	long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    // std::cout << ms << " ms\n";
	return ms;
}

struct handler_t {
    //to access the frame resource in mutex
    sem_t acc_frame;

    //frame taken from the camera
    Mat frame;

    // semafori privati
    // sem_t priv_pizzaiolo;
    // sem_t priv_cliente[N];

} handler;

//it handles the synchronization between the detection tasks
struct detection_handler_t {
    //to access the results of each detection part
    sem_t det_sem;
	sem_t priv_col;
	sem_t priv_circ;

	//syncronization variables
	int color_ready;
	int circles_ready;

    //frame taken from the camera
    Mat color_thresh;
	Mat circles_thresh;

} detection_handler;

//values of the various components: sensors, motors, etc.
struct components_handler_t {
    //to access the values
    sem_t acc_serial_out;

    int sens_dist_val;
	int last_object_detected;

} components_handler;

void init() {
    ptask_init(SCHED_RR, GLOBAL, NO_PROTOCOL);
	
	init_handlers(&handler, &components_handler, &detection_handler);

	//--- open the camera
    cap.open(0);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        
    }
	
	cap.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	
	out_capture.open("4video_car.avi", VideoWriter::fourcc('M','J','P','G'), 1000/PER, Size(640,480));
	//DEBUG
	threshold_debug.open("4debug.avi", VideoWriter::fourcc('M','J','P','G'), 1000/PER, Size(640,480));
	// circles_debug.open("4debugcirc.avi", VideoWriter::fourcc('M','J','P','G'), 1000/PER, Size(640,480));

	//opens and initializes serial connection
	if ((fd_serial = serialOpen ("/dev/ttyUSB0", 9600)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		
	}

	if (wiringPiSetup () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
		
	}

	//let's flush before start
	serialFlush(fd_serial) ;
	fflush(stdin);
	fflush(stdout);

	start = get_time_ms();
}

/* inizializzazione della struttura condivisa */
void init_handlers(struct handler_t *h, struct components_handler_t *c, struct detection_handler_t *d) {

    /* mutua esclusione */
    sem_init(&h->acc_frame,0,1);
	sem_init(&c->acc_serial_out,0,1);
	sem_init(&d->det_sem,0,1);
	sem_init(&d->priv_col,0,0);
	sem_init(&d->priv_circ,0,0);

	d->color_ready = 0;
	d->circles_ready = 0;

	c->sens_dist_val = 0;
	c->last_object_detected = 1;
}

// int checkObstacle(struct sensors_val_t *s) {
// 	int obstacle_dist, ret;

// 	sem_wait(&s->acc_sensors);

// 	obstacle_dist = s->sens_dist_val;

// 	sem_post(&s->acc_sensors);

// 	if(obstacle_dist <= MIN_OBSTACLE_DIST) {
// 		ret = 1; //true
// 		printf("Obastacle found %d!\n", obstacle_dist);
// 	} else {
// 		printf("Obastacle not found %d!\n", obstacle_dist);
// 		ret = 0; //false
// 	}
	
// 	return ret;
// }

//get a frame from the camera and stores it in matrix 'frame'
void get_frame(struct handler_t *h) {

	sem_wait(&h->acc_frame);

	cap.read(h->frame);

	//cout << "magari\n";

	// check if we succeeded
	if (h->frame.empty()) {
		cerr << "ERROR! blank frame grabbed\n";
		//break;

		//TO-DO codice per fermare la macchinina (fare una funzione di error)
	}

	sem_post(&h->acc_frame);
}

//periodic task to take frames from the camera 
void frame_acquisition() {

	long int current, start;
	int i = 0;
	
	while(1) {
		//cout << "pigliaml il frame\n";
		start = get_time_ms();

		get_frame(&handler);
		
		current = get_time_ms();

		i++;
		//cout << "frame acquisition: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}

//store the frame acquired inside the video
void store_frame(struct handler_t *h) {

	Mat local_frame;

	sem_wait(&h->acc_frame);

	// local_frame = h->frame.clone();
	h->frame.copyTo(local_frame);

	sem_post(&h->acc_frame);
	
	out_capture.write(h->frame);

}

//take all the frames and store them in a video
void store_video() {

    long int current, start;
	int i = 0;

	while(1) {
		start = get_time_ms();
		
		store_frame(&handler);

		current = get_time_ms();

		i++;
		//cout << "store video: " << current-start << " ms, frame n. " << i << "\n";

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
	dilate(thresh,thresh,dilateElement);

}

bool trackFilteredObject(int &x, int &y, Mat threshold){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE );
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
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
					//cout << "TEMP " << index << ":\n	X: " << x << "\n	Y: " << y << endl; //TRACK DEBUG
				}

				
			}

		} else ;//cout << "TOO MUCH NOISE! ADJUST FILTER";
	}

	return objectFound;
}



//preprocess of the frame
void preproc_detect(struct handler_t *h, struct detection_handler_t *d) {
	//int x = 0, y = 0;

	//DEBUG gray
	Mat local_frame, HSV, color_thresh, gray;

	sem_wait(&h->acc_frame);

	// local_frame = h->frame.clone();
	h->frame.copyTo(local_frame);

	sem_post(&h->acc_frame);

	//DEBUG
	if(local_frame.empty()) 
			printf("\n\n\n\nlocal frame COLOR EMPTY");

	fflush(stdout);

	//convert frame from BGR to HSV colorspace
	cvtColor(local_frame, HSV, COLOR_BGR2HSV);
	inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), color_thresh);
	
	//morphOps(threshold);
	
	// trackFilteredObject(x, y, threshold, local_frame);

	//DEBUG
		cvtColor(color_thresh, gray, COLOR_GRAY2BGR);
		threshold_debug.write(gray);

	//imwrite("test.jpg", local_frame);

	//cout << "FINAL:\nX: " << x << "\nY: " << y << endl;
	
	if(color_thresh.empty()) 
			printf("\n\n\n\nsONO IO CHE GLIELO DO VUOTO\n");

	fflush(stdout);
	

	sem_wait(&d->det_sem);

	color_thresh.copyTo(d->color_thresh);

	if(d->color_thresh.empty()) 
		printf("\n\n\n\nCopia sbagliata?\n");

	fflush(stdout);

	
	if(d->color_ready == 0) {
		//my part is ready, waking up the task that calculates movement
		d->color_ready = 1;
		sem_post(&d->priv_col);
	}
		

	sem_post(&d->det_sem);
	
	//cvtColor(threshold, gray, COLOR_GRAY2BGR);

}

//
void detect_color() {

	long int current, start;
	int i = 0;
	
	while(1) {
		start = get_time_ms();
		//cout << "pigliaml il frame\n";

		preproc_detect(&handler, &detection_handler);
		
		current = get_time_ms();

		i++;
		//cout << "detect color: " << current-start << " ms, frame n. " << i << "\n";
		fflush(stdout);

		ptask_wait_for_period();
	}
}

//preprocess of the frame
// void circles_detection(struct handler_t *h, struct detection_handler_t *d) {
// 	//int x = 0, y = 0;

// 	Mat local_frame, gray, edges, debugss;
// 	Mat circles_thresh(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1, Scalar(0));
// 	vector<Vec3f> circles_detected;

// 	sem_wait(&h->acc_frame);

// 	// local_frame = h->frame.clone();
// 	h->frame.copyTo(local_frame);
	

// 	sem_post(&h->acc_frame);

// 	//DEBUG
// 	if(local_frame.empty()) 
// 			printf("\n\n\n\nlocal frame circles EMPTY");
// 	fflush(stdout);
// /*
// 	cvtColor(local_frame, gray, COLOR_BGR2GRAY);

// 	Canny(gray, edges, CANNY_MIN_THRESH, CANNY_MAX_THRESH, 3);
// 	HoughCircles(edges, circles_detected, HOUGH_GRADIENT, 1, MIN_CIRCLE_DIST, CANNY_MAX_THRESH, CIRCLE_THRESH, MIN_RADIUS, MAX_RADIUS);

// 	//drawing the detected circles on a blank image, to create the 'threshold'
// 	for (size_t i = 0; i < circles_detected.size(); i++)
// 	{
// 		Point tmp_center(cvRound(circles_detected[i][0]), cvRound(circles_detected[i][1]));
// 		int tmp_radius = cvRound(circles_detected[i][2]);

// 		circle(circles_thresh, tmp_center, tmp_radius, Scalar(255), -1, 8, 0);
// 	}

// 	cvtColor(circles_thresh, debugss, COLOR_GRAY2BGR);
// 		circles_debug.write(debugss);

// 	sem_wait(&d->det_sem);

// 	circles_thresh.copyTo(d->circles_thresh);
// 	// d->circles_thresh = circles_thresh.clone();

// 	if(d->circles_ready == 0) {
// 		//notify the movement task that my part is ready
// 		d->circles_ready = 1;
// 		sem_post(&d->priv_circ);
// 	}

// 	sem_post(&d->det_sem);
// 	*/
// 	//cvtColor(threshold, gray, COLOR_GRAY2BGR);

// }

//
// void detect_circles() {

// 	long int current, start;
// 	int i = 0;
	
// 	while(1) {
// 		start = get_time_ms();
// 		//cout << "pigliaml il frame\n";

// 		circles_detection(&handler, &detection_handler);
		
// 		current = get_time_ms();

// 		i++;
// 		//cout << "detect circ: " << current-start << " ms, frame n. " << i << "\n";
// 		fflush(stdout);

// 		ptask_wait_for_period();
// 	}
// }

void serial_receive(struct components_handler_t *c) {
	char input_sensor [INPUT_SENSOR_LENGTH];
	char next_char = 0;
	char *eptr;
	long int sensor_value = 0;
	int i = 0;
	int current_obj_det = 0;

	input_sensor[INPUT_SENSOR_LENGTH - 1] = 0;

	sem_wait(&c->acc_serial_out);
	printf("SENSOR leggo \n");

	//clean up the serial from bad data
	while(next_char == 0 || next_char == '\n') {
		next_char = serialGetchar(fd_serial);
		printf("%d ", next_char);
	}

	//populate the input array with the values from our serial
	while((next_char >= '0' && next_char <= '9') && i < INPUT_SENSOR_LENGTH - 1) {
		input_sensor[i] = next_char;
		printf("%d ", next_char);

		next_char = serialGetchar(fd_serial);
		i++;

		if(i >= INPUT_SENSOR_LENGTH - 1) {
			printf("Errore nell'acquisizione del sensore, il messaggio supera il buffer\n");
		}
	}

	printf("\n");
	
	input_sensor[i] = 0; //close the string

	// printf("\n\n %s \n", input_sensor);
	
	sensor_value = strtol(input_sensor, &eptr, 10);

	//insert and elaborate


	c->sens_dist_val = (int) sensor_value;

	//printf("il sensore ha trovato: %d\n\n\n", sensor_value);


	if(sensor_value <= MIN_OBSTACLE_DIST) {
		current_obj_det = 1; //true
		//printf("Obastacle found %d!\n\n\n\n\n\n\n\n\n\n", sensor_value);
	} else {
		// if(sensor_value != 0)
		// 	printf("Obastacle not found %d!\n", sensor_value);
		// else
		// {
		// 	printf("riga 555\n");
		// }
		
		current_obj_det = 0; //false
	}

	//block the car if object is found
	if(current_obj_det == 1 && c->last_object_detected == 0) {
		//printf("BLOCKING THE CAR!!!\n");
		//serial_send(ENGINE_ID, 0);
		serial_send(SERVO_ID, NOT_CHANGE);
	} else {
;	// 	printf("curr: %d, last: %d\n", current_obj_det, c->last_object_detected);
	}

	//update object detected
	c->last_object_detected = current_obj_det;
	
	// printf("Ricevuto valore del sensore di prossimita': %d\n", c->sens_dist_val);

	sem_post(&c->acc_serial_out);
	// printf("SENSOR lascio \n");
}

void sensor_bridge() {

	long int current, start;
	int i = 0;
	
	while(1) {
		start = get_time_ms();
		//cout << "pigliaml il frame\n";

		if(serialDataAvail(fd_serial) == -1) {
			printf("serial error\n");
		} else {
			//read
			printf("%d data available\n", serialDataAvail(fd_serial));
			while (serialDataAvail(fd_serial) >= SER_MESS_LENGTH) {
				printf("leggo\n");
				serial_receive(&components_handler);
			}

			// printf("finito di leggere la seriale\n");
		}
		
		current = get_time_ms();

		i++;
		//cout << "detect circ: " << current-start << " ms, frame n. " << i << "\n";
		fflush(stdout);

		ptask_wait_for_period();
	}
}

//calculate the values of the servo 
bool calc_movement(struct detection_handler_t *d, int& servo_val, int& engine_val) {

	int x = FRAME_WIDTH / 2, y;
	bool objectFound = false;
	Mat local_col_thresh;//, local_circ_thresh, bitwise_thresh;

	//wait for circles and color threshold to be ready
	//sem_wait(&d->priv_circ);						//tolgo parte cerchi
	sem_wait(&d->priv_col);

	sem_wait(&d->det_sem);

	//DEBUG
	// if(d->circles_ready == 0 || d->color_ready == 0)
	// 	printf("NON DOVREI ESSERE QUI\n\n");

	//consume their part
	// d->circles_ready = 0;														//tolgo parte cerchi
	d->color_ready = 0;

	// printf("color and threshold part is ready!!!\n\n\n\n\n");
	// fflush(stdout);

//DEBUG
	if(d->color_thresh.empty())
		printf("Malisssssssssimo\n\n\n\n\n");
	// if(d->circles_thresh.empty())
	// 	printf("Malisssssssssimo\n\n\n\n\n");


	// local_col_thresh = d->color_thresh.clone();
	// local_circ_thresh = d->circles_thresh.clone();
	d->color_thresh.copyTo(local_col_thresh);
	// d->circles_thresh.copyTo(local_circ_thresh);														//tolgo parte cerchi

	sem_post(&d->det_sem);
	
//DEBUG
	if(local_col_thresh.empty()) 
			printf("\n\n\n\nlocal col EMPTY");
	// if(local_circ_thresh.empty())														//tolgo parte cerchi
	// 		printf("\n\n\n\nlocal circ EMPTY");
	
	// printf("prima del bitwise\n\n\n\n\n");															//tolgo parte cerchi
	// fflush(stdout);
	// bitwise_and(local_col_thresh, local_circ_thresh, bitwise_thresh);
	// printf("dopo il bitwise\n\n\n\n\n");

	//DEBUG
		Mat gray;

		// if(bitwise_thresh.empty()) 														//tolgo parte cerchi
		// 	printf("\n\n\n\nBITWISE EMPTY");
		// 	fflush(stdout);
		// cvtColor(bitwise_thresh, gray, COLOR_GRAY2BGR);														//tolgo parte cerchi
		// 																			cvtColor(local_col_thresh, gray, COLOR_GRAY2BGR);
		// imwrite("bitwise.jpg", gray);


	// morphOps(bitwise_thresh);														//tolgo parte cerchi
																					morphOps(local_col_thresh);
	//printf("ricerca dell'oggetto\n\n");
	fflush(stdout);
	
	// trackFilteredObject(x, y, bitwise_thresh);														//tolgo parte cerchi
																					objectFound = trackFilteredObject(x, y, local_col_thresh);
	
	// servo_val = ((int)(x / FRAME_WIDTH) * SERVO_RANGE) + SERVO_CENTER;
	if(objectFound) {
		servo_val = (x * SERVO_RANGE)  / FRAME_WIDTH;
		servo_val -= (SERVO_RANGE / 2); //move the val in a range of -(range/2) | +(range/2)
		servo_val += SERVO_CENTER;
		engine_val = 255;
		cout << "X trovata: " << x << " servo_val: " << servo_val << endl;
	} else 
		cout << "object not found" << endl;
	
	return objectFound;
}

void serial_send(int componentId, int componentValue) {
	char ser_message[SER_MESS_LENGTH];

	//with a negative number we keep the component unchanged
	if(componentValue >= 0) {
		sprintf(ser_message, "<%d:%d>", componentId, componentValue);
		serialPuts(fd_serial, ser_message);
		printf("%s\n", ser_message);
	}
	
}

void send_movement(struct components_handler_t *c, int servo_val, int engine_val) {


	sem_wait(&c->acc_serial_out);
	// printf("MOVE prendo \n");

	//update the movement path (only if an object is not detected)
	if(c->last_object_detected == 0) {
		serial_send(ENGINE_ID, engine_val);
		
		if(servo_val >= SERVO_CENTER - (SERVO_RANGE / 2) && servo_val <= SERVO_CENTER + (SERVO_RANGE / 2)) {
			printf("muovo servo a %d\n", servo_val);
			serial_send(SERVO_ID, servo_val);	
		}
		else
			printf("ERRORE: servo value not valid %d\n", servo_val);

	} else {
		printf("NON POSSO, MACCHINA BLOCCATA\n");
	}

	sem_post(&c->acc_serial_out);
	// printf("MOVE lascio \n");
}

//periodic task - moves the car using the parameters calculated by the other tasks
void check_move() {

	long int current, start;
	bool objectFound;
	int i = 0, servo_val = 0, engine_val = 0;
	while(1) {
		//cout << "pigliaml il frame\n";
		start = get_time_ms();		

		objectFound = calc_movement(&detection_handler, servo_val, engine_val);


		// if(checkObstacle(&sensors_val) == 0) {  														//DA CAMBIARE
		// 	//obstacle si not present, we continue
		// 	send_movement(servo_val, engine_val);
		// } else {
		// 	//obstacle si present, we stop
		// 	send_movement(NOT_CHANGE, 0);
		// }
		if(objectFound) {
			send_movement(&components_handler, servo_val, engine_val);
		} else {
			send_movement(&components_handler, SERVO_CENTER, 0);
		}
		

		current = get_time_ms();

		i++;
		//cout << "move: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}






void create_tasks() {
    tpars param;
	tpars mover_prm;
	tpars param_sec;
	tpars sensor;
	int ret;

    init();

	param = TASK_SPEC_DFL;
	param.period = tspec_from(PER, MILLI);
	param.rdline = tspec_from(DREL, MILLI);
	param.priority = PRIO;
	param.measure_flag = 1;
	param.act_flag = NOW;

	param = TASK_SPEC_DFL;
	param.period = tspec_from(PER, MILLI);
	param.rdline = tspec_from(DREL, MILLI);
	param.priority = PRIO-1;
	param.measure_flag = 1;
	param.act_flag = NOW;

	mover_prm = TASK_SPEC_DFL;
	mover_prm.period = tspec_from(PER, MILLI);
	mover_prm.rdline = tspec_from(DREL, MILLI);
	mover_prm.priority = PRIO-1;
	mover_prm.measure_flag = 1;
	mover_prm.act_flag = NOW;

	ret = ptask_create_param(frame_acquisition, &param);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
	
	ret = ptask_create_param(store_video, &param);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");

	ret = ptask_create_param(detect_color, &param);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");

	// ret = ptask_create_param(detect_circles, &param);
	// if(ret == -1)
	// 	printf("Error during the creation of the tasks\n");
	
	//create the task that checks the sensors
	ret = ptask_create_param(sensor_bridge, &mover_prm);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");

	//create the task that moves the car
	ret = ptask_create_param(check_move, &mover_prm);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
}