//TO-DO mettere gli include dentro un file .h
//CERCARE TUTTI I TO-DO e DEBUG NEL CODICE E IMPLEMETARLI PRIMA DI ELIMINARE QUESTO MESSAGGIO!!

#include "../lib/carTraker.h"

using namespace std;
using namespace cv;

#define PER 1000
#define DREL 1100
#define PRIO 80
#define SER_MESS_LENGTH 20

int fd_serial;

//physical components values
const int SERVO_RANGE = 180;

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

//values of the various components: sensors, motors, etc.
struct comp_val_t {
    //to access the values
    sem_t acc_comp;

    //frame taken from the camera
    int x, y;

} comp_val;

void init() {
    ptask_init(SCHED_RR, GLOBAL, NO_PROTOCOL);
	
	init_handlers(&handler, &comp_val);

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
void init_handlers(struct handler_t *h, struct comp_val_t *c) {

    /* mutua esclusione */
    sem_init(&h->acc_frame,0,1);
	sem_init(&c->acc_comp,0,1);
}



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
		cout << "frame acquisition: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}

//store the frame acquired inside the video
void store_frame(struct handler_t *h) {

	Mat local_frame;

	sem_wait(&h->acc_frame);

	local_frame = h->frame.clone();

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
		cout << "store video: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}










void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(7,7));

	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	
	
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

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
				}else {
					objectFound = false;
					x = 0;
					y = 0;
				}

				cout << "TEMP " << index << ":\n	X: " << x << "\n	Y: " << y << endl;
			}
			//let user know you found an object
			if(objectFound ==true)
				cout << "object found";

		}else cout << "TOO MUCH NOISE! ADJUST FILTER";
	}
}



//preprocess of the frame
void preproc_detect(struct handler_t *h, struct comp_val_t *c) {
	int x = 0, y = 0;

	Mat local_frame, HSV, threshold, gray;

	sem_wait(&h->acc_frame);

	local_frame = h->frame.clone();

	sem_post(&h->acc_frame);


	//convert frame from BGR to HSV colorspace
	cvtColor(local_frame, HSV, COLOR_BGR2HSV);
	inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
	
	morphOps(threshold);
	
	trackFilteredObject(x, y, threshold, local_frame);

	cvtColor(threshold, gray, COLOR_GRAY2BGR);
	threshold_debug.write(gray);

	imwrite("test.jpg", local_frame);

	//cout << "FINAL:\nX: " << x << "\nY: " << y << endl;
	

	sem_wait(&c->acc_comp);

	c->x = x;
	c->y = y;

	sem_post(&c->acc_comp);
	
	//cvtColor(threshold, gray, COLOR_GRAY2BGR);

}

//periodic task to take frames from the camera 
void detect_track() {

	long int current, start;
	int i = 0;
	
	while(1) {
		start = get_time_ms();
		//cout << "pigliaml il frame\n";

		preproc_detect(&handler, &comp_val);
		
		current = get_time_ms();

		i++;
		cout << "detect: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}

//calculate the values of the servo 
void calc_movement(struct comp_val_t *c, int& servo_val, int& engine_val) {

	int x, y;

	sem_wait(&c->acc_comp);

	x = c->x;
	y = c->y;

	sem_post(&c->acc_comp);

	cout << "x: " << x << " and y: " << y << "\n\n\n\n\n";

	

	servo_val = (int)(x * SERVO_RANGE / FRAME_WIDTH);

	cout << "servo_val: " << servo_val << endl;
}

void send_movement(int componentId, int componentValue) {
	char ser_message[SER_MESS_LENGTH];

	sprintf(ser_message, "<%d:%d>", componentId, componentValue);
	serialPuts(fd_serial, ser_message);
}

void receive_sensor_data(int& componentId, int& componentValue) {
	printf ("%c", serialGetchar(fd_serial)) ;
}

//periodic task - moves the car using the parameters calculated by the other tasks
void check_move() {

	long int current, start;
	int i = 0, servo_val = 0, engine_val = 0;
	int componentId, componentValue;
	while(1) {
		//cout << "pigliaml il frame\n";
		start = get_time_ms();

		if(serialDataAvail(fd_serial) == -1) {
			printf("error\n");
		} else {
			//read
			printf("%d data available\n", serialDataAvail(fd_serial));
			while (serialDataAvail(fd_serial) != 0) {
				receive_sensor_data(componentId, componentValue);
			}
		}
		
		calc_movement(&comp_val, servo_val, engine_val);
		send_movement(2, servo_val);

		current = get_time_ms();

		i++;
		cout << "move: " << current-start << " ms, frame n. " << i << "\n";

		ptask_wait_for_period();
	}
}






void create_tasks() {
    tpars param;
	tpars mover_prm;
	tpars param_sec;
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

	fflush(stdout);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
	else
		printf("%i\n", ret);
	
	ret = ptask_create_param(store_video, &param);

	fflush(stdout);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
	else
		printf("%i\n", ret);

	ret = ptask_create_param(detect_track, &param);

	fflush(stdout);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
	else
		printf("%i\n", ret);

	//create the task that moves the car
	ret = ptask_create_param(check_move, &mover_prm);

	fflush(stdout);
	if(ret == -1)
		printf("Error during the creation of the tasks\n");
	else
		printf("%i\n", ret);
}