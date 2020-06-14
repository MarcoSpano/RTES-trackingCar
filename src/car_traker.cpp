#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

int fd_serial;

//camera handlers
VideoCapture cap;

//video streaming
VideoWriter out_capture;
VideoWriter threshold_debug;

int ret[NUM_TASKS];

struct handler_t handler;

//it handles the synchronization between the detection tasks
struct detection_handler_t detection_handler;

//values of the various components: sensors, motors, etc.
struct components_handler_t components_handler;


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
		printf("ERROR! Unable to open camera\n");
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

	//wait some time to block all the tasks
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

//periodic task to take frames from the camera 
void frame_acquisition() {
	
	while(1) {

		get_frame(&handler);

		ptask_wait_for_period();
	}
}

//take all the frames and store them in a video
void store_video() {

	while(1) {
		
		store_frame(&handler);

		ptask_wait_for_period();
	}
}

void detect_color() {

	while(1) {

		preproc_detect(&handler, &detection_handler);

		ptask_wait_for_period();
	}
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