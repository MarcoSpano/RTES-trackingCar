#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

//------------------------------------------------------------------------------
//		This file contains the main variable istantions and functions.
//		Here are present:
//		- main global variables
//		- init functions
//		- close and error functions
//		- task functions
//		- task creation function
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//						GLOBAL VARIABLE ISTANTIATION
//------------------------------------------------------------------------------

int					fd_serial;			//file descriptor for the serial device
int					ret[NUM_TASKS];		//ptask ID for each task
VideoCapture		cap;				//camera instantiation
VideoWriter			video_camera;		//video streaming of the onboard camera
VideoWriter			video_processed;	//video streaming of the processed frames
struct camera_h		camera;				//synchronization for accessing the frames 
struct detection_h	detection;			//synchronization for the detection part
struct control_h	control;			//synchronization for the serial part

//------------------------------------------------------------------------------
//							INIT FUNCTIONS
//------------------------------------------------------------------------------

/**
 * Initializes all the resources and global variables needed from the application
 */
void init() {
	Mat		tmp;	//temporary variable to synchronize the camera streaming

    ptask_init(SCHED_FIFO, GLOBAL, PRIO_CEILING);
	init_strc(&camera, &control, &detection);
	init_resources();

	//setup of the camera
	cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	// opencv has an internal buffer. For RT purposes, we cap it at only 1 frame
	cap.set(CAP_PROP_BUFFERSIZE, 1);
	
	cap.read(tmp);	//sync of frame capture

	//flushing before the start
	serialFlush(fd_serial);
	fflush(stdout);
	fflush(stdin);

	//sync with the serial communication device
	while (serialDataAvail(fd_serial) == 0) {}
}

/**
 * Initialization of the shared structs
 * @param	: struct camera_h *h;		camera sync and resources
 * @param	: struct control_h *c;		serial, sensors and actuators sync and data
 * @param	: struct detection_h *d;	detection related sync and resources
 */
void init_strc(struct camera_h *h, struct control_h *c, struct detection_h *d) {

    //semaphore initialisation
    sem_init(&h->acc_frame, 0, 1);
	sem_init(&c->acc_serial_out, 0, 1);
	sem_init(&d->det_sem, 0, 1);
	sem_init(&d->priv_col, 0, 0);

	h->newFrame = 0;
	h->newFrame2Det = 0;
	h->newDetection = 0;

	d->color_ready = 0;

	c->sens_dist_val = 0;
	c->last_obstacle_detected = 1;
}

/**
 * Initialization of the external resources, as the camera, serial communication
 * and video streamings
 */
void init_resources() {

	//opens the camera
    cap.open(0);
	if (!cap.isOpened()) {
		printf("ERROR! Unable to open camera\n");
        exit(-1);
    }

	//creation of the video streamings
	video_camera.open("1_onboard_video.avi", VideoWriter::fourcc('M','J','P','G'), 
		VIDEO_FRAMERATE, Size(FRAME_WIDTH, FRAME_HEIGHT));
	video_processed.open("2_car_view.avi", VideoWriter::fourcc('M','J','P','G'), 
		VIDEO_FRAMERATE, Size(FRAME_WIDTH, FRAME_HEIGHT));

	//open and initialise serial connection
	if ((fd_serial = serialOpen ("/dev/ttyUSB0", BAUD_RATE)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		exit(-1);
	}

	if (wiringPiSetup () == -1) {
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror (errno));
		exit(-1);
	}
}

//------------------------------------------------------------------------------
//					CLOSE FUNCTIONS AND ERROR HANDLING
//------------------------------------------------------------------------------

/**
 * It is called when an error occured during the application execution (so when
 * it is already initialised and running). It puts the actuators in their
 * defalut position and it stops the execution showing an error message.
 * @param	: char *f;			function name where the error occured
 * @param	: char *msg;		error message
 */
void app_error(char *f, char *msg) {

	//put the actuators to default values
	serial_send(ENGINE_ID, 0);
	serial_send(SERVO_ID, SERVO_CENTER);

	ptask_syserror(f, msg);
}

/**
 * Closes the app, displaying some approximated time measurements
 */
void close_app() {
	int		i;					//loop index
	tspec	wc_et[NUM_TASKS];	//to store the worst case execution time

	//pointers to structs (already defined up)
	struct	camera_h *h = &camera;
	struct	control_h *c = &control;
	struct	detection_h *d = &detection;

	//access the serial to put the actuators to default values
	sem_wait(&c->acc_serial_out);
	serial_send(ENGINE_ID, 0);
	serial_send(SERVO_ID, SERVO_CENTER);

	//take all the other semaphores to block all the tasks
	sem_wait(&h->acc_frame);
	sem_wait(&d->det_sem);

	//wait some time to ensure tasks are blocked
	usleep(WAIT_BEFORE_CLOSE);

	//close the video streamings
	video_camera.release();
	video_processed.release();
	
	printf("Time measurements approximated\n");
	for (i = 0; i < NUM_TASKS; i++) {
		wc_et[i] = ptask_get_wcet(ret[i]);
		printf("\nTask id %d:\nWorst case execution time: %ld\n",
			i, tspec_to(&wc_et[i], MILLI));
	}
	fflush(stdout);
}

//------------------------------------------------------------------------------
//						TASK FUNCTIONS AND CREATION
//------------------------------------------------------------------------------

/**
 * Periodic task that takes frames from the camera
 */
void frame_acquisition() {
	while (1) {
		get_frame(&camera);

		ptask_wait_for_period();
	}
}

/**
 * Periodic task that takes the frames and store them into video streamings
 */
void store_video() {
	while (1) {
		store_frame(&camera);

		ptask_wait_for_period();
	}
}

/**
 * Periodic task that filters the color from the current frame 
 */
void detect_color() {
	while (1) {
		preproc_detect(&camera, &detection);

		ptask_wait_for_period();
	}
}

/**
 * Periodic task that checks the sensors values and behave accordingly
 */
void sensor_bridge() {
	while (1) {
		if (serialDataAvail(fd_serial) == -1)
			app_error((char *) "sensor_bridge", 
				(char *) "ERROR! Can't access to the serial communication");
		else {
			while (serialDataAvail(fd_serial) >= SER_MESS_LENGTH) {
				serial_receive(&control);
			}
		}

		ptask_wait_for_period();
	}
}

/**
 * Periodic task that moves the car using the informations
 * calculated by the detection and sensor tasks
 */
void check_move() {
	bool	objectFound;					//true if an object is found on the frame
	int		servo_val = 0, engine_val = 0;	//actuators variables

	while (1) {
		objectFound = calc_movement(&detection, servo_val, engine_val);

		if (objectFound)
			send_movement(&control, servo_val, engine_val);
		else
			send_movement(&control, SERVO_CENTER, 0);

		ptask_wait_for_period();
	}
}

/**
 * Sets the parameters of each task before their creation
 * @param	: tpars	params[];	parameters of each task, as priority, period, etc.
 */
void set_task_parameters(tpars (&params)[NUM_TASKS]) {
	int		i;		//loop index

	for (i = 0; i < NUM_TASKS; i++) {
		params[i] = TASK_SPEC_DFL;
		params[i].measure_flag = 1;
		params[i].act_flag = DEFERRED;
	}

	//frame acquisition
	params[0].period = tspec_from(FRAME_PER, MILLI);
	params[0].priority = FRAME_PRIO;

	//sensor bridge
	params[1].period = tspec_from(SENSOR_PER, MILLI);
	params[1].priority = SENSOR_PRIO;

	//store video
	params[2].period = tspec_from(STORE_PER, MILLI);
	params[2].priority = STORE_PRIO;

	//object detection
	params[3].period = tspec_from(DETECT_PER, MILLI);
	params[3].priority = DETECT_PRIO;

	//check & move
	params[4].period = tspec_from(MOVE_PER, MILLI);
	params[4].priority = MOVE_PRIO;
}

/**
 * Creates and activates the tasks
 */
void create_tasks() {
    tpars	params[NUM_TASKS];	//parameters of the tasks
	int		i;					//loop index

	set_task_parameters(params);

	ret[0] = ptask_create_param(frame_acquisition, &params[0]);
	if (ret[0] == -1)
		app_error((char *) "create_tasks", 
			(char *) "Error during the creation of the tasks");

	ret[1] = ptask_create_param(sensor_bridge, &params[1]);
	if (ret[1] == -1)
		app_error((char *) "create_tasks", 
			(char *) "Error during the creation of the tasks");

	ret[2] = ptask_create_param(store_video, &params[2]);
	if (ret[2] == -1)
		app_error((char *) "create_tasks", 
			(char *) "Error during the creation of the tasks");

	ret[3] = ptask_create_param(detect_color, &params[3]);
	if (ret[3] == -1)
		app_error((char *) "create_tasks", 
			(char *) "Error during the creation of the tasks");

	ret[4] = ptask_create_param(check_move, &params[4]);
	if (ret[4] == -1)
		app_error((char *) "create_tasks", 
			(char *) "Error during the creation of the tasks");

	//activate the tasks
	for (i = 0; i < NUM_TASKS; i++) {
		ptask_activate(ret[i]);
	}
}