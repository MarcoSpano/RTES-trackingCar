//TO-DO mettere gli include dentro un file .h
//CERCARE TUTTI I TO-DO NEL CODICE E IMPLEMETARLI PRIMA DI ELIMINARE QUESTO MESSAGGIO!!

#include "../lib/carTraker.h"

using namespace std;
using namespace cv;

#define PER 100
#define DREL 200
#define PRIO 80

VideoCapture cap;
VideoWriter out_capture;

struct handler_t {
    //to access the frame resource in mutex
    sem_t acc_frame;

    //
    Mat frame;

    // semafori privati
    // sem_t priv_pizzaiolo;
    // sem_t priv_cliente[N];

} handler;

void init() {
    ptask_init(SCHED_RR, GLOBAL, NO_PROTOCOL);
	
	init_handler(&handler);

	//--- open the camera
    cap.open(0);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        //return -1;
    }

	out_capture.open("4video_car.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(640,480));
}

/* inizializzazione della struttura condivisa */
void init_handler(struct handler_t *h) {

    /* mutua esclusione */
    sem_init(&h->acc_frame,0,1);
}

//get a frame from the camera and stores it in matrix 'frame'
void get_frame(struct handler_t *h) {

	sem_wait(&h->acc_frame);

	cap.read(h->frame);

	cout << "magari\n";

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

	while(1) {
		cout << "pigliaml il frame\n";

		get_frame(&handler);

		ptask_wait_for_period();
	}
}

//store the frame acquired inside the video
void store_frame(struct handler_t *h) {

	sem_wait(&h->acc_frame);

	cout << "piazziaml\n";
	out_capture.write(h->frame);

	sem_post(&h->acc_frame);

}

//take all the frames and store them in a video
void store_video() {

    

	while(1) {
		
		store_frame(&handler);

		ptask_wait_for_period();
	}
}

void create_tasks() {
    tpars param;
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
}