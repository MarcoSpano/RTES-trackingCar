#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

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