#include "../lib/car_traker.h"

using namespace cv;
using namespace std;

//------------------------------------------------------------------------------
//		This file contains all the functions related to the onboard
//		control system, that communicates through the serial to
//		gain sensors data and send actuators data (steering, engine etc.)
//------------------------------------------------------------------------------

/**
 * Receives and parses serial packets, reacting according to the
 * sensor values acquired
 * @param	: struct control_h *c;	pointer to the control struct
 */
void serial_receive(struct control_h *c) {
	char		input_sensor[INPUT_SENSOR_LEN];	//RX buffer for serial
	char		next_char = 0;					//character received from the serial
	char		*eptr;							//needed by strtol, not used
	long int	sensor_value = 0;				//value of the sensor after parsing
	int			i = 0;							//loop index

	input_sensor[INPUT_SENSOR_LEN - 1] = 0;		//array init

	sem_wait(&c->acc_serial_out);

	//clean up the serial from bad data
	while (next_char == 0 || next_char == '\n') {
		next_char = serialGetchar(fd_serial);
	}

	//populate the input array with the values received from the serial
	while ((next_char >= '0' && next_char <= '9') && i < INPUT_SENSOR_LEN - 1) {
		input_sensor[i] = next_char;

		next_char = serialGetchar(fd_serial);
		i++;

		if (i >= INPUT_SENSOR_LEN - 1) {
			printf("An error occured during the sensor value acquisition, \
				serial packet exceeds the rx buffer\n");
		}
	}
	input_sensor[i] = 0;	//closes the string
	
	sensor_value = strtol(input_sensor, &eptr, 10);	//parse the string to an int
	printf("sensor value: %d\n", sensor_value);

	obstacle_check(c, (int) sensor_value);	//reacts accordingly to the sensor value

	sem_post(&c->acc_serial_out);
}

/**
 * Checks the ultrasonic sensor value and stops the car if it is under a
 * certain threshold
 * @param	: struct control_h *c;		pointer to the control struct
 * @param	: int sensor_value;			value of the ultrasonic sensor
 */
void obstacle_check(struct control_h *c, int sensor_value) {
	int		current_obst_det = 0;	//1 if an obstacle is too close to the car, 0 instead

	c->sens_dist_val = (int) sensor_value;	//updates the sensor value in the struct

	if (sensor_value <= MIN_OBSTACLE_DIST)
		current_obst_det = 1;	//true
	else 
		current_obst_det = 0;	//false

	//block the car if an obstacle is found
	if (current_obst_det == 1 && c->last_obstacle_detected == 0) {
		printf("BLOCKING THE CAR!!!\n");
		serial_send(ENGINE_ID, 0);
		serial_send(SERVO_ID, NOT_CHANGE);
	}

	//updates obstacle detected value in the struct
	c->last_obstacle_detected = current_obst_det;
}

/**
 * Sends a packet to the serial communication
 * @param	: int componentId;		ID of the actuator
 * @param	: int componentValue;	new value of the actuator
 */
void serial_send(int componentId, int componentValue) {
	char	ser_message[SER_MESS_LENGTH];	//TX buffer for serial

	//with a negative number we keep the component value unchanged
	if (componentValue >= 0) {
		sprintf(ser_message, "<%d:%d>", componentId, componentValue);
		serialPuts(fd_serial, ser_message);
	}
}

/**
 * Sends the new values for the actuators to the serial communication,
 * doing that only if the sensor didn't find a close obstacle.
 * @param	: struct control_h *c;		pointer to the control struct
 * @param	: int servo_val;			value of the steering part
 * @param	: int engine_val;			value of the engine part
 */
void send_movement(struct control_h *c, int servo_val, int engine_val) {
	int		lower_val, upper_val;		//max and min values for the steering part

	sem_wait(&c->acc_serial_out);

	//update the movement path (only if an object is not detected)
	if (c->last_obstacle_detected == 0) {
		serial_send(ENGINE_ID, engine_val);
		
		lower_val = SERVO_CENTER - (SERVO_RANGE / 2);
		upper_val = SERVO_CENTER + (SERVO_RANGE / 2);
		if (servo_val >= lower_val && servo_val <= upper_val) {
			printf("moving servo to: %d\n", servo_val);
			serial_send(SERVO_ID, servo_val);	
		}
		else printf("Warning: servo value not valid %d\n", servo_val);

	} else printf("Can't move the car, it is blocked!\n");

	sem_post(&c->acc_serial_out);
}