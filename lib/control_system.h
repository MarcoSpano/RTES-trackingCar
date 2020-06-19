#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

//------------------------------------------------------------------------------
//		This file contains all the function declaration 
//		related to control_system.cpp
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//						FUNCTION DECLARATION
//------------------------------------------------------------------------------

extern void serial_receive(struct control_h *c);
extern void serial_send(int componentId, int componentValue);
extern void send_movement(struct control_h *c, int servo_val, int engine_val);
void obstacle_check(struct control_h *c, int sensor_value);

#endif