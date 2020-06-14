#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

extern void serial_receive(struct components_handler_t *c);
extern void serial_send(int componentId, int componentValue);
extern void send_movement(struct components_handler_t *c, int servo_val, int engine_val);

#endif