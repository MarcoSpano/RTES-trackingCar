#ifndef DETECTION_H
#define DETECTION_H

extern void preproc_detect(struct handler_t *h, struct detection_handler_t *d);
extern bool calc_movement(struct detection_handler_t *d, int& servo_val, int& engine_val);

#endif