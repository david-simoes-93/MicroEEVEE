/* 
 * File:   Sensors.h
 * Author: nuno
 *
 * Created on October 28, 2014, 11:28 AM
 */

#include <math.h>
#include "bluetooth_comm.h"
#include "mr32.h"

#ifndef SENSORS_H
#define	SENSORS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define POS_LEFT		-15
#define POS_RIGHT		15
#define compass 	(t * 180 / PI)

int visible, returning_home;
int x, y;
double t;
double obstValFront, obstValLeft, obstValRight;
int beaconDir, currServoPos;
int target_point_x, target_point_y;
int ground_sensor_buffer[5];

bool servoControl();
void follow_target_point();

#ifdef	__cplusplus
}
#endif

#endif	/* SENSORS_H */

