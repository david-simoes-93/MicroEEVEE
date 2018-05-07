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
double t, oldCompass;
double obstValFront, obstValLeft, obstValRight;
int groundSensor, beaconDir, currServoPos;
int lastPointX = 0, lastPointY = 0;

int gndVals[5];
int pointsList[2][100];
int pointsListPointer;

int checkPointsRadius();
int removePoint();
void markPoint();
void getLastPoint();
void addPoint(double x2, double y2);
int lastPointX, lastPointY ;

int isOnRadius( double x, double y, double nextX, double nextY, int radius);
double dot_product(double v[], double u[], int n);
bool servoControl();
void followPoints_();
void rotateRel_naive(double deltaAngle);

#ifdef	__cplusplus
}
#endif

#endif	/* SENSORS_H */

