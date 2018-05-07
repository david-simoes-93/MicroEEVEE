#include "Sensors.h"
#include "mr32.h"
#include "FollowTheWall.h"

void followPoints_() {
    //getLastPoint();
    targetX = lastPointX;
    targetY = lastPointY;

    //double myNorm, targetNorm;
    double meToTargetVector[2];
    double myDirectionVector[2];
    double myX = (double)x, myY = (double)y, myDir = t;    // 0.35 radians

    meToTargetVector[0] = (targetX - myX);
    meToTargetVector[1] = (targetY - myY);
    myDirectionVector[0] = cos(myDir);
    myDirectionVector[1] = sin(myDir);

    double radianBeaconDir =
            atan2(meToTargetVector[1], meToTargetVector[0]) - atan2(myDirectionVector[1], myDirectionVector[0]);

    beaconDir = -normalizeAngle(radianBeaconDir) * (180 / PI);
}

bool servoControl() {
    bool vis_tmp= false;
    bVis = readBeaconSens();

    if (bVis && !oldBVis) {                            // Started seeing beacon!
        vis_tmp = true;
        if (rotate_right) {
            left = currServoPos;
        } else {
            right = currServoPos;
        }

        if (currServoPos == POS_RIGHT || currServoPos == POS_LEFT) {    // Got to the end of the line
            rotate_right = !rotate_right;
        }
    } else if (!bVis && oldBVis) {                        // Stopped seeing beacon!
        if (rotate_right) { right = currServoPos - 1; }
        else { left = currServoPos + 1; }

        rotate_right = !rotate_right;
    } else if (currServoPos == POS_RIGHT || currServoPos == POS_LEFT) {    // Got to the end of the line
        if (currServoPos == POS_RIGHT) { right = POS_RIGHT + 1; }
        else { left = POS_LEFT - 1; }

        rotate_right = !rotate_right;
    }

    visible = bVis || oldBVis || oldBVis2;
    //led(2, visible);

    oldBVis2 = oldBVis;
    oldBVis = bVis;

    beaconDir = (left + right) / 2 * 6;

    //printf("Beacon at %d\n",beaconDir);

    modder = rotate_right ? 1 : -1;
    //currServoPos+=modder;

    currServoPos = visible ? currServoPos + modder * 1 : currServoPos + modder * 2;
    if (currServoPos > POS_RIGHT) { currServoPos = POS_RIGHT; }
    else if (currServoPos < POS_LEFT) { currServoPos = POS_LEFT; }

    setServoPos(currServoPos);
    return vis_tmp;


}

int checkPointsRadius() {
    int j;

    if (radiusConst != 200)
        radiusConst += 2;

    for (j = 0; j < pointsListPointer; j++) {
        if (isOnRadius(x, y, pointsList[0][j], pointsList[1][j], radiusConst)) {

            printf("Was close to point %d %d, position [%d] out of %d\n", pointsList[0][j], pointsList[1][j], j + 1,
                   pointsListPointer);
            pointsListPointer = j + 1;
            return 1;
        }
    }

    return 0;
}

int removePoint() {
    printf("Removed point %d %d!\n", pointsList[0][pointsListPointer - 1], pointsList[1][pointsListPointer - 1]);
    pointsListPointer--;
    if (pointsListPointer == 0) {
        if (radiusConst != 200)     // if point 0,0 was found with radius 150, end!
        {
            return 1;
        }
        else                        // if point 0,0 was found with radius 300, switch to radius 100
        {
            pointsListPointer++;
            radiusConst = 151;
        }
    }
    //newPointMark();
    atBeacon();
    getLastPoint();
    return 0;
}

void markPoint() {
    if (!returning_home) {
        getLastPoint();
        if (!isOnRadius(x, y, lastPointX, lastPointY, 100)) {
            addPoint(x, y);
        }
    }
}

void getLastPoint() {
    lastPointX = pointsList[0][pointsListPointer - 1];
    lastPointY = pointsList[1][pointsListPointer - 1];

}

void addPoint(double x2, double y2) {
    printf("Adding point %2.0f %2.0f\n", x2, y2);
    pointsList[0][pointsListPointer] = x2;
    pointsList[1][pointsListPointer] = y2;
    pointsListPointer++;
}

int isOnRadius(double x2, double y2, double nextX, double nextY, int radius) {
    int i;
    double dist = 0.0;
    for (i = 0; i < 2; i++) {
        dist = sqrt((pow((nextX - x2), 2)) + (pow((nextY - y2), 2)));
        //printf("dist= %f\n", dist);
        if (dist <= radius)
            return 1;
    }
    return 0;
}

double dot_product(double v[], double u[], int n) {
    int i;
    double result = 0.0;
    for (i = 0; i < n; i++)
        result += v[i] * u[i];
    return result;
}

void rotateRel_naive(double deltaAngle) {
    double x, y, t;
    double targetAngle;
    double error;
    int cmdVel;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    error = normalizeAngle(targetAngle - t);
    if (error < 0)
        cmdVel = -30;
    else
        cmdVel = 30;

    setVel2(-cmdVel, cmdVel);

    do {
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);
    } while (fabs(error) > 0.01 && (error * cmdVel) > 0);
    setVel2(0, 0);
}

