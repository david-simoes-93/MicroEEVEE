#include "Sensors.h"
#include "FollowTheWall.h"

// Target is start position
double radiusConst = 200;
int targetX = 0, targetY = 0;
int modder = -1;
int left = 0, right = 0;

bool bVis = false, oldBVis = false, oldBVis2 = false;
bool rotate_right = false;

void follow_target_point() {
    double meToTargetVector[2];
    double myDirectionVector[2];

    meToTargetVector[0] = (target_point_x - x);
    meToTargetVector[1] = (target_point_y - y);
    myDirectionVector[0] = cos(t);
    myDirectionVector[1] = sin(t);

    double radianBeaconDir =
            atan2(meToTargetVector[1], meToTargetVector[0]) - atan2(myDirectionVector[1], myDirectionVector[0]);
    beaconDir = -normalizeAngle(radianBeaconDir) * (180 / PI);

    printf("My position %d,%d;      Target Position %d,%d;         MyAngle %2.0fº     AngleToNextPoint %dº\n",
           x, y, targetX, targetY, (t * 180 / PI), beaconDir);   //NEGATIVO PRA ESQUERDA
}

bool servoControl() {
    bool vis_tmp = false;
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

    oldBVis2 = oldBVis;
    oldBVis = bVis;

    beaconDir = (left + right) / 2 * 6;
    modder = rotate_right ? 1 : -1;

    currServoPos = visible ? currServoPos + modder * 1 : currServoPos + modder * 2;
    if (currServoPos > POS_RIGHT) { currServoPos = POS_RIGHT; }
    else if (currServoPos < POS_LEFT) { currServoPos = POS_LEFT; }

    setServoPos(currServoPos);
    return vis_tmp;
}
