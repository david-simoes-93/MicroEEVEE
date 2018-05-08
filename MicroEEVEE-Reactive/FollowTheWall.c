#include "Sensors.h"
#include "rmi-mr32.h"
#include <stdlib.h>

#include "FollowTheWall.h"

bool wallOnTheRight = false, wallOnTheLeft = false;
bool facedBeacon = false;
double lastKnownBeaconAngle = 0, lastKnownBeaconCompass = 0;


#define KP_WALL 5

int my_state = 0, obstacleState = 0, obst_history = 0;
bool dist_blind = false, found_wall_after_dead = false;
double xx = 100, dead_angle_Xpos = 0, dead_angle_Ypos = 0, sensor_dist_history=0;
#define OBSTACLE 0
#define NO_WALL 1

void atBeacon() {
    printf("Stopped following wall!\n");
    wallOnTheRight = false;
    wallOnTheLeft = false;
}

char *ftw_getName() {
    return "FollowTheWall";
}


void followWalls(int speed) {

    int sensor;

    if (wallOnTheRight) {
        sensor = obstValRight;
    } else {
        sensor = obstValLeft;
    }
    //sensor_dist_history = sensor; //TODO ver se é preciso
    double error = 0;
    int cmdVel;

    if (obstValFront < 23) {
        error -= 13.0; //acertar este valor
    } else {

        if (sensor > 40) error += 4.0; //TODO 4.0
        else if (sensor > 35) error += 2.0;
        else if (sensor > 30) error += 1.0;
        else if (sensor < 25) error -= 1.0;
        else if (sensor < 15) error -= 2.0; //CHANGE if NEEDED
    }

    cmdVel = (int) ((KP_WALL * error));
    setVel2(speed + cmdVel, speed - cmdVel);
}

void perform_blind(int speed) {
    found_wall_after_dead = false;

    double x, y, t;
    getRobotPos(&x, &y, &t);

    if (obstValFront < 17) { //TODO see value
        dist_blind = true;
        setVel2(0, 0);
        return;
    }

    if (sqrt(pow((dead_angle_Xpos - x), 2) + pow((dead_angle_Ypos - y), 2)) >= xx) {
        dist_blind = true;
    }

    setVel2(speed, speed);
}


int check_Obstacle() {
    int sensor;

    if (sensor < 40 || obstValFront < 23 || !found_wall_after_dead) {
        if (sensor < 30) {
            found_wall_after_dead = true;
        }
        return OBSTACLE;
    }

    // NO_WALL (first time)
    if (obst_history == OBSTACLE) {
        xx = ((1 / sqrt(2)) * sensor_dist_history - 5.5) * 10 + 100;//mm
        if (xx > 200)
            xx = 200;
        dist_blind = false;

        double t;
        getRobotPos(&dead_angle_Xpos, &dead_angle_Ypos, &t);

    }

    return NO_WALL;
}


void ftw_execute() {

    switch (my_state) {
        case 0 :
            followWalls(35);

            if (obstacleState == NO_WALL && !dist_blind) {
                my_state = 1;
            }
            break;
        case 1 :
            perform_blind(35);

            if (dist_blind) {
                my_state = 0;
            }
            break;
    }
}

void prep_ftw() {
    obst_history = obstacleState;
    obstacleState = check_Obstacle();

    if (visible) {
        lastKnownBeaconAngle = beaconDir;
        lastKnownBeaconCompass = compass - lastKnownBeaconAngle;
    } else {
        lastKnownBeaconAngle = compass - lastKnownBeaconCompass;
    }
}

bool ftw_isPossible() {
    // Beacon visible, on the right side, with a wall on the right
    bool rightWallOnBeaconSide =
            visible && obstValRight < 20 && lastKnownBeaconAngle >= 0 && lastKnownBeaconAngle <= 60;
    bool leftWallOnBeaconSide = visible && obstValLeft < 20 && lastKnownBeaconAngle <= 0 && lastKnownBeaconAngle >= -60;
    bool beaconNotVisWithWall = !visible && (obstValLeft < 20 || obstValRight < 20);

     if (facedBeacon && visible && (wallOnTheRight || wallOnTheLeft) &&
             (lastKnownBeaconAngle < 60 && lastKnownBeaconAngle > -60)
             && obstValFront > 15) {
        wallOnTheRight = false;
        wallOnTheLeft = false;
    }
        // IF NOT FOLLOWING WALL and near a wall
    else if ((!wallOnTheRight && !wallOnTheLeft) &&
             (rightWallOnBeaconSide || leftWallOnBeaconSide || beaconNotVisWithWall)) {
        wallOnTheLeft = obstValLeft < obstValRight;
        wallOnTheRight = !wallOnTheLeft;
        facedBeacon = visible;
    }

    return wallOnTheRight || wallOnTheLeft || !dist_blind;
}

