#include "Sensors.h"
#include "rmi-mr32.h"
#include <string.h>
#include <stdlib.h>

#include "FollowTheWall.h"
bool wallOnTheRight = false, wallOnTheLeft = false;
bool facedBeacon = false;
double lastKnownBeaconAngle = 0, lastKnownBeaconCompass = 0;

#define KP_WALL 5

void atBeacon(){
  printf("Stopped following wall!\n");
  wallOnTheRight = false; 
  wallOnTheLeft = false;
}

char* ftw_getName() {
    return "FollowTheWall";
}


void followWalls(int speed){

    int sensor;

    if(wallOnTheRight){
        sensor = analogSensors.obstSensRight;
    }
    else{
        sensor = analogSensors.obstSensLeft;
    }
    //sensor_dist_history = sensor; //TODO ver se é preciso
    double error = 0;
    int cmdVel;



    if (analogSensors.obstSensFront < 23 ){
        error -= 13.0; //acertar este valor
        /*if( !first_time ){
            count_turns++;
            first_time = true;
        }*/

    }else{

        if (sensor > 40) error += 4.0; //TODO 4.0
        else if (sensor > 35) error += 2.0;
        else if (sensor > 30) error += 1.0;
        else if (sensor < 25) error -= 1.0;
        else if (sensor < 15 ) error -= 2.0; //CHANGE if NEEDED

        //first_time = false;
    }

    /*if(laps >= 2){ //Left wall
        error = -error;
    }*/

    cmdVel = (int)((KP_WALL * error) ) ;
    setVel2(speed + cmdVel, speed - cmdVel);

}

void perform_blind(int speed){
    found_wall_after_dead = false;


    double x, y, t;
    getRobotPos(&x, &y, &t);

    if(analogSensors.obstSensFront < 17 ){ //TODO see value
        dist_blind = true;
        //printHeader("DEBUG", CLR_YELLOW);
        //printf("X : %f\n", x);
        setVel2(0, 0);
        return;
    }

    if(  sqrt(pow((dead_angle_Xpos-x),2)+ pow((dead_angle_Ypos-y),2) ) >= xx) {
        dist_blind = true;
        //printHeader("DEBUG", CLR_YELLOW);
        //printf("X : %f\n", x);
    }


    setVel2(speed, speed);

}


void ftw_execute() {
    //TODO
}

void prep_ftw(){
    if (visible) {
        lastKnownBeaconAngle = beaconDir;
        lastKnownBeaconCompass = compass - lastKnownBeaconAngle;
    } else {
        lastKnownBeaconAngle = compass - lastKnownBeaconCompass;
    }
}

bool ftw_isPossible() {
    // Beacon visible, on the right side, with a wall on the right
    bool rightWallOnBeaconSide = visible && obstValRight < 20 && lastKnownBeaconAngle >= 0 && lastKnownBeaconAngle <= 60;
    bool leftWallOnBeaconSide = visible && obstValLeft < 20 && lastKnownBeaconAngle <= 0 && lastKnownBeaconAngle >= -60;
    bool beaconNotVisWithWall = !visible && (obstValLeft < 20 || obstValRight < 20);

    //if(!wallOnTheRight && !wallOnTheLeft && (rightWallOnBeaconSide || leftWallOnBeaconSide || beaconNotVisWithWall))
		//printf("Gonna start following wall: rightW:%d leftW:%d notVisWall:%d\n",rightWallOnBeaconSide, leftWallOnBeaconSide,beaconNotVisWithWall);
    
    double distanceNeededToAbandonWall=0;
    if(abs(lastKnownBeaconAngle)<10)
    	distanceNeededToAbandonWall=15;
    else
    	distanceNeededToAbandonWall=(abs(lastKnownBeaconAngle)-10)*0.4+15;

    // IF FOLLOWING WALL , beacon wasn't visible but now is
    if (!facedBeacon && visible && (wallOnTheRight || wallOnTheLeft)) {
        wallOnTheRight = false;
        wallOnTheLeft = false;
    } 
    // if following wall, beacon has been visible all along, beacon is in front, no obstacles nearby and not in any loops
    else if (facedBeacon && visible && (wallOnTheRight || wallOnTheLeft) && (lastKnownBeaconAngle < 60 && lastKnownBeaconAngle > -60) 
    	&& obstValFront > 15){ //&& grausRodados < 180 && grausRodados > -180) {
        wallOnTheRight = false;
        wallOnTheLeft = false;
    } 
    // IF NOT FOLLOWING WALL and near a wall
    else if ((!wallOnTheRight && !wallOnTheLeft) && (rightWallOnBeaconSide || leftWallOnBeaconSide || beaconNotVisWithWall)) {
        wallOnTheLeft = obstValLeft < obstValRight;
        wallOnTheRight = !wallOnTheLeft;
        facedBeacon = visible;
    }

    return wallOnTheRight || wallOnTheLeft;
}

