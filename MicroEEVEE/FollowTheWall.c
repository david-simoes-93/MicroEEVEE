#include "Sensors.h"
#include "rmi-mr32.h"
#include <string.h>
#include <stdlib.h>

#include "FollowTheWall.h"
bool wallOnTheRight = false, wallOnTheLeft = false;
bool facedBeacon = false, keepValues=false;
double lastKnownBeaconAngle = 0, lastKnownBeaconCompass = 0;
double grausRodados = 0.0, prevComp = 0.0, prevSpeed=0, mainSide=0;
int sharpTurnCounter=-1;

void atBeacon(){
  printf("Stopped following wall!\n");
  wallOnTheRight = false; 
  wallOnTheLeft = false;
}

int minimum(int a, int b){
	if (a<b)
		return a;
	return b;
}

char* ftw_getName() {
    return "FollowTheWall";
}

void ftw_execute() {
    mainSide = obstValRight;
    double weakSide = obstValLeft, midSide = obstValFront;
    
    if (!wallOnTheRight) {
        mainSide = obstValLeft;
        weakSide = obstValRight;
    }

    led(0,0); led(1,0); led(2,0); led(3,0);
    int wallSpeed=0, opposSpeed=0;

    if ((mainSide < 10 || midSide < 5)) { 								// REALLY CLOSE WALL ON MAINSIDE OR FRONT (less than 10)
        opposSpeed = -30;        
        wallSpeed = 30;
    } else if (weakSide < 10 && mainSide > 20) {						// REALLY CLOSE WALL ON WEAKSIDE; MEDIUM WALL ON MAINSIDE (less than 10)
        opposSpeed = 30;        
        wallSpeed = -30;
    } else if ((mainSide < 15 && weakSide > 20) || midSide < 10) {  	// CLOSE WALL ON MAINSIDE OR FRONT (less than 15)
    	int dist = (minimum(mainSide, midSide)-10)*2;			// Dist between 0 and 20
    	opposSpeed = dist;
        wallSpeed = 60 + dist/2;                        
        sharpTurnCounter=-1;
    } else if (mainSide < 35 && weakSide > 25) {    					// MEDIUM WALL ON MAINSIDE (between 15 and 35) and far wall on weak side
		int dist = (mainSide-25);								// Dist between -10 and 10      
		opposSpeed = 60+dist;									// 70,50						
        wallSpeed = 60-dist;   
        prevSpeed=(mainSide-25)*2;													// prevSpeed between -20 and 20; positive means its away from wall
        sharpTurnCounter=-1;
    } else if (mainSide < 35 && weakSide <=25) {    					// MEDIUM WALL ON MAINSIDE (between 15 and 35) but medium wall on weak side as well
	   if(mainSide<weakSide-7.5){
            int dist = (mainSide-25);                           // Dist between -10 and 10
            opposSpeed = 45+dist;                               // 55,35
            wallSpeed = 45-dist;   
        } else if(weakSide<mainSide-7.5){                                 // off wall is closer!
            int dist = (weakSide-17.5)*4/3;                     // Dist between -10 and 10
            opposSpeed = 45-dist;                               // 55,35
            wallSpeed = 45+dist;                       
        } else {                                                        // walls are relatively at the same distance
            opposSpeed = 45;                               
            wallSpeed = 45;
        }
    	prevSpeed=(mainSide-25)*2;                                                  // prevSpeed between -20 and 20; positive means its away from wall	
        sharpTurnCounter=-1;											
    }
    else if (mainSide >= 35) {
      // FAR WALL ON mainSide (over 35)
        if(sharpTurnCounter<0)
            sharpTurnCounter = abs(prevSpeed)/4;                                    // turn counter between 0 and 10

    	if(prevSpeed>0){												
    	 	led(0,1); led(1,1); led(2,1); led(3,1); 
    		opposSpeed = 60;									// 60, 30-10	 curva apertada
        	wallSpeed = 35 - prevSpeed;            //35 15

        	prevSpeed=prevSpeed-3;
    	}
    	else if(prevSpeed<=0){
            led(0,0); led(1,1); led(2,1); led(3,0);
		    opposSpeed = 65;									// 65, 35-55	 Curva larga
		    wallSpeed = 35 + (-prevSpeed); 
		    prevSpeed+=3;
        }

    } else {
    	opposSpeed = 20;
        wallSpeed = 20;
    }

    if( mainSide == obstValRight){
    	setVel2(opposSpeed, wallSpeed);
    } else {
    	setVel2(wallSpeed, opposSpeed);
    }
}

void prep_ftw(){
    if (visible) {
        lastKnownBeaconAngle = beaconDir;
        lastKnownBeaconCompass = compass - lastKnownBeaconAngle;
    } else {
        lastKnownBeaconAngle = compass - lastKnownBeaconCompass;
    }

    prevComp = compass;
}

bool ftw_isPossible() {
    if(sharpTurnCounter>=0)
        sharpTurnCounter--;

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
    // podemos mudar isto e po-lo a largar a parede caso deixe d sentir uma parede e nao souber nunca do beacon
    if (!facedBeacon && visible && (wallOnTheRight || wallOnTheLeft) && sharpTurnCounter<=0 ) {
        wallOnTheRight = false;
        wallOnTheLeft = false;
    } 
    // if following wall, beacon has been visible all along, beacon is in front, no obstacles nearby and not in any loops
    else if (facedBeacon && visible && (wallOnTheRight || wallOnTheLeft) && (lastKnownBeaconAngle < 60 && lastKnownBeaconAngle > -60) 
    	&& obstValFront > 15 && mainSide > distanceNeededToAbandonWall && sharpTurnCounter<=0 ){ //&& grausRodados < 180 && grausRodados > -180) {
        wallOnTheRight = false;
        wallOnTheLeft = false;
    } 
    // IF NOT FOLLOWING WALL and near a wall
    else if ((!wallOnTheRight && !wallOnTheLeft) && (rightWallOnBeaconSide || leftWallOnBeaconSide || beaconNotVisWithWall)) {
        wallOnTheLeft = obstValLeft < obstValRight;
        wallOnTheRight = !wallOnTheLeft;

        if(!keepValues){
        	prevSpeed = 0;
        } else {
        	keepValues = 0;
        }
        facedBeacon = visible;
        grausRodados = 0;

    }

    return wallOnTheRight || wallOnTheLeft;
}

