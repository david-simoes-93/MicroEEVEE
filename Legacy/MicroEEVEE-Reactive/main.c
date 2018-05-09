/* 
 * File:   main.c
 * Author: nuno
 *
 * Created on October 28, 2014, 11:26 AM
 */

#include "rmi-mr32.h"
#include <string.h>
#include <math.h>
#include "DodgeObstacle.h"
#include "FollowTheBeacon.h"
#include "FollowTheWall.h"
#include "StopAtBeacon.h"
#include "Wander.h"
#include "Sensors.h"
#include "bluetooth_comm.h"

//void rotateRel_naive(double deltaAngle);
void decide();
int lastBeh;


int main(void) {
    initPIC32();
    configBTUart(3, 115200); // Configure Bluetooth UART
    //bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART

    closedLoopControl(true);

    printf("RMI-example, robot %d\n\n\n", ROBOT);

    setVel2(0, 0); 
    int c = 0;
    initAvg();

    lastBeh=-1;

    while (1) {
        //int prevAngle=0;
        printf("Press start to continue\n");
        while (!startButton());
        enableObstSens();

        int i;
		followPoints = 0;
	
		for(i=0; i < 5; i++)
			gndVals[0] = 0;
		
        for(i=0; i<valsSize; i++){
            waitTick40ms(); 
            readAnalogSensors(); 
            calculateAverage();
        }

       //led(3,1);

        do {
            // Wait for next 40ms tick
            waitTick40ms(); 
			//servoControl();
            //waitTick20ms(); 
            
            // Fill in "analogSensors" structure
            readAnalogSensors(); 
            
            // Read ground sensor
            groundSensor = readLineSensors(70); 
	    	
	    	gndVals[c] = groundSensor;	c=(c+1)%5;
            /* Track robot position and orientation */
            getRobotPos(&x, &y, &t);
	    
            /* Average in order to diminish noise */
            calculateAverage();
	   
            servoControl();

            markPoint();
            
            //printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d, Ground_sens=", analogSensors.obstSensLeft,
            //        analogSensors.obstSensFront, analogSensors.obstSensRight, analogSensors.batteryVoltage);

            //printf("Bat_Voltage=%03d\n", analogSensors.batteryVoltage);
            //printInt(groundSensor, 2 | 5 << 16); // System call
            //printf("\n");
	  
            decide();

        } while (!stopButton());
        disableObstSens();
        setVel2(0,0);

    }
    return 0;
}

void decide() {
    //printf("\tLeft %2.2f; Front %2.2f; Right %2.2f\n", obstValLeft, obstValFront, obstValRight);
	//printf("X= %f Y= %f Compass= %f\n", x, y, compass);
	prep_ftw(); 

    if(sab_isPossible()){
        sab_execute();
        if(lastBeh!=0){
        	printf("\tExecuting behaviour %s\n", sab_getName());
        	lastBeh=0;
    	}
    } 
    else if(ftw_isPossible()){
        ftw_execute();
        if(lastBeh!=1){
            printf("\tExecuting behaviour %s\n", ftw_getName());
            lastBeh=1;
    	}
    } 
    else if(ftb_isPossible()){
        ftb_execute();
        if(lastBeh!=2){
            printf("\tExecuting behaviour %s\n", ftb_getName());
            lastBeh=2;
    	}
    } 
    else if(do_isPossible()){
        do_execute();
        if(lastBeh!=3){
            printf("\tExecuting behaviour %s\n", do_getName());
            lastBeh=3;
    	}
    } 
    else if(wan_isPossible()){
        wan_execute();
        if(lastBeh!=4){
            printf("\tExecuting behaviour %s\n", wan_getName());
            lastBeh=4;
    	}
    }
}
