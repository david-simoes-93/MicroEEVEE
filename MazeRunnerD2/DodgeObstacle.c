#include "Sensors.h"
#include "rmi-mr32.h"
#include <math.h>
#include <string.h>

#include "DodgeObstacle.h"

char* do_getName() {
    return "DodgeObstacle";
}

void do_execute() {

    /* Rotate right on obstacles close in front or close on the left */
    int mod = 1;
    
    if(obstValLeft > obstValRight){ 
		mod=-1;
    }

    if(obstValFront < 15)			// if obstacle in front, rotate quickly
    	setVel2(mod*40, mod*-40);	
    else							// if obstacle on the sides, rotate slowly
    	setVel2(mod*20, mod*-20);
}

bool do_isPossible() {
    /* There's an obstacle somewhere at front/right/left */
    return obstValFront < 15 || obstValLeft < 10 || obstValRight < 10;
}

