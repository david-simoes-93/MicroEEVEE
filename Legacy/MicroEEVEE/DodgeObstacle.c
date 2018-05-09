#include "Sensors.h"
#include "mr32.h"
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
    	//setVel2(mod*40, mod*-40);
        rotateRel_basic(50, PI/2);
    else{							// if obstacle on the sides, rotate slowly
        printf(" Não devia acontecer \n");
        setVel2(mod*20, mod*-20);
    }
}


bool do_isPossible() {
    /* There's an obstacle somewhere at front/right/left */
    return obstValFront < 15 || obstValLeft < 5 || obstValRight < 5;
}

