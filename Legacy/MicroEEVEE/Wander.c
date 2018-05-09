#include "Sensors.h"
#include "rmi-mr32.h"
#include <math.h>
#include <string.h>

#include "Wander.h"

char* wan_getName() {
    return "Wander";
}

void wan_execute() {
    
    led(4,1);
    if(obstValFront > 25 && obstValLeft > 20 && obstValRight > 20)
        setVel2(100,100);
    else if(obstValFront > 15 && obstValLeft > 10 && obstValRight > 10)
        setVel2(65,65);
    else
        setVel2(30,30);
}

bool wan_isPossible() {
    return true;
}
