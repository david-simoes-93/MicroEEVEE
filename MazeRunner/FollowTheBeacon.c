#include "Sensors.h"
#include "rmi-mr32.h"
#include <stdlib.h>
#include <string.h>

#include "FollowTheBeacon.h"

int turnedOnMyself=0;

int ftb_turningOnSelf(){
    return turnedOnMyself;
}

char* ftb_getName() {
    return "FollowTheBeacon";
}

void ftb_execute() {
    int constSpeed=70;
    double vel1 = constSpeed+abs(beaconDir*1/5);
    double vel2 = constSpeed-abs(beaconDir*4/5);

    if(turnedOnMyself && (obstValFront < 15 || obstValLeft < 10 || obstValRight < 10)){                 // i was turning on myself and stopped because beacon was in front of me but there were obstacles
        setVel2(50*turnedOnMyself, -turnedOnMyself*50);                 //turn back
    }else if(beaconDir<=90 && beaconDir>=-90){                                                               // beacon in front of me and no obstacles
        turnedOnMyself=0;
        if(beaconDir >= 0){
            setVel2(vel1, vel2);
        }else if(beaconDir < 0){
            setVel2( vel2, vel1 );
        }
    } else {                                                                                            // beacon behind me, dont care about obstacles
        if(beaconDir >= 0){
            turnedOnMyself=1;
        }else{
            turnedOnMyself=-1;
        }
        setVel2(50*turnedOnMyself, -turnedOnMyself*50);
    }
    
}

bool ftb_isPossible() {
    return (obstValFront > 25 && obstValLeft > 20 && obstValRight > 20 && visible) || 
            (visible && (beaconDir>90 || beaconDir<-90)) || 
            turnedOnMyself;
}

