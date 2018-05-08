#include "Sensors.h"
#include "rmi-mr32.h"
#include "StopAtBeacon.h"
#include "FollowTheWall.h"

int firstTime=0;    // newPoint=0, 

char* sab_getName() {
    return "StopAtBeacon";
}

void sab_execute() {
  if(!followPoints){
    followPoints = 1;
    firstTime=1;
    visible=1;

    setVel2(0,0);
    
    printf("Beacon found at %2.0f %2.0f\n", x, y);
    while (!startButton());
  }else {
    if(beaconDir > 0) {
        setVel2(30, -30);
    } else {
        setVel2(-30, 30);
    }
  }
}

bool sab_isPossible() {
    int gnd = 1, i;
    for(i=0; i < 5; i++)
        if(gndVals[i] == 0)
            gnd = 0;

    if(firstTime && (beaconDir<-20 || beaconDir>20))
        return true;
    
    if(firstTime){
      atBeacon();
      firstTime=0;
    }
    
    if(followPoints)
      return false;
    
    return gnd;
}

