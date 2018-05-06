#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "Sensors.h"

bool update_map_and_recalculate();

//next_cell = [1,2]
bool update_map_and_recalculate(int* next_cell){
	//if updated map influences path return true
	return false;
}

int count_beacon;



int main(void)
{
	/*init*/
	initPIC32();  
    closedLoopControl( true );
    setVel2(0, 0);
    setRobotPos(0,0,0);

    /*wait until press start*/
    printf("Press start to continue\n");
    while(!startButton());

    enableObstSens();
    // Fill in "analogSensors" structure
    int i;
    for (i = 0; i < 7; ++i)
    {
        readAnalogSensors();
        //groundSensor = readLineSensors(0);
    }

    /*variables*/
    count_beacon = 0;
    int count_ticks; 
    count_ticks = 0;


    /*-------------------*/
    //update_map(float my_x, float my_y, float left_sensor, float front_sensor, float right_sensor, float compass)
    while(!stopButton()){
    	count_ticks++;
    	waitTick20ms(); 
    	switch(count_ticks){
    		case 1 :
    			// Fill in "analogSensors" structure
	            readAnalogSensors(); 
	            
	            // Read ground sensor
	            groundSensor = readLineSensors(70); 
		    	
		    	gndVals[c] = groundSensor;	c=(c+1)%5; //buffer 
            
    		break;

    		case 2;
    			count_ticks = 0;
    		break
    	}

    	/* Track robot position and orientation */
        getRobotPos(&x, &y, &t);

    }


}




//##########################################################################################################
//############################# JAVA ############################

// added a new beacon line, mark a new beacon point, try and find a path there
public Cell getDirectionTarget(Cell curr, float dir, Cell actualCurr) {
	boolean contains = false;
	// check if we have any beacon line pointing at the beacon and starting close to current position
	for ( Line l :	beaconLines) {
		if ( getDistanceBetweenCells(l.a, curr) < 3) {
			contains = true;
			break;
		}
	}

	int dist = 5 * 3;
	int newX = (int) (curr.x + cos(dir) * dist);
	int newY = (int) (curr.y - sin(dir) * dist);
	Cell endLine = new Cell(newX, newY	);

	// if we have no such line, add a new line and mark the point it intersects with other lines
	if (!contains) {
		Line newLine = new
		Line(curr, endLine	);
		beaconLines.add(new	Line(curr, endLine	));

		float x3 = newLine.a.x;
		float y3 = newLine.a.y;
		float x4 = newLine.b.x;
		float y4 = newLine.b.y;

		for (	int i = 0;	i < beaconLines.size() - 1; i++) {
			float x1 = beaconLines.get(i).a.x;
			float y1 = beaconLines.get(i).a.y;
			float x2 = beaconLines.get(i).b.x;
			float y2 = beaconLines.get(i).b.y;

			//if parallel with other line, skip it
			if ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) == 0) {
				continue;
			}

			//intersect with other line
			float Px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))
			           / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
			float Py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))
			           / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

			//if point between both lines' end points, keep it
			if (Px > Math.min(x3, x4) && Px < Math.max(x3, x4) && Py > Math.min(y3, y4) && Py < Math.max(y3, y4	)) {
				if (gui != null) {
					gui.paintPoint((int) Px, (int) Py, Color.RED);
				}
				setUniqueBeaconPoint((	int) Px, (int) Py);
			}
		}
	}

	if (!beaconPoints.isEmpty()	) {
		return	getWayToBeacon(actualCurr);
	} 
	else {
		Cell tar = new	Cell(newX, newY	);
		tar = findClosestFreeCell(tar, 2);
		return	StarSearchExplorerZone(actualCurr, tar,	2, 3);
	}
}