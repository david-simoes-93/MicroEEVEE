#include "Sensors.h"
#include "rmi-mr32.h"
#include "FollowTheWall.h"

// Target is start position
double radiusConst = 200;
int lastPointX=0, lastPointY=0, targetX=0, targetY=0;

int vals [3][valsSize];
int modder = -1;
int left = 0, right = 0;
int pointer;

bool bVis=false, oldBVis=false, oldBVis2=false;

bool rotate_right=false;

void initAvg(){
	pointer=0;
	currServoPos = 0;    
	beaconDir = 0;
    setServoPos(currServoPos);
    right = POS_RIGHT;
    left = POS_LEFT;

	int c;

	for (c = 0 ; c < valsSize; c++){
		vals[OBST_SENSOR_LEFT][c]=0;
		vals[OBST_SENSOR_RIGHT][c]=0;
		vals[OBST_SENSOR_FRONT][c]=0;
	}

	addPoint(0,0);
}

void servoControl(){
    //printf("BeaconVisible=%d beaconDir=%d left=%d right=%d ServoPos=%d\n", beaconVisible, beaconDir, left, right, currServoPos);

    //led(3,visible);

    if(followPoints){
	    //printf("FOLLOW POINT\n"); 
    	getLastPoint();
    	targetX=lastPointX;
    	targetY=lastPointY;
	    if( checkPointsRadius() ){
	    	if(removePoint()){
				// update point
				printf("Found home!\n");
				setVel2(0,0);
				while (!startButton());
			}
	    }
	    //double myNorm, targetNorm;
	    double meToTargetVector [2];
		double myDirectionVector [2];
	    double myX=x, myY=y, myDir=t;	// 0.35 radians
		

		meToTargetVector[0]=(targetX-myX);
		meToTargetVector[1]=(targetY-myY);
		myDirectionVector[0]=cos(myDir);
		myDirectionVector[1]=sin(myDir);

		double radianBeaconDir = atan2(meToTargetVector[1], meToTargetVector[0]) - atan2(myDirectionVector[1], myDirectionVector[0]);

		beaconDir=-normalizeAngle(radianBeaconDir)*(180 / PI);
		int servoPos = beaconDir/6;

		printf("My position %2.1f,%2.1f;      Target Position %d,%d;         MyAngle %2.0f     AngleToNextPoint %d\n", 
		       myX, myY, targetX, targetY, (myDir*180/PI), beaconDir);   //NEGATIVO PRA ESQUERDA
		
		if(servoPos > POS_RIGHT)
		  setServoPos(POS_RIGHT);
		else if( servoPos < POS_LEFT )
		  setServoPos(POS_LEFT);
		else
		  setServoPos(servoPos);
	    
	    
    }else{
		bVis=readBeaconSens();

		if(bVis && !oldBVis){							// Started seeing beacon!
		    if(rotate_right){ 
		    	left = currServoPos; 
		    	//if(left<0 && right>=POS_RIGHT)
		    	//	right=left+15;
		    }
		    else { 
		    	right = currServoPos; 
		    	//if(right>0 && left<=POS_LEFT)
		    	//	left=right-15;
		    }
		    
		    if(currServoPos==POS_RIGHT || currServoPos==POS_LEFT){	// Got to the end of the line
		      rotate_right=!rotate_right;
		    }
		}
		else if(!bVis && oldBVis){						// Stopped seeing beacon!
		    if(rotate_right){ right = currServoPos-1; }
		    else { left = currServoPos+1; }

		    rotate_right=!rotate_right;
		} else if(currServoPos==POS_RIGHT || currServoPos==POS_LEFT){	// Got to the end of the line
		    if(currServoPos==POS_RIGHT){ right = POS_RIGHT+1; }
		    else { left = POS_LEFT-1; }

		    rotate_right=!rotate_right;
		}

		visible = bVis||oldBVis||oldBVis2;
		//led(2, visible);

		oldBVis2=oldBVis;
		oldBVis=bVis;

		beaconDir = (left + right)/2*6;
		
		//printf("Beacon at %d\n",beaconDir);

		modder = rotate_right ? 1 : -1;
		//currServoPos+=modder;

		currServoPos = visible ? currServoPos+modder*1 : currServoPos+modder*2;
		if(currServoPos>POS_RIGHT){ currServoPos=POS_RIGHT; }
		else if(currServoPos<POS_LEFT){ currServoPos=POS_LEFT; }

		setServoPos(currServoPos);
    }
    
}

int checkPointsRadius(){
	int j;

	if(radiusConst!=200)
		radiusConst+=2;

	for(j = 0; j < pointsListPointer; j++){
		if(isOnRadius( x,y, pointsList[0][j], pointsList[1][j], radiusConst)){
			
			printf("Was close to point %d %d, position [%d] out of %d\n",pointsList[0][j], pointsList[1][j], j+1, pointsListPointer);
			pointsListPointer=j+1;
			return 1;
		}
	}
	
	return 0;
}

int removePoint(){
	printf("Removed point %d %d!\n", pointsList[0][pointsListPointer-1], pointsList[1][pointsListPointer-1]);
	pointsListPointer--;
	if(pointsListPointer==0){
		if(radiusConst!=200){	// if point 0,0 was found with radius 150, end!
			return 1;
		}
		else					// if point 0,0 was found with radius 300, switch to radius 100
		{
			pointsListPointer++;
			radiusConst=151;
		}
	}
	//newPointMark();
	atBeacon();
	getLastPoint();
	return 0;
}

void markPoint(){
	if(!followPoints){
		getLastPoint();
		if(!isOnRadius(x, y, lastPointX, lastPointY, 100)){
			addPoint(x,y);
		}
	}
}

void getLastPoint(){
	lastPointX=pointsList[0][pointsListPointer-1];
	lastPointY=pointsList[1][pointsListPointer-1];

}

void addPoint(double x2, double y2){
	printf("Adding point %2.0f %2.0f\n",x2, y2);
	pointsList[0][pointsListPointer]=x2;
	pointsList[1][pointsListPointer]=y2;
	pointsListPointer++;
}

int isOnRadius( double x2, double y2, double nextX, double nextY, int radius){
	int i;
        double dist=0.0;
        for(i = 0; i < 2; i++){
            dist = sqrt( (pow((nextX - x2) , 2)) + (pow( (nextY - y2) , 2)) );
            //printf("dist= %f\n", dist);
            if(dist <= radius)
                return 1;
        }
            return 0;
}

double dot_product(double v[], double u[], int n)
{
    int i;
    double result = 0.0;
    for ( i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

void calculateAverage(){
	int temp=obstacleSensor(OBST_SENSOR_LEFT);
	if(temp<155) temp=155;
	vals[OBST_SENSOR_LEFT][pointer]=temp;

	temp=obstacleSensor(OBST_SENSOR_FRONT);
	if(temp<155) temp=155;
	vals[OBST_SENSOR_FRONT][pointer]=temp;

	temp=obstacleSensor(OBST_SENSOR_RIGHT);
	if(temp<155) temp=155;
	vals[OBST_SENSOR_RIGHT][pointer]=temp;

	pointer = (pointer+1) % valsSize;

	int i, j;
	int tempVals[3][valsSize];
	for (i = 0; i < valsSize; i++){
		tempVals[OBST_SENSOR_LEFT][i]=vals[OBST_SENSOR_LEFT][i];
		tempVals[OBST_SENSOR_FRONT][i]=vals[OBST_SENSOR_FRONT][i];
		tempVals[OBST_SENSOR_RIGHT][i]=vals[OBST_SENSOR_RIGHT][i];
	}

    for (i = 1; i < valsSize; i++) {
      	for (j = 0; j < valsSize - 1; j++) {
         	if (tempVals[OBST_SENSOR_LEFT][j] > tempVals[OBST_SENSOR_LEFT][j + 1]) {
            	temp = tempVals[OBST_SENSOR_LEFT][j];
            	tempVals[OBST_SENSOR_LEFT][j] = tempVals[OBST_SENSOR_LEFT][j + 1];
            	tempVals[OBST_SENSOR_LEFT][j + 1] = temp;
         	}

         	if (tempVals[OBST_SENSOR_FRONT][j] > tempVals[OBST_SENSOR_FRONT][j + 1]) {
            	temp = tempVals[OBST_SENSOR_FRONT][j];
            	tempVals[OBST_SENSOR_FRONT][j] = tempVals[OBST_SENSOR_FRONT][j + 1];
            	tempVals[OBST_SENSOR_FRONT][j + 1] = temp;
         	}

         	if (tempVals[OBST_SENSOR_RIGHT][j] > tempVals[OBST_SENSOR_RIGHT][j + 1]) {
            	temp = tempVals[OBST_SENSOR_RIGHT][j];
            	tempVals[OBST_SENSOR_RIGHT][j] = tempVals[OBST_SENSOR_RIGHT][j + 1];
            	tempVals[OBST_SENSOR_RIGHT][j + 1] = temp;
         	}
      	}
  	}

    //printf("--> %d %d %d %d %d %d %d - %d\n", tempVals[OBST_SENSOR_LEFT][0], tempVals[OBST_SENSOR_LEFT][1], tempVals[OBST_SENSOR_LEFT][2], 
    //	tempVals[OBST_SENSOR_LEFT][3], tempVals[OBST_SENSOR_LEFT][4], tempVals[OBST_SENSOR_LEFT][5], tempVals[OBST_SENSOR_LEFT][6], tempVals[OBST_SENSOR_LEFT][valsSize/2]);

	obstValFront = (6200 / (tempVals[OBST_SENSOR_FRONT][valsSize/2]-80)) - 7;
    obstValLeft = (6200 / (tempVals[OBST_SENSOR_LEFT][valsSize/2]-80)) - 9;
    obstValRight = (6200 / (tempVals[OBST_SENSOR_RIGHT][valsSize/2]-80)) - 8;

    if(obstValFront>80) obstValFront = 80;
    else if (obstValFront<0) obstValFront = 0;

    if(obstValLeft>80) obstValLeft = 80;
    else if (obstValLeft<0) obstValLeft = 0;

    if(obstValRight>80) obstValRight = 80;
    else if (obstValRight<0) obstValRight = 0;

}

void rotateRel_naive(double deltaAngle) {
    double x, y, t;
    double targetAngle;
    double error;
    int cmdVel;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    error = normalizeAngle(targetAngle - t);
    if (error < 0)
        cmdVel = -30;
    else
        cmdVel = 30;

    setVel2(-cmdVel, cmdVel);

    do {
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);
    } while (fabs(error) > 0.01 && (error * cmdVel) > 0);
    setVel2(0, 0);
}

