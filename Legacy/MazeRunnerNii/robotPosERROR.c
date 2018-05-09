#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h> //file

void rotateRel_basic(int speed, double deltaAngle);
void rotateRel(int maxVel, double deltaAngle);
void onlyX();
void errorXY();

#define  STEP 100 //mm
#define VEL 20 
                  
int main(void)
{
    int groundSensor;

    initPIC32();
    closedLoopControl( true );
    setVel2(0, 0);

    printf("RMI-script to find the cumulative error in robotPos, robot %d\n\n\n", ROBOT);
    onlyX();
    return 0;
}

void errorXY(){
    //output file
    FILE *fp;

    fp = fopen("/output.txt", "w+");
    fprintf(fp, "robotX realX robotY realY\n");
    //fputs("This is testing for fputs...\n", fp);
    fclose(fp);
    
    double xx, yy, tt;
    int tmp = 0;
    double angle = -M_PI/4;
    printf("Instructions:\nPress stop to finish\n");
    while(!stopButton())
    {
        tmp++;
        printf("Press start to continue\n");
        while(!startButton());
        rotateRel(100, angle ); //45º
        do
        {
            setVel2(VEL, VEL);

            getRobotPos(&xx, &yy, &tt);
        } while( (xx%STEP ) == 0 || (yy%STEP ) == 0  );
        if ( tmp == 2){
            angle = angle*(-1);
            tmp = 0;
        }
        printf("Enter real measurement x y (mm) \n");
        int x, y;
        scanf("%d %d", &x, %y);
        

        fprintf(fp, "%d %d %d %d\n", xx , x, yy, y);
        
        setVel2(0, 0);
    }

    fclose(fp);
    return;
}



void onlyX(){
    //output file
    FILE *fp;

    fp = fopen("/output.txt", "w+");
    fprintf(fp, "robot real\n");
    //fputs("This is testing for fputs...\n", fp);
    fclose(fp);
 
    double xx, yy, tt;
    printf("Instructions:\nPress stop to finish\n");
    while(!stopButton())
    {
        printf("Press start to continue\n");
        while(!startButton());
        
        do
        {
            setVel2(VEL, VEL);

            getRobotPos(&xx, &yy, &tt);
        } while( (xx%STEP ) == 0  );
        printf("Enter real measurement (mm) \n");
        int i;
        scanf("%d", &i);
        fprintf(fp, "%d %d\n", xx , i);

        setVel2(0, 0);
    }

    fclose(fp);
    return;


}

#define KP_ROT	40
#define KI_ROT	5

// deltaAngle in radians
void rotateRel(int maxVel, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
    int cmdVel;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    do
    {
        waitTick40ms();
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);

        integral += error;
        integral = integral > PI / 2 ? PI / 2: integral;
        integral = integral < -PI / 2 ? -PI / 2: integral;

        cmdVel = (int)((KP_ROT * error) + (integral * KI_ROT));
        cmdVel = cmdVel > maxVel ? maxVel : cmdVel;
        cmdVel = cmdVel < -maxVel ? -maxVel : cmdVel;

        setVel2(-cmdVel, cmdVel);
    } while (fabs(error) > 0.01);
    setVel2(0, 0);
}


void rotateRel_basic(int speed, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    int cmdVel, errorSignOri;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    error = normalizeAngle(targetAngle - t);
    errorSignOri = error < 0 ? -1 : 1;

    cmdVel = error < 0 ? -speed : speed;
    setVel2(-cmdVel, cmdVel);

    do
    {
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);
    } while (fabs(error) > 0.01 && errorSignOri * error > 0);
    setVel2(0, 0);
}

