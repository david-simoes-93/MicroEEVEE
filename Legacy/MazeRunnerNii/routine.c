#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pretty.c"


void rotateRel(int maxVel, double deltaAngle);

//speeds
#define SPEED 30
#define CONTROL_SPEED 20

//ground sensor positions in array
#define MOST_LEFT 0
#define MOST_RIGHT 4
#define LEFT 1
#define RIGHT 3
#define CENTER 2


//states
#define FOLLOW_LINE 100
#define FOLLOW_WALL 101
#define OBSTACLE 102
#define NO_WALL 103
#define START_LINE 104
#define CONTROL_LINE 105
#define DEAD_ANGLE 106
//other
#define G_BUFFER_SIZE 3
#define G_SENSOR_NUM 5

#define COLOR_LINE 1 // TODO (value of found line)

void printSensors();
void followWalls(int speed);
void followLine(int speed, int state);
void readGroundSensors();
bool equals(int a[], int b[], int size);
bool foundLine();
int eval_ground_sensors();
int check_Obstacle();
void readGroundSensors();
void perform_blind(int speed);

int groundcounter_ALL = 0;
int groundcounter_CENTER = 0;
int nowallcounter = 0;
bool looking_for_wall = false;
int laps;
int groundSensor;
int ground_buffer[G_SENSOR_NUM];
int ground_history[G_SENSOR_NUM];
double dist_to_start[2]; //first and second lap
double last_turn_pos[2]; //(x,y)
bool dist_blind; //false - not yet | true - not in the blind spot
double xx;//distance to walk blind
double dead_angle_Xpos, dead_angle_Ypos;
int obst_history;
int sensor_dist_history;                      
int main(void)
{
    
    initPIC32();
    
    closedLoopControl( true );
    setVel2(0, 0);
    setRobotPos(0,0,0);
    printHeader("HELLO", CLR_GREEN);
    printf("RMI-wall follower, robot %d\n\n\n", ROBOT);

    printf("Press start to continue\n");
    while(!startButton());
    enableObstSens();

    int currentState = FOLLOW_WALL;
    int obstacleState = 0;
    double x, y, t;
    bool inside_control_area = false;
    dist_blind = true;
    obst_history = OBSTACLE;
    int speed = SPEED;
    laps = -1;

    // Fill in "analogSensors" structure
    int i;
    for (i = 0; i < 7; ++i)
    {
        readAnalogSensors();
        //groundSensor = readLineSensors(0);
    }


    
    
    
    while(!stopButton())
    {
        //waitTick40ms(); 

        //check position
        if (laps >1){ //last lap
           
            getRobotPos(&x, &y, &t);
            double dist_avg = (dist_to_start[0]+ dist_to_start[1])/2;
 
            double d;
            if(  (d = sqrt(pow((last_turn_pos[0]-x),2)+ pow((last_turn_pos[1]-y),2)) ) >= dist_avg){
                //laps++;
                disableObstSens();
                setVel2(0,0);
                printf("-----Finished!----- \n");
                return 0;
            }
        }

        //read Sensors
        readAnalogSensors();

        readGroundSensors();
        
        int ground_state = eval_ground_sensors();

        if(ground_state == START_LINE && !equals(ground_buffer, ground_history, 5)){
            laps++;
            if(laps>0){
                getRobotPos(&x, &y, &t);
                double d = sqrt(pow((last_turn_pos[0] -x),2)+ pow((last_turn_pos[1] -y),2));  
                dist_to_start[laps-1] = d; 
            }
            //reset position
            setRobotPos(0.0 ,0.0 ,0.0 );
            //rotate 180º
            if(laps==2)
                rotateRel(speed, PI);
        }
        else if(ground_state == CONTROL_LINE && !equals(ground_buffer, ground_history, 5) ){
            if(!inside_control_area){
                speed = CONTROL_SPEED;
                inside_control_area = true;
                printf("Changing speed to CONTROL_SPEED\n");
            }
            else{
                speed = SPEED;
                inside_control_area = false;
                printf("Changing speed to NORMAL_SPEED\n");
            }
        }

        //update history
        for (i= 0; i < G_BUFFER_SIZE; ++i)
        {
            ground_history[i] = ground_buffer[i];
        }
        obst_history = obstacleState;
        obstacleState = check_Obstacle();
        //printHeader("INFO", CLR_CYAN);
        //printf("obst :%d , dist_blind: %d\n", obstacleState , dist_blind);

    
        if( foundLine() && obstacleState == NO_WALL){
            currentState = FOLLOW_LINE;
            printHeader("LINE", CLR_RED);
            printf("Following a line! Ground State: %d\n", ground_state);
        }
        else if(obstacleState == NO_WALL && !dist_blind ){
            //currentState = DEAD_ANGLE;
            currentState = FOLLOW_WALL;
            printHeader("DEBUG", CLR_GREEN);
            printf("Entered DEAD_ANGLE\n");
        }
        else{
            currentState = FOLLOW_WALL;
            //printHeader("STATE", CLR_CYAN);
            //printf("Entered FOLLOW_WALL\n");
        }
        

        
        //printf("Changing vel to: %d - %d\n", left_wheel_speed, right_wheel_speed);                     
        

        switch(currentState){
            case FOLLOW_LINE:
                followLine(speed, ground_state);
            break;

            case FOLLOW_WALL :
                followWalls(speed);
            break;
            case DEAD_ANGLE :
                perform_blind(speed);
            break;
            
        }

        //printf("Obst_right=%03d", analogSensors.obstSensRight );
        //printSensors();
        
    }

    disableObstSens();
    
    return 0;
}


void printSensors(){
    printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d, Bat_voltage=%03d, Ground_sens=", 
                    analogSensors.obstSensLeft,
                    analogSensors.obstSensFront, 
                    analogSensors.obstSensRight, 
                    analogSensors.batteryVoltage);
    printInt(groundSensor, 2 | 5 << 16);    // System call
    printf("\n");
}


#define KP_ROT  40
#define KI_ROT  5

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

#define KP_WALL 5 

void followWalls(int speed){

    int sensor;

    if(laps < 2){
        sensor = analogSensors.obstSensRight;
    }
    else{
        sensor = analogSensors.obstSensLeft;
    }
    sensor_dist_history = sensor;
    double error = 0;
    int cmdVel;


    if (analogSensors.obstSensFront < 25)

        error -= 5.0; //acertar este valor
    //testing values

    if (sensor > 50) error += 4.0;
    else if (sensor > 35) error += 2.0;   
    else if (sensor > 30) error += 1.0;
    else if (sensor < 20) error -= 3.0;
    else if (sensor < 25) error -= 1.0;

    //error += 0.4* sensor -11 ;

    if(error > 3 ){
         double x, y, t;
         getRobotPos(&x, &y, &t);
         last_turn_pos[0] = x;
         last_turn_pos[1] = y;

    }

    if(laps >= 2){ //Left wall
        error = -error;
    }

    cmdVel = (int)((KP_WALL * error) ) ;
    setVel2(speed + cmdVel, speed - cmdVel);

}

void perform_blind(int speed){
    int sensor;
    double error = 0.0;
    int cmdVel;

    /*if(analogSensors.obstSensFront <20 )
        error = -5.0;*/

    double x, y, t;
    getRobotPos(&x, &y, &t);
    
    double d;
    if(  (d = sqrt(pow((dead_angle_Xpos-x),2)+ pow((dead_angle_Ypos-y),2)) ) >= xx) {
        dist_blind = true;
        printHeader("DEBUG", CLR_YELLOW);
        printf("X : %f\n", x);
    }
    cmdVel = (int)((KP_WALL * error) ) ;
    setVel2(speed + cmdVel, speed - cmdVel);

}

int eval_ground_sensors(){
    if (ground_buffer[LEFT] && ground_buffer[CENTER] && ground_buffer[RIGHT] && ground_buffer[MOST_LEFT] && ground_buffer[MOST_RIGHT] && (check_Obstacle() == OBSTACLE)) {
        groundcounter_ALL++;
        groundcounter_CENTER++;
        //printHeader("GROUND", CLR_YELLOW);
        //printf("Detected %d consecutive positive readings - ALL Sensors\n", groundcounter_ALL);
    }
    else if (ground_buffer[LEFT] && ground_buffer[CENTER] && ground_buffer[RIGHT] && (check_Obstacle() == OBSTACLE)) {
        groundcounter_CENTER++;
        //printHeader("GROUND", CLR_YELLOW);
        //printf("Detected %d consecutive positive readings - Only center sensors\n", groundcounter_CENTER);
    } else {
        if (groundcounter_ALL >= 7) {
            printHeader("LINE", CLR_GREEN);
            printf("Detected a speed control line.\n"   );
            groundcounter_ALL = 0;
            groundcounter_CENTER = 0;

            return CONTROL_LINE;
        }
        else if (groundcounter_ALL < 7 && groundcounter_CENTER > 3) {
            printHeader("LINE", CLR_GREEN);
            printf("Detected a start line.\n");

            groundcounter_ALL = 0;
            groundcounter_CENTER = 0;

            return START_LINE;
        }
        // Clean-up

    }

    if( ground_buffer[MOST_LEFT] && !ground_buffer[LEFT] && !ground_buffer[CENTER] && !ground_buffer[RIGHT] && ground_buffer[MOST_RIGHT]){
        //TODO
        if(laps < 2)
            return MOST_LEFT;
        else
            return MOST_RIGHT;
    }
    else if( ground_buffer[MOST_LEFT] && !ground_buffer[LEFT] && !ground_buffer[CENTER] && !ground_buffer[RIGHT] && !ground_buffer[MOST_RIGHT])
        return MOST_LEFT;
    else if( ground_buffer[LEFT] && !ground_buffer[CENTER] && !ground_buffer[RIGHT] && !ground_buffer[MOST_RIGHT])
        return LEFT;
    else if( !ground_buffer[MOST_LEFT] && !ground_buffer[LEFT] && !ground_buffer[CENTER] && !ground_buffer[RIGHT] && ground_buffer[MOST_RIGHT])
        return MOST_RIGHT;
    else if( !ground_buffer[MOST_LEFT] && !ground_buffer[LEFT] && !ground_buffer[CENTER] && ground_buffer[RIGHT] )
        return RIGHT;
    else if(ground_buffer[CENTER] )
        return CENTER;

    return -1;

}

#define KP_LINE 10

void followLine(int speed, int sit){
    double error = 0;
    int cmdVel;
   
    //testing values
    switch(sit){
        case MOST_LEFT:
            //turn left
            error = -5.0;
        break;

        case LEFT:
            error = -4.0;
        break;

        case CENTER:
            error = -2.0;
        break;

        case RIGHT:
            error = -1.0;
        break;
        default:
            //printHeader("LINE", CLR_RED);
            //printf("Following a line! Default case!\n");
        break;
    }

    if(laps >= 2){ //Left wall
        error = -error;
    }

    printHeader("LINE", CLR_RED);
    printf("Following a line! Error: %f\n", error);


    cmdVel = (int)((KP_LINE * error) ) ;
    setVel2(speed + cmdVel, speed - cmdVel);
}

int groundSensorAverage(int index) {
    int number_of_zeroes = 0;
    int i;
    for(i = 0; i < G_BUFFER_SIZE; i++) {
        if (!ground_buffer[index]) number_of_zeroes++;
    }

    return number_of_zeroes > G_BUFFER_SIZE / 2 ? 0 : 1;
}

void readGroundSensors(){
    //clean structer
    ground_buffer[MOST_LEFT] = 0;
    ground_buffer[LEFT] = 0;
    ground_buffer[CENTER] = 0;
    ground_buffer[RIGHT] = 0;
    ground_buffer[MOST_RIGHT] = 0;

    //fill buffer for ground sensors
    int i;
    for ( i = 0; i < G_BUFFER_SIZE; ++i) // TODO: Check sensor sample rate
    {   
        groundSensor = readLineSensors(0);  // Read ground sensor
        
        ground_buffer[MOST_LEFT] += ((groundSensor & 0x00000010) >> 4);
        ground_buffer[LEFT] += ((groundSensor & 0x00000008) >> 3);
        ground_buffer[CENTER] += ((groundSensor & 0x00000004) >> 2);
        ground_buffer[RIGHT] += ((groundSensor & 0x00000002) >> 1);
        ground_buffer[MOST_RIGHT] += (groundSensor & 0x00000001);
    }

    ground_buffer[MOST_LEFT] = groundSensorAverage(MOST_LEFT);
    ground_buffer[LEFT] = groundSensorAverage(LEFT);
    ground_buffer[CENTER] = groundSensorAverage(CENTER);
    ground_buffer[RIGHT] = groundSensorAverage(RIGHT);
    ground_buffer[MOST_RIGHT] = groundSensorAverage(MOST_RIGHT);

    /*
    ground_buffer[MOST_LEFT] = ground_buffer[MOST_LEFT] / G_BUFFER_SIZE ;
    ground_buffer[LEFT] = ground_buffer[LEFT] / G_BUFFER_SIZE ;
    ground_buffer[CENTER] = ground_buffer[CENTER]/ G_BUFFER_SIZE ;
    ground_buffer[RIGHT] = ground_buffer[RIGHT] / G_BUFFER_SIZE ;
    ground_buffer[MOST_RIGHT] = ground_buffer[MOST_RIGHT]/ G_BUFFER_SIZE ;
    */

    
}

/**
 * @brief      evaluates if a and b are equal 
 *
 * @param      a     { parameter_description }
 * @param      b     { parameter_description }
 * @param[in]  size  The size
 *
 * @return     { 1 - all equal | 0 - one or more diffent }
 */
bool equals(int a[] , int b[] , int size){
    bool equal = true;
    int k;
    for (k = 0; k < size; ++k)
    {
        if(a[k] != b[k])
            equal = false;
    }
    return equal;
}

bool foundLine(){
    int k;
    for (k = 0; k < G_SENSOR_NUM; ++k)
    {
        if(ground_buffer[k] == COLOR_LINE)
            return true;
    }
    return false;
}

int check_Obstacle(){
    int sensor;

    if(laps < 2){
        sensor = analogSensors.obstSensRight;
    }
    else{
        sensor = analogSensors.obstSensLeft;
    }

    if(sensor < 40 || analogSensors.obstSensFront < 30 ){
        
        return OBSTACLE;
    }

    // NO_WALL (first time)
    if(obst_history == OBSTACLE){
        xx = ((1/sqrt(2) ) * sensor_dist_history - 5.5 )*10 + 40 ;//mm 
        dist_blind = false;
        double t;
        getRobotPos(&dead_angle_Xpos, &dead_angle_Ypos, &t);

        /*pos[0] = cos(t) * xx;
        if(abs(pos[0]) < 2) pos[0] = 0;
        pos[1] = sin(t) * xx;
        if(abs(pos[1])< 2) pos[1] = 0;
*/
        printHeader("DEBUG", CLR_CYAN);
        printf("xx: %f, now pos(%f , %f)\n", xx, dead_angle_Xpos, dead_angle_Ypos);
    }
   
    return NO_WALL;
}
