#include "rmi-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pretty.c"


void rotateRel(int maxVel, double deltaAngle);

//speeds
#define SPEED 40
#define CONTROL_SPEED 20

//ground sensor positions in array
#define MOST_LEFT 0
#define MOST_RIGHT 4
#define LEFT 1
#define RIGHT 3
#define CENTER 2

#define ALL 107

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
#define STOPPING_DISTANCE 40
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
void updateSensors();
void rotateRel_basic(int speed, double deltaAngle);
int groundcounter_ALL = 0;
int groundcounter_CENTER = 0;
int nowallcounter = 0;
int last_ground_state = 0;
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
int ground_state;
int obstacleState;
int speed;
int empty_reading_counter;
double total_error;
double error_all_laps=0;
double avg_x = 0;
bool inside_control_area;
bool found_wall_after_dead = true;
bool ready_to_stop = false;
bool now= false;
//count turns to detect the distance do start line correctly
int count_turns = 0;
double avg_start_dist = 0;
bool curve = false;

int avg_curves[2];
double pos[2];

int main(void)
{
    total_error = 0;
    empty_reading_counter = 0;
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
    int NS = currentState;
    obstacleState = 0;
    
    inside_control_area = false;
    dist_blind = true;
    obst_history = OBSTACLE;
    speed = SPEED;
    laps = -1; //TODO change to -1
    curve = false;
    // Fill in "analogSensors" structure
    int i;
    for (i = 0; i < 7; ++i)
    {
        readAnalogSensors();
        //groundSensor = readLineSensors(0);
    }

    updateSensors();
    printHeader("DEBUG", CLR_GREEN);
    printf("Entered FOLLOW_WALL\n");
    while(!stopButton()){
        if(laps == 2) {
            if (curve) {
                double x,y,t;
                getRobotPos(&x, &y, &t);

                printHeader("DISTANCE", CLR_RED);
                printf("AVG_X: %f, X: %f\n", avg_start_dist/2, x );
                if( ( sqrt(pow((0-x),2)+ pow((0-y),2)) ) >= avg_start_dist/2   ) {
                    setVel2(0,0);
                    setVel2(0,0);
                    setVel2(0,0);
                    disableObstSens();
                    printHeader("ENUF", CLR_GREEN);
                    printf("Bro we're DONE!\n");
                    while(1);
                }
            }
            /*else{
                double x,y,t;
                getRobotPos(&x, &y, &t);

                printHeader("DISTANCE", CLR_RED);
                printf("AVG_X: %d, X: %f\n", 200, x );
                if( ( sqrt(pow((x),2)+ pow((y),2)) ) < 200   ) {
                    setVel2(0,0);
                    setVel2(0,0);
                    setVel2(0,0);
                    disableObstSens();
                    printHeader("ENUF", CLR_GREEN);
                    printf("Bro we're DONE!\n");
                    while(1);
                }
                
                
            }*/
            if (!ready_to_stop && total_error > error_all_laps/2/2) {
                ready_to_stop = true;
            }
        }
        
        
        //state machine
        currentState = NS;



        switch(currentState){
            case FOLLOW_LINE:
                leds(0x001);
                followLine(speed, ground_state);
                NS = FOLLOW_LINE;

                updateSensors();

                int s; 
                if(laps<2) s = analogSensors.obstSensRight;
                else s= analogSensors.obstSensLeft;

                if(analogSensors.obstSensFront < 25 || s < 25){
                    double angle;
                    if (laps<2)
                    {
                        angle = PI/4;
                    }
                    else angle = -PI/4;
                    rotateRel_basic(speed , angle);
                    //walk for a little bit
                    obst_history = NO_WALL;
                    NS = DEAD_ANGLE;
                    xx = 20 ;//mm 
                    dist_blind = false;
                    double t;
                    getRobotPos(&dead_angle_Xpos, &dead_angle_Ypos, &t);
                   // printf("front %d , s %d\n", analogSensors.obstSensFront, s );
                   // NS = FOLLOW_WALL;
                    printHeader("DEBUG", CLR_GREEN);
                    printf("Entered DEAD_ANGLE\n");

                }

            break;

            case FOLLOW_WALL :
                leds(0x002);
                //printf("obstacleState now %d , ", obstacleState);
                followWalls(speed);
                NS = FOLLOW_WALL;

                updateSensors();
                
                obstacleState = check_Obstacle();
                printHeader("SENSORS", CLR_GREEN);
                //printf("front %d , left %d, right %d \n", analogSensors.obstSensFront, analogSensors.obstSensLeft, analogSensors.obstSensRight);
                if(obstacleState == NO_WALL && !dist_blind ){

                    //printf(" obst_history %d , obstacleState %d , found_wall_after_dead %d", obst_history, obstacleState, found_wall_after_dead);
                    printHeader("DEBUG", CLR_GREEN);
                    printf("Entered DEAD_ANGLE\n");
                    count_turns++;
                    if(laps < 2 && count_turns == 1){
                        avg_start_dist += dead_angle_Xpos +  xx;
                    }
                        


                    printHeader("DEBUG", CLR_CYAN);
                    printf("xx: %f, now pos(%f , %f)\n", xx, dead_angle_Xpos, dead_angle_Ypos);
                    NS = DEAD_ANGLE;
                }
                else if(obstacleState == NO_WALL && foundLine() ){
                    NS = FOLLOW_LINE;
                    printHeader("DEBUG", CLR_GREEN);
                    printf("Entered FOLLOW_LINE\n");
                }


            break;
            case DEAD_ANGLE :
                
                leds(0x004);
                perform_blind(speed);
                NS = DEAD_ANGLE;
                
                // printf("Obst_left=%03d, Obst_center=%03d, Obst_right=%03d\n", 
                //     analogSensors.obstSensLeft,
                //     analogSensors.obstSensFront, 
                //     analogSensors.obstSensRight);
                updateSensors();

                //arranjar o found line para ver pelos menos 3 pintados
                if(foundLine() ){//&& obstacleState == NO_WALL){
                    NS = FOLLOW_LINE;
                    printHeader("DEBUG", CLR_GREEN);
                    printf("Entered FOLLOW_LINE\n");
                    // printHeader("LINE", CLR_RED);
                    // printf("Following a line! Ground State: %d\n", ground_state);
                }
                else if( dist_blind ){
                    NS = FOLLOW_WALL;
                    printHeader("DEBUG", CLR_GREEN);
                    printf("Entered FOLLOW_WALL\n");
                    if(laps == 2 && (avg_curves[0] == avg_curves[1]) && count_turns == (avg_curves[0]+avg_curves[1])/2){
                        curve = true;
                        setRobotPos(0,0,0);
                        // double x, y, t;
                        // getRobotPos(&x, &y, &t);
                        // pos[0] = x;
                        // pos[1] = y;
                    }

                }

            break;
            
        }

        
    }

    disableObstSens();
    
    return 0;
}

void updateSensors(){
    double x, y, t;
    //check position
        if (laps >2){ //last lap
           
            getRobotPos(&x, &y, &t);
            double dist_avg = (dist_to_start[0]+ dist_to_start[1])/2;
 
            double d;
            if( x== 0 && (y < 5 && y > -5) ){//d= sqrt(pow((last_turn_pos[0]-x),2)+ pow((last_turn_pos[1]-y),2)) ) >= dist_avg){
                //laps++;
                //disableObstSens();
                //setVel2(0,0);
                //setVel2(0,0);
                //setVel2(0,0);
                //while(1) ;
                return;
            }
        }

        //read Sensors
        readAnalogSensors();

        readGroundSensors();
        
        ground_state = eval_ground_sensors();

        if(ground_state == START_LINE && last_ground_state != START_LINE/*!equals(ground_buffer, ground_history, 5) */){
            laps++;
            if(laps>0){
                getRobotPos(&x, &y, &t);
                double d = sqrt(pow((last_turn_pos[0] -x),2)+ pow((last_turn_pos[1] -y),2));  
                dist_to_start[laps-1] = d; 
            }
            //reset position
            //setRobotPos(0,0,0);
            //rotate 180º
            if(laps==2)
                rotateRel(speed, PI);
        }
        else if(ground_state == CONTROL_LINE && last_ground_state != CONTROL_LINE /*!equals(ground_buffer, ground_history, 5)*/ ){
            last_ground_state = CONTROL_LINE;
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
        int i;
        //update history
        for (i= 0; i < G_BUFFER_SIZE; ++i)
        {
            ground_history[i] = ground_buffer[i];
        }
        obst_history = obstacleState;

        obstacleState = check_Obstacle();
        

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
    //setVel2(0, 0);
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



    if (analogSensors.obstSensFront < 25 && sensor < 40){
        error -= 10.0; //acertar este valor
        if(!now){
            count_turns++;
            now = true;
        }
            
        //return;
    }else{
        now = false;
        
        //testing values
        
        
        //error += 0.0000464749 *  pow(sensor, 3) - 0.0115687 * pow(sensor,2) + 0.875928 *sensor - 16.5849 ;//(cubic)
        if (sensor > 40) error += 5.0; //TODO 4.0
        else if (sensor > 35) error += 2.0;   
        else if (sensor > 30) error += 1.0;

        else if (sensor < 25) error -= 1.0;
        //else if (sensor < 20) error -= 2.0;
        //error = 0.3* sensor - 8.2 ;
    }

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

    total_error += fabs(error);
    cmdVel = (int)((KP_WALL * error) ) ;
    setVel2(speed + cmdVel, speed - cmdVel);


}

void perform_blind(int speed){
    found_wall_after_dead = false;
    double error = 0.0;
    int cmdVel;

    if(analogSensors.obstSensFront < 20 ){
        dist_blind = true;
        setVel2(0, 0);
        return;
    }
       

    double x, y, t;
    getRobotPos(&x, &y, &t);
    
    double d;
    if(  (d = sqrt(pow((dead_angle_Xpos-x),2)+ pow((dead_angle_Ypos-y),2)) ) >= xx) {
        dist_blind = true;
        printHeader("DEBUG", CLR_YELLOW);
        printf("X : %f\n", x);

        
    }
    cmdVel = (int)((KP_WALL * error) ) ;
    setVel2(speed, speed);

}

int eval_ground_sensors(){
    if (ground_buffer[LEFT] && ground_buffer[CENTER] && ground_buffer[RIGHT] && ground_buffer[MOST_LEFT] && ground_buffer[MOST_RIGHT] && (check_Obstacle() == OBSTACLE)) {
        groundcounter_ALL++;
        groundcounter_CENTER++;
        printHeader("LINE", CLR_YELLOW);
        double x, y, t;
        getRobotPos(&x, &y, &t);
        printf("Detected ALL ground readings %d times. x: %f y: %f, angulo: %f\n", groundcounter_ALL, x, y, t);
        //printHeader("GROUND", CLR_YELLOW);
        //printf("Detected %d consecutive positive readings - ALL Sensors\n", groundcounter_ALL);
    }
    else if (ground_buffer[LEFT] && ground_buffer[CENTER] && ground_buffer[RIGHT] && (check_Obstacle() == OBSTACLE)) {
        groundcounter_CENTER++;
        //printHeader("GROUND", CLR_YELLOW);
        //printf("Detected %d consecutive positive readings - Only center sensors\n", groundcounter_CENTER);
    } else {
        if (empty_reading_counter++ > 10) {
            empty_reading_counter = 0;
            if (groundcounter_ALL >= 4) {
                printHeader("LINE", CLR_GREEN);
                printf("Detected a speed control line.\n"   );
                groundcounter_ALL = 0;
                groundcounter_CENTER = 0;
                return CONTROL_LINE;
            }
            else {
                double x, y, t;
                getRobotPos(&x, &y, &t);
                if (groundcounter_ALL < 4 && groundcounter_ALL > 0 && groundcounter_CENTER > 1 && !inside_control_area /*&& (t < 0.5 && t > -0.5) */) {
                    printHeader("LINE", CLR_GREEN);
                    printInt(groundSensor, 2 | 5 << 16);
                    printf(" Detected a start line. TOTAL_ERROR: %f\n", total_error);
                    printf("count_turns %d\n",count_turns );
                    if (laps > -1)
                        avg_curves[laps] = count_turns;
                    if(laps <2) curve = false;//to detect the start line very closely
                    count_turns = 0;

                    if(laps != -1) {
                        avg_x += x;
                        error_all_laps += total_error;
                    }
                    if(laps != 2) {
                        total_error = 0;
                        setRobotPos(0,0,0);
                    }
                    groundcounter_ALL = 0;
                    groundcounter_CENTER = 0;
                    
                    return START_LINE;
                }
            }
        } 
        last_ground_state = 0;
        // Clean-up

    }


    if (ground_buffer[LEFT] && ground_buffer[CENTER] && ground_buffer[RIGHT] && ground_buffer[MOST_LEFT] && ground_buffer[MOST_RIGHT])
        return ALL;
    else if( ground_buffer[MOST_LEFT] && !ground_buffer[LEFT] && !ground_buffer[CENTER] && !ground_buffer[RIGHT] && ground_buffer[MOST_RIGHT]){
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
        case ALL:
            if(laps >= 2){ //Left wall
                error = 10.0;
            }else
                error = -10.0;
        break;

        case MOST_LEFT:
            //turn left
            if(laps >= 2){ //Left wall
                error = 0.0;
            }else
                error = -5.0;
        break;
        case MOST_RIGHT:
            //turn left
            if(laps >= 2){ //Left wall
                error = 5.0;
            }else
                error = 0.0;
        break;

        case LEFT:
            if(laps >= 2){ //Left wall
                error = 1.0;
            }else
                error = -4.0;
        break;

        case CENTER:
            if(laps >= 2){ //Left wall
                error = 2.0;
            }else
                error = -2.0;
        break;

        case RIGHT:
            if(laps >= 2){ //Left wall
                error = 4.0;
            }else
                error = -1.0;

        break;
        default:
            //printHeader("LINE", CLR_RED);
            //printf("Following a line! Default case!\n");
        break;
    }

    
    total_error += fabs(error);
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

    if(sensor < 50 || analogSensors.obstSensFront < 25 ){

        return OBSTACLE;
    }


    // NO_WALL (first time)
    if(obst_history == OBSTACLE ){
        // double factor = 22.5 - 0.75* sensor_dist_history;

        // xx = ( (1/sqrt(2)) * sensor_dist_history + factor ) * 10; //until center of robot
        // xx = (xx + 30 ) /2;
        // if(sensor_dist_history > 30) xx = 15*10;
        //double yy  = (1/sqrt(2)) * sensor_dist_history -5.5;
        //printf( "dist = %d | y = %.2f | x = %.2f\n", sensor_dist_history, yy, xx/10);
        xx = ((1/sqrt(2) ) * sensor_dist_history - 5.5 )*10 + 100;//mm 
        if (xx > 200)
            xx = 200;
        dist_blind = false;
        double t;
        getRobotPos(&dead_angle_Xpos, &dead_angle_Ypos, &t);

        /*pos[0] = cos(t) * xx;
        if(abs(pos[0]) < 2) pos[0] = 0;
        pos[1] = sin(t) * xx;
        if(abs(pos[1])< 2) pos[1] = 0;
*/
        
    }


    
    return NO_WALL;
}




