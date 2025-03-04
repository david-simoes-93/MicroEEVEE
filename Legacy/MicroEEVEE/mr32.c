/**
 * \file
 * \brief MR32 library (Source File)
 */

// ****************************************************************************
// MR32.C
//
// Current version: 
//  - 1.6 - 23/10/2014
//  - 1.7 - 16/09/2015
//  - 1.8 - 13/04/2016
//  - 1.9 - 21/09/2016
//  - 2.0 - 15/09/2017
//  - 2.1 - 12/04/2018
//
// J.L.Azevedo, DETI-UA
// ****************************************************************************

#include "mr32.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IN  1
#define OUT 0

// ****************************************************************************
// local functions
void pid(int sp_m1, int enc_m1, int sp_m2, int enc_m2);

void readEncoders(int *enc_m1, int *enc_m2);

void actuateMotors(int pwmL, int pwmR);

unsigned int updateBatteryVoltage(int value);

void setPWM2(int pwmL, int pwmR);

void setSP2(int spL, int spR);

int median(int sensor, int newvalue);

int average(int sensor, int newValue);

int calcDistance(int adcValue);


// ****************************************************************************
// Servo calibarion values (these values must be adjusted for each servo)
#define SERVO_MIN_PWM_IS_RIGHT

#if(ROBOT == 1)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 2)
int SERVO_WIDTH_MIN = 960; // 0.96 ms
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 3)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 5)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 6)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 7)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 8)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 9)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 10)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 11)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#elif(ROBOT == 12)
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#else
int SERVO_WIDTH_MIN = 960; // 0.96 ms 
int SERVO_WIDTH_MAX = 2000; // 2 ms
#endif


// ****************************************************************************
//
#define M1_IN1 LATBbits.LATB5
#define M1_IN2 LATCbits.LATC13
#define M2_IN1 LATBbits.LATB13
#define M2_IN2 LATFbits.LATF3
#define STDBY  LATCbits.LATC14

#define LED1   LATEbits.LATE0
#define LED2   LATEbits.LATE1
#define LED3   LATEbits.LATE2
#define LED4   LATEbits.LATE3

#define M1_FORWARD M1_IN1=1; M1_IN2=0
#define M1_REVERSE M1_IN1=0; M1_IN2=1

#define M2_FORWARD M2_IN1=0; M2_IN2=1
#define M2_REVERSE M2_IN1=1; M2_IN2=0

// ****************************************************************************
// Servo constants (do not change any of these values)
//
#define POS_LEFT     -15
#define POS_RIGHT    15
#define SERVO_LEVELS (POS_RIGHT - POS_LEFT)
#define T2_FREQ      625  // fin_t2 = 625 kHz
#define SERVO_K  (((SERVO_WIDTH_MAX - SERVO_WIDTH_MIN) * T2_FREQ) / 1000) / SERVO_LEVELS

// **************************************************************************
// Global variables
volatile bool tick10ms;
volatile bool tick20ms;
volatile bool tick40ms;
volatile bool tick80ms;
volatile bool tick160ms;

MR32_analogSensors analogSensors;
volatile static int counterLeft;
volatile static int counterRight;
volatile static int spLeft, spRight;
volatile static double xpos, ypos, theta;
volatile static int pwmRight, pwmLeft;

static bool _closedLoopControl = false;

// ***************************************************************************
// Robot dimensions
#define  WHEEL2WHEEL_DIST  165       // mm
#define  WHEEL_DIAM        80.5       // mm [changed from 80.6]
#define  WHEEL_PER         (PI * WHEEL_DIAM)

// ***************************************************************************
// Motor parameters
#define  RPM_OUT       160            // RPM in the output shaft @ 9.6V
#define GEAR_RATIO     10             // Gear box ratio
#define ENCODER_PULSES 100            // Encoder, pulses per revolution

// ***************************************************************************
// Controller parameters
#define SAMPLING_T    20             // [ms]
#define SP_MAX     ((RPM_OUT * GEAR_RATIO * ENCODER_PULSES * SAMPLING_T) / 60000)
#define INTEGRAL_CLIP 70             // Cliping value of the Integral component
const signed int KP_num = 7, KP_den = 2;   // Proportional constant
const signed int KI_num = 1, KI_den = 1;   // Integral constant

// ***************************************************************************
#define DIM_BAT_ARRAY   128          // Dimension of the battery voltage array
#define MEDIAN_SIZE     5            // Median buffer size for obstacle sensor filtering              
#define AVERAGE_SIZE    1            // Average buffer size for obstacle sensor filtering              

// ****************************************************************************
// initPIC32()
//
/** \brief initPIC32 Init PIC32 peripherals
 */
void initPIC32(void) {
    int i;
    // Disable JTAG
    DDPCON = 3;

    // Config Timer2, Timer3, OC1, OC2 and OC5
    T2CONbits.TCKPS = 5; // 1:16 prescaler (i.e. fin = 625 KHz)
    PR2 = 6249;          // Fout = 20M / (32 * (6249 + 1)) = 100 Hz
    TMR2 = 0;            // Reset timer T2 count register
    T2CONbits.TON = 1;   // Enable timer T2 (must be the last command of
    // the timer configuration sequence)
    //
    T3CONbits.TCKPS = 4; // 1:32 prescaler (i.e. fin = 1.25 MHz)
    PR3 = 63;            // Fout = 20M / (16 * (63 + 1)) = 20000 Hz
    TMR3 = 0;            // Reset timer T2 count register
    T3CONbits.TON = 1;   // Enable timer T2 (must be the last command of
    // the timer configuration sequence)
    // Motor1 PWM
    OC1CONbits.OCM = 6;  // PWM mode on OCx; fault pin disabled
    OC1CONbits.OCTSEL = 1;// Use timer T3 as the time base for PWM generation
    OC1RS = 0;           //
    OC1CONbits.ON = 1;   // Enable OC1 module

    // Motor2 PWM
    OC2CONbits.OCM = 6;  // PWM mode on OCx; fault pin disabled
    OC2CONbits.OCTSEL = 1;// Use timer T3 as the time base for PWM generation
    OC2RS = 0;           //
    OC2CONbits.ON = 1;   // Enable OC2 module

    // Servo PW
    OC5CONbits.OCM = 6;  // PWM mode on OCx; fault pin disabled
    OC5CONbits.OCTSEL = 0;// Use timer T2 as the time base for PWM generation
    OC5RS = 0;           //
    OC5CONbits.ON = 1;   // Enable OC5 module

    IFS0bits.T2IF = 0;
    IPC2bits.T2IP = 1;
    IEC0bits.T2IE = 1;   // Enable Timer 2 interrupts
    setServoPos(0);

    // ****************************************************************************
    // IO Config
    //
    //  1-Bridge control
    STDBY = 1;                 // Half-Bridge ON

    M1_IN1 = M1_IN2 = 0;       // STOP
    M2_IN1 = M2_IN2 = 0;       // STOP

    TRISCbits.TRISC14 = OUT;   // STDBY
    TRISBbits.TRISB5 = OUT;    // M1_IN1
    TRISCbits.TRISC13 = OUT;   // M1_IN2

    TRISBbits.TRISB13 = OUT;   // M2_IN1
    TRISFbits.TRISF3 = OUT;    // M2_IN2

    //  2-Leds
    LATECLR = 0x000F;          // Leds 4-1 OFF
    LATBCLR = 0x8000;          // Led 5 OFF
    TRISECLR = 0x000F;         // RE3-0 as output
    TRISBCLR = 0x8000;         // RB15 as output

    //  3-Sensors
    LATBbits.LATB10 = 0;       // Disable Obstacle sensors output
    TRISBbits.TRISB10 = OUT;   // EN_OBST_SENS as output
    TRISBbits.TRISB9 = IN;     // IV BEACON as input

    LATECLR = 0x0020;          // Disable line sensor
    TRISEbits.TRISE5 = OUT;    // EN_GND_SENS as output

    LATD = LATD | 0x00EC;      // Line sensor: output latches must be set
    TRISD = TRISD & ~(0x00EC); // Line sensor: 5 bits as output

    //  4- start/stop buttons
    CNPUE = CNPUE | 0x60;      // Activate weak pull-ups in input ports RB3 and RB4

    // ADC Config
    AD1PCFGbits.PCFG0 = 0;     // RB0 configured as analog input (AN0)
    AD1PCFGbits.PCFG1 = 0;     // RB1 configured as analog input (AN1)
    AD1PCFGbits.PCFG2 = 0;     // RB2 configured as analog input (AN2)
    AD1PCFGbits.PCFG6 = 0;     // RB6 configured as analog input (AN6)
    AD1PCFGbits.PCFG7 = 0;     // RB7 configured as analog input (AN7)
    AD1PCFGbits.PCFG11 = 0;    // RB11 configured as analog input (AN11)

    AD1CON1bits.SSRC = 7;      // Conversion trigger: internal counter ends
    // sampling and starts conversion
    AD1CON1bits.CLRASAM = 1;   // Stop conversions when the 1st A/D converter
    // interrupt is generated. At the same time,
    // hardware clears the ASAM bit
    AD1CON3bits.SAMC = 16;     // Sample time is 16 TAD (TAD = 100 ns)
    AD1CON2bits.SMPI = 2 - 1;  // Interrupt is generated after 2 samples
    AD1CON1bits.ON = 1;        // Enable A/D converter

    // Encoders
    INTCONbits.INT1EP = 1;     // interrupt generated on rising edge
    INTCONbits.INT4EP = 1;     // interrupt generated on rising edge

    IPC1bits.INT1IP = 4;
    IPC4bits.INT4IP = 4;

    IFS0bits.INT1IF = 0;
    IFS0bits.INT4IF = 0;

    IEC0bits.INT1IE = 1;       // Enable INT1 interrupts
    IEC0bits.INT4IE = 1;       // Enable INT4 interrupts

    pwmLeft = pwmRight = 0;
    _closedLoopControl = true;
    counterLeft = counterRight = 0;
    spLeft = spRight = 0;
    theta = 0.0;               // Initially robot points towards the X world reference axis

    // Fill in the battery array
    for (i = 0; i < DIM_BAT_ARRAY; i++)
        readAnalogSensors();

    EnableInterrupts();
}


// Rough distance calculation

// int calcDistance(int sensValue)
// {
//    int dist;
//    sensValue = sensValue < 79 ? 79 : sensValue;
// 
//    dist = 6200 / (sensValue - 78);  // distance in centimeters
//    return dist > 80 ? 80 : dist;
// }

#define NR_CALIB_VALUES 16

// Function to calculate the distance measured by a Sharp GP2D12 sensor
// Calibration must be done for each sensor, individually
//
int calcDistance(int adcValue) {
    static int calibValue[] = {840, 773, 548, 427, 349, 296, 260, 227, 201, 184, 168, 155, 144, 136, 128, 122};
    static float dist[] = {8.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0,
                           80.0};
    int i, distance = 0;

    adcValue = adcValue < calibValue[NR_CALIB_VALUES - 1] ? calibValue[NR_CALIB_VALUES - 1] : adcValue;
    adcValue = adcValue > calibValue[0] ? calibValue[0] : adcValue;

    for (i = 1; i < NR_CALIB_VALUES; i++) {
        if (adcValue >= calibValue[i]) {
            distance = (int) (
                    ((dist[i] - dist[i - 1]) * (adcValue - calibValue[i])) / (calibValue[i] - calibValue[i - 1]) +
                    dist[i] + 0.5);
            break;
        }
    }
    return distance;
}

// ****************************************************************************
// readAnalogSensors()
//
void readAnalogSensors(void) {
    // Ch 00 - Obstacle sensor 1
    // Ch 01 - Obstacle sensor 2
    // Ch 02 - Obstacle sensor 3
    // Ch 06 - Additional analog input (Connector CN3)
    // Ch 07 - Additional analog input (Connector CN3)
    // Ch 11 - Battery
    static int channels[] = {0, 1, 2, 6, 7, 11}; // Do not change order...
    int i;
    int adcValue, filteredValue;

    for (i = 0; i < 6; i++) {
        AD1CHSbits.CH0SA = channels[i];  // analog channel
        AD1CON1bits.ASAM = 1;            // Start conversion
        while (IFS1bits.AD1IF == 0);    // Wait until AD1IF = 1
        adcValue = (ADC1BUF0 + ADC1BUF1) / 2;
        IFS1bits.AD1IF = 0;
        switch (i) {
            case 0:  // Obstacle sensors
            case 1:
            case 2:
                filteredValue = median(i, adcValue);
                if (AVERAGE_SIZE > 1)
                    filteredValue = average(i, filteredValue);

                analogSensors.array[i] = calcDistance(filteredValue) * 100;
                break;
            case 5:  // Battery voltage
                analogSensors.array[BATTERY] = updateBatteryVoltage(adcValue);
                break;

            default:
                analogSensors.array[i] = adcValue;
                break;
        }
    }
}

void sort(int *array, int size) {
    int flag, i, j, aux;
    j = size;
    do {
        flag = false;
        for (i = 0; i < j - 1; i++) {
            if (array[i] > array[i + 1]) {
                aux = array[i];
                array[i] = array[i + 1];
                array[i + 1] = aux;
                flag = true;
            }
        }
        j--;
    } while (flag);
}

int median(int sensor, int newValue) {
    static int buf[3][MEDIAN_SIZE];
    static int i[3] = {0, 0, 0};
    int aux[MEDIAN_SIZE];
    int j, k;

    sensor = sensor > 2 ? 2 : sensor;
    k = i[sensor];

    buf[sensor][k] = newValue;
    i[sensor] = (k + 1) % MEDIAN_SIZE;

    for (j = 0; j < MEDIAN_SIZE; j++)
        aux[j] = buf[sensor][j];

    sort(aux, MEDIAN_SIZE);
    return aux[MEDIAN_SIZE / 2];
}

int average(int sensor, int newValue) {
    static int buf[3][AVERAGE_SIZE];
    static int i[3] = {0, 0, 0};
    static int soma[3];
    int k;

    sensor = sensor > 2 ? 2 : sensor;
    k = i[sensor];

    soma[sensor] = soma[sensor] - buf[sensor][k] + newValue;
    buf[sensor][k] = newValue;
    i[sensor] = (k + 1) % AVERAGE_SIZE;
    return soma[sensor] / AVERAGE_SIZE;
}


// ****************************************************************************
// readLineSensors()
//
// Input: gain [1...100] (100 -> maximum gain)
//    Default value is 0  (gain = 50)
unsigned int readLineSensors(int gain) {
    unsigned int sensValue;

    if (gain <= 0)
        gain = 50;
    else if (gain > 100)
        gain = 100;

    // discharge capacitors
    LATECLR = 0x0020;          // Disable line sensor
    LATD = LATD | 0x00EC;      // All 5 outputs set (just in case, set in initPIC32() )
    TRISD = TRISD & ~(0x00EC); // 5 bits as output

    delay(1);                  // Wait, discharging capacitors
    // charge capacitors
    TRISD = TRISD | 0x00EC;    // 5 bits as input
    LATESET = 0x0020;          // Enable line sensor

    delay(gain);               // wait 5 ms (for the default gain)
    // this time is critical... capacitors take time to charge
    // too little time: output capacitors don't charge enough
    sensValue = PORTD >> 2;
    sensValue = (sensValue & 0x0003) | ((sensValue & 0x38) >> 1);
    LATECLR = 0x0020;          // Disable line sensor

    return sensValue;
}

void getRobotPos(double *xp, double *yp, double *hp) {
    DisableInterrupts();
    *xp = xpos;
    *yp = ypos;
    *hp = theta;
    EnableInterrupts();
}

void getRobotPos_int(int *xx, int *yy, double *tt){
   DisableInterrupts();
   *xx = (int) xpos * 10;
   *yy = (int)ypos * -10; //same ref from map
   *tt = theta;

    EnableInterrupts();
}

void setRobotPos(double xp, double yp, double hp) {
    DisableInterrupts();
    xpos = xp;
    ypos = yp;
    theta = hp;
    EnableInterrupts();
}


// ****************************************************************************
// updateBatteryVoltage()
//
// - update battery voltage (average of the last DIM_BAT_ARRAY readings)
// - returned value is multiplied by 10 (max. value is 101, i.e. 10,1 V)
unsigned int updateBatteryVoltage(int value) {
    static int array[DIM_BAT_ARRAY];
    static int i = 0, sum = 0;

    value = (value * 330 + 511) / 1023;
//   value = (value * (3300 + 6800) + 16500) / 33000;
    value = (value * (3300 + 10000) + 16500) / 33000;

    sum = sum - array[i] + value;
    array[i++] = value;
    if (i >= DIM_BAT_ARRAY)
        i = 0;

    return sum / DIM_BAT_ARRAY;
}


// ****************************************************************************
// setServoPos(int pos)
//
void setServoPos(int pos) {
#ifdef SERVO_MIN_PWM_IS_LEFT
    pos = pos < POS_LEFT ? POS_LEFT : pos;
    pos = pos > POS_RIGHT ? POS_RIGHT : pos;
    pos += -POS_LEFT; // PWM is minimum @ left position
#else
    pos = pos < POS_LEFT ? POS_LEFT : pos;
    pos = pos > POS_RIGHT ? POS_RIGHT : pos;
    pos = -pos;   //
    pos += POS_RIGHT; // PWM is minimum @ right position
#endif
    OC5RS = ((SERVO_WIDTH_MIN * T2_FREQ) / 1000 + pos * SERVO_K) + 1;
}

// ****************************************************************************
// led()
//
void led(int ledNr, int value) {
    // Value: 0->OFF, 1->ON

    ledNr = ledNr < 0 ? 0 : ledNr;
    ledNr = ledNr > 3 ? 3 : ledNr;

    value &= 0x01;
    if (value == 1)
        LATE = LATE | (value << ledNr);
    else {
        value = 1;
        LATE = LATE & ~(value << ledNr);
    }
}

// ****************************************************************************
// leds()
//
void leds(int value) {
    value &= 0x0F;

    LATE = (LATE & 0xFFF0) | value;
}

// ****************************************************************************
// obstacleSensor() - Get the value of a specific obstacle sensor
//
unsigned int obstacleSensor(unsigned int sensorId) {
    if (sensorId > 2)
        sensorId = 2;
    return analogSensors.array[sensorId];
}

// ****************************************************************************
// batteryVoltage() - Get the value of the battery voltage
//
unsigned int batteryVoltage(void) {
    return analogSensors.array[BATTERY];
}

// ****************************************************************************
// setSP2()
//
void setSP2(int spL, int spR) {
    spL = spL > SP_MAX ? SP_MAX : spL;
    spR = spR > SP_MAX ? SP_MAX : spR;
    spL = spL < -SP_MAX ? -SP_MAX : spL;
    spR = spR < -SP_MAX ? -SP_MAX : spR;

    DisableInterrupts();
    spLeft = spL;
    spRight = spR;
    EnableInterrupts();
}

// ****************************************************************************
// setPWM2()
//
void setPWM2(int pwmL, int pwmR) {
    DisableInterrupts();
    pwmLeft = pwmL;
    pwmRight = pwmR;
    EnableInterrupts();
}

void setVel2(int velL, int velR) {
    velL /= 2;    velR /= 2;
    if (_closedLoopControl == true)
        setSP2(velL / 2, velR / 2);
    else
        setPWM2(velL, velR);
}

void closedLoopControl(bool flag) {
    _closedLoopControl = flag;
}


// ****************************************************************************
// readEncoders()
//
void readEncoders(int *encLeft, int *encRight) {
    DisableInterrupts();
    *encLeft = -counterLeft;
    *encRight = counterRight;
    counterLeft = counterRight = 0;
    EnableInterrupts();
}

// ****************************************************************************
double normalizeAngle(double angle) {
    while (angle <= -PI) angle += 2.0 * PI;
    while (angle > PI) angle -= 2.0 * PI;
    return angle;
}

// ****************************************************************************
// delay() - input: value in 1/10 ms
//
void delay(unsigned int tenth_ms) {
    tenth_ms = tenth_ms > 500000 ? 500000 : tenth_ms;

    resetCoreTimer();
    while (readCoreTimer() <= (2000 * tenth_ms));
}

// ****************************************************************************
// wait() - input: value in 1/10 s
//
void wait(unsigned int tenth_seconds) {
    resetCoreTimer();
    while (readCoreTimer() <= (2000000 * tenth_seconds));
}


// ****************************************************************************
// PID controller. 
// 
// THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE ROUTINE (T >= 10ms)
// DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE (MAIN). DOING THAT MAY 
// PERMANENTLY DAMAGE THE MOTOR DRIVE.
//
void pid(int spMLeft, int encMLeft, int spMRight, int encMRight) {
    static int erro, prop = 0;
    static int integralMLeft, integralMRight = 0;
    static int cmdMLeft, cmdMRight;

    erro = -(encMLeft - spMLeft);
    prop = (KP_num * erro) / KP_den;
    integralMLeft += erro;

    integralMLeft = integralMLeft > INTEGRAL_CLIP ? INTEGRAL_CLIP : integralMLeft;
    integralMLeft = integralMLeft < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integralMLeft;

    cmdMLeft = prop + (integralMLeft * KI_num) / KI_den;

    cmdMLeft = cmdMLeft > 100 ? 100 : cmdMLeft;
    cmdMLeft = cmdMLeft < -100 ? -100 : cmdMLeft;

    erro = -(encMRight - spMRight);
    prop = (KP_num * erro) / KP_den;
    integralMRight += erro;

    integralMRight = integralMRight > INTEGRAL_CLIP ? INTEGRAL_CLIP : integralMRight;
    integralMRight = integralMRight < -INTEGRAL_CLIP ? -INTEGRAL_CLIP : integralMRight;

    cmdMRight = prop + (integralMRight * KI_num) / KI_den;

    cmdMRight = cmdMRight > 100 ? 100 : cmdMRight;
    cmdMRight = cmdMRight < -100 ? -100 : cmdMRight;

    actuateMotors(cmdMLeft, cmdMRight); // Actuate directly on motors
    // (do not use any filter)
}

// ****************************************************************************
// CAUTION:
// THIS FUNCTION SHOULD BE CALLED FROM A TIMER INTERRUPT SERVICE 
// ROUTINE (T >= 10ms).
// DO NOT CALL THIS FUNCTION DIRECTLY FROM YOUR CODE (MAIN). DOING THAT MAY 
// PERMANENTLY DAMAGE THE MOTOR DRIVE.
//
// ****************************************************************************
void actuateMotors(int pwmL, int pwmR) {
    pwmL = pwmL > 100 ? 100 : pwmL;
    pwmL = pwmL < -100 ? -100 : pwmL;

    pwmR = pwmR > 100 ? 100 : pwmR;
    pwmR = pwmR < -100 ? -100 : pwmR;
    if (pwmL < 0) {
        pwmL = -pwmL;
        M1_REVERSE;
    } else {
        M1_FORWARD;
    }

    if (pwmR < 0) {
        pwmR = -pwmR;
        M2_REVERSE;
    } else {
        M2_FORWARD;
    }
    OC1RS = ((PR3 + 1) * pwmL) / 100;
    OC2RS = ((PR3 + 1) * pwmR) / 100;
}


void rotateRel_basic(int speed, double deltaAngle)
{
   double xx, yy, tt;
   double targetAngle;
   double error;
   int cmdVel, errorSignOri;

   getRobotPos(&xx, &yy, &tt);
   targetAngle = normalizeAngle(tt + deltaAngle);
   error = normalizeAngle(targetAngle - tt);
   errorSignOri = error < 0 ? -1 : 1;

   cmdVel = error < 0 ? -speed : speed;
   setVel2(-cmdVel, cmdVel);

   do
   {
      getRobotPos(&xx, &yy, &tt);
      error = normalizeAngle(targetAngle - tt);
   } while (fabs(error) > 0.01 && errorSignOri * error > 0);
   setVel2(0, 0);
}

// ****************************************************************************
void updateLocalization(int encLeft, int encRight) {
    static double dLeft, dRight, dCenter;
    static double phi;

    dLeft = (encLeft * WHEEL_PER) / (ENCODER_PULSES * GEAR_RATIO);
    dRight = (encRight * WHEEL_PER) / (ENCODER_PULSES * GEAR_RATIO);

    dCenter = (dLeft + dRight) / 2.0;
    phi = (dRight - dLeft) / WHEEL2WHEEL_DIST;

    ypos = ypos + dCenter * sin(theta);

    xpos = xpos + dCenter * cos(theta);

    theta = normalizeAngle(theta + phi);
}


// ****************************************************************************
// Interrupt Service routine - Timer2
// (called every 10 ms)
void _int_(_TIMER_2_VECTOR)isr_t2(void)
{
    static int cntT2Ticks = 0;
    static int encLeft, encRight;
    static int encLeftAcc = 0, encRightAcc = 0;

    cntT2Ticks++;
    tick10ms = 1;        // Set every 10 ms
    if ((cntT2Ticks % 2) == 0) tick20ms = 1;   // Set every 20 ms
    if ((cntT2Ticks % 4) == 0) tick40ms = 1;   // Set every 40 ms
    if ((cntT2Ticks % 8) == 0) tick80ms = 1;   // Set every 80 ms
    if ((cntT2Ticks % 16) == 0) tick160ms = 1; // Set every 160 ms

    if (_closedLoopControl == true) {
        readEncoders(&encLeft, &encRight);
        updateLocalization(encLeft, encRight);

        encLeftAcc += encLeft;
        encRightAcc += encRight;

        if (cntT2Ticks & 0x01)                  // This way SAMPLING_T is 20 ms
        {
            pid(spLeft, encLeftAcc, spRight, encRightAcc); // spLeft, spRight are global vars
            encRightAcc = encLeftAcc = 0;
        }
    } else {
        actuateMotors(pwmLeft, pwmRight);     // pwmLeft and pwmRight are global vars
    }
    if (analogSensors.array[BATTERY] <= 102)  // battery voltage <= 10.2 volt
    {
        actuateMotors(0, 0);
        while (1) {
            leds(0x005);
            wait(2);
            leds(0x0A);
            wait(2);
        }
    }
    IFS0bits.T2IF = 0;
}

// ****************************************************************************
// Interrupt Service routine - External Interrupt 1 
// (encoder coupled to left motor)
//
void _int_(_EXTERNAL_1_VECTOR)isr_enc_left(void)
{
    if (PORTEbits.RE6 == 1)
        counterLeft++;
    else
        counterLeft--;

    IFS0bits.INT1IF = 0;

}

// ****************************************************************************
// Interrupt Service routine - External Interrupt 4
// (encoder coupled to right motor)
//
void _int_(_EXTERNAL_4_VECTOR)isr_enc_right(void)
{
    if (PORTEbits.RE7 == 1)
        counterRight++;
    else
        counterRight--;

    IFS0bits.INT4IF = 0;
}

void __gxx_personality_v0(void) { exit(1); }

#ifdef __cplusplus
}
#endif
