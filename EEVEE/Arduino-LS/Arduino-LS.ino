const int ledPin = 13;
const int enablePin = 7;
const int enablePin2 = 6;

int buttonState = 0;
int gain = 100;

//*************** Switchs *************
const int stopButton = 6;
const int startButton = 7;

//*************** Encoders *************
static bool _closedLoopControl = true;

#define encoderLeftPinA  2
#define encoderLeftPinB  3

#define encoderRightPinA  2
#define encoderRightPinB  3
volatile static int counterLeft;
volatile static int counterRight;

//volatile unsigned int encoder0Pos = 0;

//*************************************

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
/*
  //----------- Timer config -----------
  // Config Timer2 to 100Hz

  noInterrupts();           // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 101;            // preload timer 256- 16MHz/(1024*100Hz) //timer frequancy is 100Hz
  TCCR2B |= (1 << CS12);
  TCCR2B |= (1 << CS10);    // 1024 prescaler
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
*/
  //----------- Encoders ----------------

  /* Uses Arduino pull-ups on A & B channel outputs
    turning on the pull-ups saves having to hook up resistors
    to the A & B channel outputs
  */
/*
  pinMode(encoderLeftPinA, INPUT);
  digitalWrite(encoderLeftPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderLeftPinB, INPUT);
  digitalWrite(encoderLeftPinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(digitalPinToInterrupt(encoderLeftPinA)
                  , doEncLeftA, CHANGE); // encoder pin on interrupt 0
  attachInterrupt(digitalPinToInterrupt(encoderLeftPinB), doEncLeftB, CHANGE); // encoder pin on interrupt 1

  pinMode(encoderRightPinA, INPUT);
  digitalWrite(encoderRightPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderRightPinB, INPUT);
  digitalWrite(encoderRightPinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(2, doEncRighA, CHANGE); // encoder pin on interrupt 2
  attachInterrupt(3, doEncRighB, CHANGE); // encoder pin on interrupt 3
  //-------------------------------------
*/
  Serial.begin(9600);
}

// ****************************************************************************
// Interrupt Service routine - Timer2
// (called every 10 ms)
/*ISR(TIMER2_OVF_vect)
{
  //static int cntT2Ticks = 0;
  static int encLeft, encRight;

  TCNT2 = 101;            // preload timer
  //cntT2Ticks++;

  if (_closedLoopControl == true)
  {
    readEncoders(&encLeft, &encRight);
    //updateLocalization(encLeft, encRight); //send values
  }
  //cntT2Ticks = 0;

}

// ****************************************************************************
// readEncoders()
//
void readEncoders(int *encLeft, int *encRight)
{
  noInterrupts();
  *encLeft = -counterLeft;
  *encRight = counterRight;
  counterLeft = counterRight = 0;
  interrupts();
}

//TODO ver se os encoders estao bem para para os lados que são!!!

// ****************************************************************************
// Interrupt Service routine - External Interrupt 0 e 1
//  (encoder coupled to left motor)
//
void doEncLeftA() {
  if (digitalRead(encoderLeftPinA) == digitalRead(encoderLeftPinB)) {
    counterLeft--; // CCW
  } else {
    counterLeft++; // CW
  }
}
void doEncLeftB() {
  if (digitalRead(encoderLeftPinB) == digitalRead(encoderLeftPinA)) {
    counterLeft++;  // CW
  } else {
    counterLeft--; // CCW
  }
}

// ****************************************************************************
// Interrupt Service routine - External Interrupt 2 e 3
//  (encoder coupled to right motor)
//
void doEncLeftA() {
  if (digitalRead(encoderRightPinA) == digitalRead(encoderRightPinB)) {
    counterRight--; // CCW
  } else {
    counterRight++; // CW
  }
}
void doEncLeftB() {
  if (digitalRead(encoderRightPinB) == digitalRead(encoderRightPinA)) {
    counterRight++; // CW
  } else {
    counterRight--; // CCW
  }
}
*/


// ****************************************************************************
// readLineSensors()
//
// Input: gain [1...100] (100 -> maximum gain)
//    Default value is 0  (gain = 50)
int readLineSensors() {
  unsigned int sensValue;

  /*if(gain <= 0)
    gain = 50;
    else if(gain > 100)
    gain = 100;*/

  // discharge capacitors
  digitalWrite(enablePin, LOW);   // Disable line sensor
  digitalWrite(enablePin2, LOW);

  // All 5 outputs set (just in case, set in initPIC32() )
  
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  //TRISD = TRISD & ~(0x00EC);  // 5 bits as output

  delayMicroseconds(1000);         // Wait, discharging capacitors

  // charge capacitors
  pinMode(8, INPUT);  // 5 bits as input
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  digitalWrite(enablePin, HIGH);   // Enable line sensor
  digitalWrite(enablePin2, HIGH);

  delayMicroseconds(500);        // wait 5 ms (for the default gain)
  // this time is critical... capacitors take time to charge
  // too little time: output capacitors don't charge enough
  sensValue = digitalRead(8)+ digitalRead(9)*10+digitalRead(10)*100+digitalRead(11)*1000;
  //sensValue = (sensValue & 0x0003) | ((sensValue & 0x38) >> 1);
  digitalWrite(enablePin, LOW);
  digitalWrite(enablePin2, LOW);     // Disable line sensor

  return sensValue;
}

void loop() {
  // put your main code here, to run repeatedly:
  int randNumber = random(10);

  if (randNumber > 5)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);

  Serial.print(readLineSensors(), DEC);
  Serial.print("\n");
  delay(250);
}
