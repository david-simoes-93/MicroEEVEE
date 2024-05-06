const int groundEnablePin = 7;
const int ground0pin = 4;
const int ground1pin = 5;
const int ground2pin = 6;
const int ground3pin = 8;
const int ground4pin = 9;
bool ground0, ground1, ground2, ground3, ground4;

const int ir0pin = 0;
const int ir1pin = 1;
float ir0, ir1;

const int button0pin = 13;
const int button1pin = 10;
bool button0, button1;

const int m2enA = 2;
const int m2enB = 11;
const int m1enA = 3;
const int m1enB = 12;
int m1, m2, counterM1, counterM2;

void setup()
{
  digitalWrite(groundEnablePin, LOW);
  pinMode(groundEnablePin, OUTPUT);

  pinMode(button0pin, INPUT);
  pinMode(button1pin, INPUT);

  // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(digitalPinToInterrupt(m1enA), m1_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(m2enA), m2_enc, RISING);

  Serial.begin(9600);
}

void readLineSensors()
{
  // All 5 outputs set
  digitalWrite(ground0pin, HIGH);
  digitalWrite(ground1pin, HIGH);
  digitalWrite(ground2pin, HIGH);
  digitalWrite(ground3pin, HIGH);
  digitalWrite(ground4pin, HIGH);
  pinMode(ground0pin, OUTPUT);
  pinMode(ground1pin, OUTPUT);
  pinMode(ground2pin, OUTPUT);
  pinMode(ground3pin, OUTPUT);
  pinMode(ground4pin, OUTPUT);

  // Wait, discharging capacitors
  delayMicroseconds(1000);

  // charge capacitors
  pinMode(ground0pin, INPUT);
  pinMode(ground1pin, INPUT);
  pinMode(ground2pin, INPUT);
  pinMode(ground3pin, INPUT);
  pinMode(ground4pin, INPUT);

  // Enable line sensor
  digitalWrite(groundEnablePin, HIGH);

  // wait 0.5 ms (for the default gain)
  delayMicroseconds(400);
  ground0 = digitalRead(ground0pin);
  ground1 = digitalRead(ground1pin);
  ground2 = digitalRead(ground2pin);
  ground3 = digitalRead(ground3pin);
  ground4 = digitalRead(ground4pin);

  // Disable line sensor
  digitalWrite(groundEnablePin, LOW);
}

void readIrSensors()
{
  ir0 = 2547.8 / ((float)analogRead(ir0pin) * 0.49 - 10.41) - 0.42;
  if (ir0 < 5)
    ir0 = 5;
  else if (ir0 > 80)
    ir0 = 80;

  ir1 = 2547.8 / ((float)analogRead(ir1pin) * 0.49 - 10.41) - 0.42;
  if (ir1 < 5)
    ir1 = 5;
  else if (ir1 > 80)
    ir1 = 80;
}

void readButtons()
{
  button0 = digitalRead(button0pin);
  button1 = digitalRead(button1pin);
}

void m1_enc()
{
  if (digitalRead(m1enB))
    counterM1++;
  else
    counterM1--;
}

void m2_enc()
{

  if (digitalRead(m2enB))
    counterM2++;
  else
    counterM2--;
}

void readMotors()
{
  noInterrupts();
  m1 = -counterM1;
  m2 = -counterM2;
  counterM1 = 0;
  counterM2 = 0;
  interrupts();
}

void loop()
{
  readLineSensors();
  readIrSensors();
  readButtons();
  readMotors();

  Serial.print(ir0);
  Serial.print(";");
  Serial.print(ir1);
  Serial.print(";");

  Serial.print(button0);
  Serial.print(";");
  Serial.print(button1);
  Serial.print(";");

  Serial.print(ground0);
  Serial.print(";");
  Serial.print(ground1);
  Serial.print(";");
  Serial.print(ground2);
  Serial.print(";");
  Serial.print(ground3);
  Serial.print(";");
  Serial.print(ground4);
  Serial.print(";");

  Serial.print(m1);
  Serial.print(";");
  Serial.print(m2);
  Serial.println(";");

  // delay(250);
}
