const int groundEnablePin, ground0pin, ground1pin, ground2pin, ground3pin, ground4pin = 7, 2, 3, 4, 5, 6;
bool ground0, ground1, ground2, ground3, ground4;

const int ir0pin, ir1pin = 4, 5;
float ir0, ir1;

const int button0pin, button1pin = 8, 9;
bool button0, button1;

const int m1enA, m1enB, m2enA, m2enB = 13, 12, 11, 10;
int m1, m2;

void setup() {
  digitalWrite(groundEnable, LOW);
  pinMode(groundEnable, OUTPUT);

  pinMode(button0pin, INPUT);
  pinMode(button1pin, INPUT);
  
  Serial.begin(9600);
}


int readLineSensors() {
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
  digitalWrite(groundEnable, HIGH);    

  // wait 0.5 ms (for the default gain)
  delayMicroseconds(500);           
  ground0 = digitalRead(ground0pin);
  ground1 = digitalRead(ground1pin);
  ground2 = digitalRead(ground2pin);
  ground3 = digitalRead(ground3pin);
  ground4 = digitalRead(ground4pin);

  // Disable line sensor
  digitalWrite(groundEnable, LOW);  
}

readIrSensors(){
  ir0=2547.8/((float)analogRead(ir0pin)*0.49-10.41)-0.42;
  if(ir0<5)
    ir0=5;
  else if(ir0>80)
    ir0=80;

  ir1=2547.8/((float)analogRead(ir1pin)*0.49-10.41)-0.42;
  if(ir1<5)
    ir1=5;
  else if(ir1>80)
    ir1=80;
}

readButtons(){
  button0 = digitalRead(button0pin);
  button1 = digitalRead(button1pin);
}

void loop() {
  readLineSensors();
  readIrSensors();

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
  
  //delay(250);
}
