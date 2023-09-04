const int sensor1Pin = 2;    // pin for the first sensor
const int sensor2Pin = 3;    // pin for the second sensor
const int sensor3Pin = 4;    // pin for the third sensor
const int sensor4Pin = 5;    // pin for the fourth sensor
const int sensor5Pin = 6;    // pin for the fifth sensor
const int leftMotorPin = 9;  // pin for the left motor
const int rightMotorPin = 10; // pin for the right motor
const int in1 = 3;
const int in2 = 4;
const int in3 = 7;
const int in4 = 8;
const int en1 = 5;
const int en2 = 6;
const int buz =9;
void Forward () {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
 analogWrite(buz,0);

}
void Back() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
   analogWrite(buz,0);

}
void Left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
 analogWrite(buz,127);

}
void Right() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
 analogWrite(buz,127);

}
void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
 analogWrite(buz,255);
}
void setup()
{
    pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
pinMode(buz,OUTPUT);

  // Set the sensor and motor pins as inputs and outputs
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(sensor3Pin, INPUT);
  pinMode(sensor4Pin, INPUT);
  pinMode(sensor5Pin, INPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
}

void loop()
{
  // Read the sensor values
  int sensor1Value = digitalRead(sensor1Pin);
  int sensor2Value = digitalRead(sensor2Pin);
  int sensor3Value = digitalRead(sensor3Pin);
  int sensor4Value = digitalRead(sensor4Pin);
  int sensor5Value = digitalRead(sensor5Pin);

  // Set the motor speeds based on the sensor values
  if (sensor1Value == LOW && sensor2Value == LOW && sensor3Value == LOW && sensor4Value == LOW && sensor5Value == LOW)
  {
    // If all sensors are off the line, stop
   Stop();
  }
  else if (sensor1Value == HIGH && sensor2Value == LOW && sensor3Value == LOW && sensor4Value == LOW && sensor5Value == LOW)
  {
    // If only the first sensor is on the line, turn left
 Left();
  }
  else if (sensor1Value == LOW && sensor2Value == LOW && sensor3Value == LOW && sensor4Value == LOW && sensor5Value == HIGH)
  {
    // If only the fifth sensor is on the line, turn right
Right();
  }
  else
  {
    // If more than one sensor is on the line, go straight
Forward();
  }
}
