const int in1 = 3;
const int in2 = 4;
const int in3 = 7;
const int in4 = 8;
const int en1 = 5;
const int en2 = 6;
const int buz =9;
void forward () {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
 analogWrite(buz,0);

}
void back() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
   analogWrite(buz,0);

}
void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
 analogWrite(buz,127);

}
void right() {
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

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
pinMode(buz,OUTPUT);
}






void loop() {
  forward();
  delay(5000);
  Stop();
  delay(2000);
  back();
  delay(5000);
  Stop();
  delay(1000);
  right();
  delay(2000);
  left();
  delay(3000);
  Stop();


  delay(5000);
}
