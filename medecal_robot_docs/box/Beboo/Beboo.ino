#include <LiquidCrystal.h>
const int in1 = 3;
const int in2 = 4;
const int in3 = 7;
const int in4 = 8;
const int en1 = 5;
const int en2 = 6;
const int buz = 9;
const int led1 = 10;
const int led2 = 11;
const int led3 = 12;
const int led_sensor = 13;
const int ir1 = 14;
const int ir2 = 15;
const int ir3 = 16;

#define ON 1
#define OFF 0

#define TAKEN 1
#define NOT_TAKEN 0


int state;

void forward();
void back();
void left();
void right();
void Stop();
void Move1();
void BUZZER(int x);
void Alaram_Buzzer(int count);
void LED(int a, int b);

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // set up the LCD's number of columns and rows:

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("pataint name");
  lcd.setCursor(0, 1);
  lcd.print("Date - Day");
}

void loop() {

  Move1();
  lcd.setCursor(0, 2);
  lcd.print("Take your medicine");
  Alaram_Buzzer(3);
  LED(1, ON);
  delay(60 * 1000);
  Alaram_Buzzer(3);
  LED(1, OFF);
  state = Check_Medicine(1) if (State == TAKEN);
  {


  }
  else if (State == NOT_TAKEN) {'
  
  
  
  '
  }
}

void Move1() {
  forward();
  delay(5000);
  left();
  delay(300);
  forward();
  delay(2000);
  Stop();
  left();
  delay(300);
}
void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 0);
}
void back() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 0);
}
void left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 127);
}
void right() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 127);
}
void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  analogWrite(buz, 255);
}

void BUZZER(int x) {
  if (x == ON) {
    digitalWrite(buz, HIGH);
  } else if (x == OFF) {
    digitalWrite(buz, LOW);
  }
}
void Alaram_Buzzer(int count) {
  for (int i = 0; i < count; i++) {
    BUZZER(ON);
    delay(500);
    BUZZER(OFF);
    delay(500);
  }
}
void LED(int a, int b) {
  switch (a) {
    case 1:
      if (b == ON) {
        digitalWrite(led1, HIGH);
      } else if (b == OFF) {
        digitalWrite(led1, LOW);
      }
      break;
    case 2:
      if (b == ON) {
        digitalWrite(led2, HIGH);
      } else if (b == OFF) {
        digitalWrite(led2, LOW);
      }
      break;
    case 3:
      if (b == ON) {
        digitalWrite(led3, HIGH);
      } else if (b == OFF) {
        digitalWrite(led3, LOW);
      }
      break;
    case 4:
      if (b == ON) {
        digitalWrite(led_sensor, HIGH);
      } else if (b == OFF) {
        digitalWrite(led_sensor, LOW);
      }
      break;
  }
  int check_medicen(int box_no) {
    int sensor_ir_read;
    switch (box_no) {
case1:
      sensor_ir_read = digitalread(ir1) break;
case2:
      sensor_ir_read = digitalread(ir2) break;
case3:
      sensor_ir_read = digitalread(ir3) break;
    }
    return sensor_ir_read;
  }
}
