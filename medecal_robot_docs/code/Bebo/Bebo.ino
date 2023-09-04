/*Section includes */
#include <LiquidCrystal.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <time.h>

/*Section macros */
typedef unsigned char       uint8;
typedef signed char         sint8;
typedef short unsigned int  uint16;
typedef signed short int    sint16;
typedef unsigned long int   uint32;
typedef signed  long int    sint32;
typedef float               f32;
typedef double              f64;


/*Section macros function  */
#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data  and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16 irBuffer[100]; //infrared LED sensor data
uint16 redBuffer[100]; //red LED sensor data
#else
uint32 irBuffer[100]; //infrared LED sensor data
uint32 redBuffer[100]; //red LED sensor data
#endif
/*Section Data type */
MAX30105 particleSensor;
typedef enum {

  NOT_TAKEN = 0x00,
  TAKEN

} take_statuse_t;

typedef enum {

  GOING_TO_PATENT = 0x00,
  GOING_TO_START

} robot_statuse_t;

typedef enum {

  BEATSPERMINUTE = 0,
  BEATAVG

} heart_rate_t;


typedef enum {
  OFF_STATE = 0x00,
  ON_STATE
};

/*define global variables*/
uint32 spo2=0; //SPO2 value
uint32 bufferLength=0; //data length
uint8 validSPO2; //indicator to show if the SPO2 calculation is valid
uint32 heartRate; //heart rate value
uint8 validHeartRate; //indicator to show if the heart rate calculation is  valid
uint8 pulseLED = 11; //Must be on PWM pin
uint8 readLED = 13; //Blinks with each data read

time_t start_time = time(NULL);

uint32 irValue=0;

uint8 RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
uint8 rates[4]; //Array of heart rates
uint8 rateSpot = 0;
uint32 lastBeat = 0; //Time at which the last beat occurred
f32 beatsPerMinute;
uint32 beatAvg;

const uint8 in1 = 3;
const uint8 in2 = 4;
const uint8 in3 = 7;
const uint8 in4 = 8;
const uint8 en1 = 5;
const uint8 en2 = 6;
const uint8 buz = 9;
const uint8 led1 = 10;
const uint8 led2 = 11;
const uint8 led3 = 12;
const uint8 led_sensor = 13;
const uint8 ir1 = 14;
const uint8 ir2 = 15;
const uint8 ir3 = 16;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

f32 temperature = 0.0;

/*Section function declaration  */

void robor_Move1_test();
void BUZZER(uint8 x);
void Alaram_Buzzer(uint8 count);
void LED(uint8 a, uint8 b);
int check_medicen(uint8 box_no);
void robot_move_forward();
void robot_move_backword();
void robot_turn_left();
void robot_turn_right();
void robot_Stop();
void send_data_to_app(uint8 status_);
void app_int(void);
void Measuring_Heart_Rate();
void Measuring_Oxygen_Saturation();
void Time(robot_statuse_t State);

/*-------------------------------------------------------code------------------------------------------------------*/

void setup() {


}

void loop() {
  /*declear local variables */
  uint8 state_medcen = 0;


  robor_Move1_test();
  lcd.setCursor(0, 2);
  lcd.print("Take your medicine");
  Alaram_Buzzer(3);
  LED(1, ON_STATE);
  delay(60 * 1000);
  Alaram_Buzzer(3);
  LED(1, OFF_STATE);
  state_medcen = check_medicen(1);
  if (state_medcen == TAKEN)
  {
    lcd.print("Medicine taked");

    send_data_to_app(TAKEN);
    lcd.clear();
    lcd.print("Put Your Finger");
    lcd.println("Measuring Temp");
    while (difftime(time(NULL), start_time) < 3.0) {
      // Code to be executed during the 3 seconds
      temperature = particleSensor.readTemperature();
      lcd.print("temperatureC=");
      lcd.print(temperature, 4);
      lcd.setCursor(0,0);

      if (temperature) {
        start_time = time(NULL); // Reset the start time if temperature changes
      }
    }

    // send data read to server(APP)
    send_data_to_app(temperature);
    lcd.clear();
    lcd.print("Put Your Finger");
    lcd.println("Measuring BPM ");
    while (difftime(time(NULL), start_time) < 3.0) {
      // Code to be executed during the 3 seconds

      Measuring_Heart_Rate();

      if (irValue< 50000) {
        start_time = time(NULL); // Reset the start time if temperature changes
      }
    }
    

    send_data_to_app(beatAvg);
    lcd.clear();
    lcd.print("Put Your Finger");
    lcd.println("Measuring spo2 ");
    
    while (difftime(time(NULL), start_time) < 3.0) {
      // Code to be executed during the 3 seconds

      Measuring_Oxygen_Saturation();

      if (validHeartRate < 0) {
        start_time = time(NULL); // Reset the start time if temperature changes
      }
    }
     send_data_to_app(validHeartRate);
    lcd.clear();
  }

  Time(GOING_TO_PATENT);

 if (state_medcen != TAKEN) {

  lcd.print("Medicine not taked");
  Time(GOING_TO_START);


}
}


/*-----------------------------------------functions definetions --------------------------------------------*/

/*-----------------robot section --------------------------------*/
void robor_Move1_test() {
  robot_move_forward();
  delay(5000);
  robot_turn_left();
  delay(300);
  robot_move_forward();
  delay(2000);
  robot_Stop();
  robot_turn_left();
  delay(300);
}
void robot_move_forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 0);
}
void robot_move_backword() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 0);
}
void robot_turn_left() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 127);
}
void robot_turn_right() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(en1, 127);
  analogWrite(en2, 127);
  analogWrite(buz, 127);
}
void robot_Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  analogWrite(buz, 255);
}



/*-----------------------------buzer section -------------------------------*/
void BUZZER(uint8 x) {
  if (x == ON_STATE) {
    digitalWrite(buz, HIGH);
  } else if (x == OFF_STATE) {
    digitalWrite(buz, LOW);
  }
}
void Alaram_Buzzer(uint8 count) {
  for (uint8 i = 0; i < count; i++) {
    BUZZER(ON_STATE);
    delay(500);
    BUZZER(OFF_STATE);
    delay(500);
  }
}


/*-------------------------------led section----------------------------*/
void LED(uint8 a, uint8 b) {
  switch (a) {
    case 1:
      if (b == ON_STATE) {
        digitalWrite(led1, HIGH);
      } else if (b == OFF_STATE) {
        digitalWrite(led1, LOW);
      }
      break;
    case 2:
      if (b == OFF_STATE) {
        digitalWrite(led2, HIGH);
      }
      else if (b == OFF_STATE) {
        digitalWrite(led2, LOW);
      }
      break;
    case 3:
      if (b == ON_STATE) {
        digitalWrite(led3, HIGH);
      }
      else if (b == OFF_STATE) {
        digitalWrite(led3, LOW);
      }
      break;
    case 4:
      if (b == ON_STATE) {
        digitalWrite(led_sensor, HIGH);
      } else if (b == OFF_STATE) {
        digitalWrite(led_sensor, LOW);
      }
      break;
  }

}




/*------------medecin check---------*/
int check_medicen(uint8 box_no) {
  uint8 sensor_ir_read;
  switch (box_no) {
    case 1:
      sensor_ir_read = digitalRead(ir1);
      break;
    case 2:
      sensor_ir_read = digitalRead(ir2);
      break;
    case 3:
      sensor_ir_read = digitalRead(ir3);
      break;
  }
  return sensor_ir_read;
}


/*-----------------Function Sending Data-----------*/
void send_data_to_app(uint8 status_) {

}




/*--------------function intitalize for application----------*/
void app_int(void) {
  // set up the LCD's number of columns and rows:

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("pataint name");
  lcd.setCursor(0, 1);
  lcd.print("Date - Day");

  /*********************Temperature Code********************/
  Serial.begin(9600);
  Serial.println("Initializing...");
  // Initialize sensor

  //The LEDs are very low power and won't affect the temp reading much but
  //you may want to turn off the LEDs to avoid any local heating
  particleSensor.setup(0); //Configure sensor. Turn off LEDs
  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
/*************************************SpO2*******************/
  pinMode(pulseLED, OUTPUT);
pinMode(readLED, OUTPUT);
uint8 ledBrightness = 60; //Options: 0=Off to 255=50mA
 uint8 sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
 uint8 ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR +  Green
 uint8 sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
uint32 pulseWidth = 411; //Options: 69, 118, 215, 411
uint32 adcRange = 4096; //Options: 2048, 4096, 8192, 16384
 particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, 
pulseWidth, adcRange); //Configure sensor with these settings
}


/*----------------------function Measuring Heart-Rate (BPM) ------------------------*/
void Measuring_Heart_Rate() {

  irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    uint32 delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (uint8)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (uint8 x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
 beatsPerMinute=(f32)beatsPerMinute;
 beatAvg=(uint32)beatAvg;

}



/*--------------void Measuring_Oxygen_Saturation ----------------- */
void Measuring_Oxygen_Saturation(){
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
//read the first 100 samples, and determine the signal range
for (byte i = 0 ; i < bufferLength ; i++)
 {
 while (particleSensor.available() == false) //do we have new data?
 particleSensor.check(); //Check the sensor for new data
 redBuffer[i] = particleSensor.getRed();
 irBuffer[i] = particleSensor.getIR();
 particleSensor.nextSample(); //We're finished with this sample so move to next sample
 Serial.print(F("red="));
 Serial.print(redBuffer[i], DEC);
 Serial.print(F(", ir="));
 Serial.println(irBuffer[i], DEC);
 }
//calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//Continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second
while (1)
 {
 //dumping the first 25 sets of samples in the memory and shift the last  75 sets of samples to the top
 for (byte i = 25; i < 100; i++)
 {
 redBuffer[i - 25] = redBuffer[i];
 irBuffer[i - 25] = irBuffer[i];
 }
 //take 25 sets of samples before calculating the heart rate.
 for (byte i = 75; i < 100; i++)
 {
 while (particleSensor.available() == false) //do we have new data?
 particleSensor.check(); //Check the sensor for new data
 digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with  every data read
 redBuffer[i] = particleSensor.getRed();
 irBuffer[i] = particleSensor.getIR();
 particleSensor.nextSample(); //We're finished with this sample so move to next sample


 }
 //After gathering 25 new samples recalculate HR and SP02
 maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
 }

  }

  /*---------------------------Robot State------------------*/

  void Time(robot_statuse_t State)
  {
    
  }
