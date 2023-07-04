#include <Wire.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <time.h>

// for time 
time_t start_time = time(NULL);

// defines movment robot
#define R1 7
#define L1 6
#define R2 9
#define L2 10
#define SPEED 50



//for medcine
#define MEDCINE_IS_TAKED 1
#define MEDCINE_IS_NOT_TAKED 0

//buzzer box
const int buzzerPin = 52;

//indicate to go out from while
int done=1;


// Led box
typedef enum {
  LED_BOX_FOR_MEDCINE_1 = 0,
  LED_BOX_FOR_MEDCINE_2,
  LED_BOX_FOR_MEDCINE_3
} led_box_number_t;

const int led1Pin = 46;
const int led2Pin = 44;
const int led3Pin = 42;

// sensor take  medcine
typedef enum {
  SENSOR_BOX_FOR_MEDCINE_1 = 0,
  SENSOR_BOX_FOR_MEDCINE_2,
  SENSOR_BOX_FOR_MEDCINE_3
} sensor_box_number_t;
const int sensor1Pin = 40;
const int sensor2Pin = 38;
const int sensor3Pin = 36;

// Line sensor pins for movment of robot
const int lineSensorPin1 = 31; // Line sensor pin 1
const int lineSensorPin2 = 33; // Line sensor pin 2
const int lineSensorPin3 = 35; // Line sensor pin 3
const int lineSensorPin4 = 37; // Line sensor pin 4
const int lineSensorPin5 = 39; // Line sensor pin 5
int s1, s2, s3, s4, s5;
int GO_read_call_statuse = 15;
int Get_call_statuse = 0;

//function decleration
int sensor_box_read(sensor_box_number_t sensor_box_number);
void buzzer_int();
void led_int();

float max_spo2_reading(); 
void max_setup();
float max_temp_reading();
void addition_int();
void lcd_int();
void rtc_int();
float max_heartrate_reading();
void Robot_movement_int();
void stopMoving();
void turnRight();
void turnLeft();
void moveBackward();
void moveForward();
void Moving_On_Track();
int getMinutes();
int getHours();

void Buzzer_Indecate_run(void);
void  Led_Indecate_run(led_box_number_t led_box_number);


// max sensor section
#define MAX_BRIGHTNESS 255
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data
and red led data in 32 - bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.
Samples become 16 - bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100]; //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100]; //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read



#define RTC_ADDRESS 0x68 
#define MAX30102_ADDRESS 0x57 

//lcd section 
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // Initialize the LCD display with the corresponding pin numbers


void setup() {


  Serial1.begin(115200);
  Serial.begin(115200);
  rtc_int();
  lcd_int();
  addition_int();
  max_setup();


  lcd.clear();



  // Print a message on the LCD to indicate successful initialization
  lcd.setCursor(0, 1);
  lcd.print("MAX30102 initialized");
  delay(2000);
  lcd.clear();
  buzzer_int();
  led_int();
}

// time for medcines
char FiristMedcineTime_Hour = 1;
char FiristMedcineTime_Minutes = 15;


int Max_reading=0.0;
void loop() {




  // Welcome start Robot
  lcd.setCursor(5, 1);
  lcd.print("Welcome");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Name : Max ");
  lcd.setCursor(0, 1);
  lcd.print("Date : 21/6/2023 Wed");
  lcd.setCursor(0, 2);
  lcd.print("Room No : 1");
  lcd.setCursor(0, 3);
  lcd.print("Bed  No : 1");




  // getting time to move from base to patient
  //reding hour and minutes
  int minutes = getMinutes();
  int hours = getHours();

  // compare between current time and desired time

  Serial1.write(GO_read_call_statuse);
  while (difftime(time(NULL), start_time) < 2.0) {
          // Code to be executed during the 3 seconds
         if (Serial1.available()) {
          int receivedInt;
          receivedInt = Serial1.read(); 
     Get_call_statuse = static_cast<char>(receivedInt);
     
    Get_call_statuse = Serial1.read();
  }
         
      
        }
  

  if ( (Get_call_statuse == 'a' ) || (FiristMedcineTime_Hour == hours && FiristMedcineTime_Minutes == minutes)) {
    // start moving
    //red data from line follower sensor
    while(done){
      s1 = digitalRead(lineSensorPin1);  //Left Most Sensor
    s2 = digitalRead(lineSensorPin2);  //Left Sensor
    s3 = digitalRead(lineSensorPin3);  //Middle Sensor
    s4 = digitalRead(lineSensorPin4);  //Right Sensor
    s5 = digitalRead(lineSensorPin5);  //Right Most Sensor



    // start moving to the track
    moveForward();
    delay(500);

    // start moving on track
    Moving_On_Track();
    }
    

    // indecate that robot is in its target
    if ( (s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0) )
    {
      Buzzer_Indecate_run();
      Led_Indecate_run(LED_BOX_FOR_MEDCINE_1);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Take Your Medcine ");
      lcd.setCursor(0, 1);
      lcd.print("Box No :");
      lcd.print(LED_BOX_FOR_MEDCINE_1 + 1);
      delay(5000);
      if (MEDCINE_IS_TAKED == sensor_box_read(SENSOR_BOX_FOR_MEDCINE_1)) {
        Serial1.write(1);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Medcine taked");

          Serial1.write(44); //indecate for sending the spo2 reading
          delay(10);
          Serial1.write(1);
        
        delay(500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Put Finger to Read");
        delay(1000);
        lcd.clear();
        lcd.setCursor(0, 0);

        
        // Display the temperature on the LCD
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
         Max_reading=0.0;
        
        while (difftime(time(NULL), start_time) < 10.0) {
          // Code to be executed during the 3 seconds
         Max_reading= max_temp_reading();
         
      
        }
          lcd.print(Max_reading+5.4, 1); // Print the temperature with 1 decimal place
          Serial1.write(33); //indecate for sending the Temp reading
          delay(10);
       Serial1.write(Max_reading);

        // display HR
        lcd.clear();
        lcd.setCursor(0, 0);
      lcd.print("HR: ");
        
        
        while (difftime(time(NULL), start_time) < 10.0) {
          // Code to be executed during the 3 seconds
         Max_reading= max_heartrate_reading();
         
      
        }
          lcd.print(Max_reading, 1); // Print the temperature with 1 decimal place
          Serial1.write(22); //indecate for sending the HR reading
          delay(10);
        Serial1.write(Max_reading);

        // spo2
        lcd.clear();
        lcd.setCursor(0, 0);
      lcd.print("SPO2: ");
        
        
        while (difftime(time(NULL), start_time) < 20.0) {
          // Code to be executed during the 3 seconds
         Max_reading= max_spo2_reading();
         
      
        }
          lcd.print(Max_reading, 1); // Print the temperature with 1 decimal place
          Serial1.write(11); //indecate for sending the spo2 reading
          delay(10);
          Serial1.write(Max_reading);
          
      }
        
      
      
      else {
        Serial1.write(0);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Medcine not taked");

          Serial1.write(44); //indecate for sending the spo2 reading
          delay(10);
          Serial1.write(0);
        
        // go home
        
         turnRight();
           s1 = digitalRead(lineSensorPin1);  //Left Most Sensor
    s2 = digitalRead(lineSensorPin2);  //Left Sensor
    s3 = digitalRead(lineSensorPin3);  //Middle Sensor
    s4 = digitalRead(lineSensorPin4);  //Right Sensor
    s5 = digitalRead(lineSensorPin5);  //Right Most Sensor
    while(s3!=0){
      
      turnRight();
           s1 = digitalRead(lineSensorPin1);  //Left Most Sensor
    s2 = digitalRead(lineSensorPin2);  //Left Sensor
    s3 = digitalRead(lineSensorPin3);  //Middle Sensor
    s4 = digitalRead(lineSensorPin4);  //Right Sensor
    s5 = digitalRead(lineSensorPin5);  //Right Most Sensor
    }

    
        done=1;
          while(done){
      s1 = digitalRead(lineSensorPin1);  //Left Most Sensor
    s2 = digitalRead(lineSensorPin2);  //Left Sensor
    s3 = digitalRead(lineSensorPin3);  //Middle Sensor
    s4 = digitalRead(lineSensorPin4);  //Right Sensor
    s5 = digitalRead(lineSensorPin5);  //Right Most Sensor



    // start moving to the track

    // start moving on track
    Moving_On_Track();
    }

    

      }


    }


  
  else
  {

  }



  // Read the time from the RTC module
  /*
    int seconds = getSeconds();
    int minutes = getMinutes();
    int hours = getHours();

    // Display the time on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    if (hours < 10) lcd.print("0"); // Add leading zero if hours is single digit
    lcd.print(hours);
    lcd.print(":");
    if (minutes < 10) lcd.print("0"); // Add leading zero if minutes is single digit
    lcd.print(minutes);
    lcd.print(":");
    if (seconds < 10) lcd.print("0"); // Add leading zero if seconds is single digit
    lcd.print(seconds);
  */

  // Read human temperature from the MAX30102 sensor

  /*
    // Display the temperature on the LCD
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(max_temp_reading()+5.4, 1); // Print the temperature with 1 decimal place
  */


  /*
    //  Display spo2
    lcd.setCursor(0, 2);
    lcd.print("SPO2: ");
    lcd.print(max_spo2_reading(), 1); // Print the temperature with 1 decimal place
    Serial.println(max_spo2_reading(),DEC);
  */
  /*
    // heartrate
    lcd.setCursor(0, 2);
    lcd.print("HR: ");
    lcd.print(max_heartrate_reading(), 1); // Print the temperature with 1 decimal place

    Serial.println(max_heartrate_reading());
  */

  // delay(1000); // Delay for 1 second

}
else{
  // do nothing
}
}



void rtc_int() {
  if (!isTimeSet()) {
    // Set the time on the RTC module
    setTime(1, 12, 0); // Set the time to 02:06:00
  }
  //setTime(1, 12, 0); // Set the time to 02:06:00
}

void setTime(int hours, int minutes, int seconds) {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0); // Control register address to set the time
  Wire.write(decToBcd(seconds)); // Seconds
  Wire.write(decToBcd(minutes)); // Minutes
  Wire.write(decToBcd(hours)); // Hours (in 24-hour format)
  Wire.endTransmission();
}
int getSeconds() {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0); // Control register address to read the time
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcdToDec(Wire.read());
}
int getMinutes() {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(1); // Control register address to read the time
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcdToDec(Wire.read());
}
int getHours() {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(2); // Control register address to read the time
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDRESS, 1);
  return bcdToDec(Wire.read());
}
bool isTimeSet() {
  Wire.beginTransmission(RTC_ADDRESS);
  Wire.write(0); // Control register address to read the seconds
  Wire.endTransmission();
  Wire.requestFrom(RTC_ADDRESS, 1);
  int seconds = bcdToDec(Wire.read());
  return (seconds != 0); // If the seconds are not zero, the time has been set before
}

byte decToBcd(byte value) {
  return ((value / 10 * 16) + (value % 10));
}

byte bcdToDec(byte value) {
  return ((value / 16 * 10) + (value % 16));
}





void max_setup() {
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {

    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

}

float max_temp_reading() {
  float temperature = particleSensor.readTemperature();
  return temperature;
}

void addition_int() {
  Wire.begin(); // Initialize I2C communication

}

void lcd_int() {

  lcd.begin(20, 4);  // Initialize the LCD display with 16 columns and 2 rows

}



float max_spo2_reading() {


  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  //Continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second


  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
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
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //send samples and calculation result to terminal program through UART
  }
  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  return spo2;
}

float max_heartrate_reading() {

  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  if (irValue < 50000)
  {
    beatAvg = 0.0;
  }


  return beatAvg;
}


void Robot_movement_int() {

  pinMode(R1, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(L2, OUTPUT);

  pinMode(lineSensorPin1, INPUT);
  pinMode(lineSensorPin2, INPUT);
  pinMode(lineSensorPin3, INPUT);
  pinMode(lineSensorPin4, INPUT);
  pinMode(lineSensorPin5, INPUT);
}



void moveForward()
{
  analogWrite(R1, SPEED);
  analogWrite(L1, 0);
  analogWrite(R2, SPEED);
  analogWrite(L2, 0);
}

void moveBackward() {
  analogWrite(R1, 0);
  analogWrite(L1, SPEED);
  analogWrite(R2, 0);
  analogWrite(L2, SPEED);
}
void turnLeft() {
  analogWrite(R1, 0);
  analogWrite(L1, SPEED);
  analogWrite(R2, SPEED);
  analogWrite(L2, 0);
}
void turnRight() {
  analogWrite(R1, SPEED);
  analogWrite(L1, 0);
  analogWrite(R2, 0);
  analogWrite(L2, SPEED);
}
void stopMoving(void)
{
  analogWrite(R1, 0);
  analogWrite(L1, 0);
  analogWrite(R2, 0);
  analogWrite(L2, 0);
}
void Moving_On_Track() {
  if (  (s2 == 1) && (s3 == 0) && (s4 == 1)  )
  {
    moveForward();
    delay(250);
  }
  if ((s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 1))
  {
    moveForward();
    delay(250);

  }

  if (  (s2 == 0) && (s3 == 0) && (s4 == 1)  )
  {
    turnLeft();
  }
  if (  (s2 == 1) && (s3 == 0) && (s4 == 0)  )
  {
    turnRight();
  }
  if ( (s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0)  )
  {
    moveForward();
    delay(500);
    turnRight();
    delay(500);
  }
  if ( (s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 1)  )
  {
    moveForward();
    delay(500);
    turnLeft();
    delay(500);
  }
  if ( (s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 0) && (s5 == 0)  )
  {
    moveForward();
    delay(500);
    turnRight();
    delay(500);
  }
  if ( (s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1)  )
  {
    moveForward();
    delay(500);
    turnLeft();
    delay(500);
  }

  if ( (s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 0)  )
  {
    turnRight();
    delay(500);
  }
  if ( (s1 == 0) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 1)  )
  {
    turnLeft();
    delay(500);
  }


  if ( (s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 0) && (s5 == 0)  )
  {
    turnRight();
    delay(500);
  }
  if ( (s1 == 0) && (s2 == 0) && (s3 == 1) && (s4 == 1) && (s5 == 1)  )
  {
    turnLeft();
    delay(500);
  }
  if ( (s1 == 1) && (s2 == 0) && (s3 == 1) && (s4 == 1) && (s5 == 1)  )
  {
    turnLeft();
    delay(500);
  }
  if ( (s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 0) && (s5 == 1)  )
  {
    turnRight();
    delay(500);
  }

  if ( (s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0) )
  {
    stopMoving();
    done=0;
  }
}

void buzzer_int() {
  pinMode(buzzerPin, OUTPUT);

}
void led_int() {

  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
}
void Buzzer_Indecate_run() {


  digitalWrite(buzzerPin, HIGH);
  delay(1000);
  // Turn off the buzzer for 1 second
  digitalWrite(buzzerPin, LOW);
  delay(1000);

}
void  Led_Indecate_run(led_box_number_t led_box_number) {
  switch (led_box_number) {
    case LED_BOX_FOR_MEDCINE_1:
      digitalWrite(led1Pin, HIGH);
      delay(1000);
      // Turn off the led for 1 second
      digitalWrite(led1Pin, LOW);
      delay(1000);
      break;
    case LED_BOX_FOR_MEDCINE_2:
      digitalWrite(led2Pin, HIGH);
      delay(1000);
      // Turn off the buzzer for 1 second
      digitalWrite(led2Pin, LOW);
      delay(1000);
      break;
    case LED_BOX_FOR_MEDCINE_3:
      digitalWrite(led3Pin, HIGH);
      delay(1000);
      // Turn off the buzzer for 1 second
      digitalWrite(led3Pin, LOW);
      delay(1000);
      break;
    default:
      break;



  }
}

int sensor_box_read(sensor_box_number_t sensor_box_number) {
  if (SENSOR_BOX_FOR_MEDCINE_1 == sensor_box_number) {
    int sensor1read = digitalRead(sensor1Pin);
    if (!sensor1read ) {
      return 0;
    } else {
      return 1;
    }

  }

  if (SENSOR_BOX_FOR_MEDCINE_2 == sensor_box_number) {
    int sensor2read = digitalRead(sensor2Pin);
    if (!sensor2read ) {
      return 0;
    } else {
      return 1;
    }

  }
  if (SENSOR_BOX_FOR_MEDCINE_3 == sensor_box_number) {
    int sensor3read = digitalRead(sensor3Pin);
    if (!sensor3read ) {
      return 0;
    } else {
      return 1;
    }

  }


}
