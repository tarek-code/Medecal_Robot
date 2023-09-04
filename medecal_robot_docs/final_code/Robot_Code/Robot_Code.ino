#include <Wire.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

//function decleration 
void max_setup();
float max_temp_reading();
void addition_int();
void lcd_int();
void rtc_int();
float max_heartrate_reading();

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
and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. 
Samples become 16-bit data.
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

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // Initialize the LCD display with the corresponding pin numbers


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

}

void loop() {
  
  
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

if (Serial1.available()) {
    // Read data from NodeMCU
    int value = Serial1.parseInt();
    
    // Print received value
     lcd.println(value);
    Serial.println(value);
  }
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





void max_setup(){
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
 particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate,pulseWidth, adcRange); //Configure sensor with these settings

 particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
 particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
 
}
float max_temp_reading(){
  float temperature = particleSensor.readTemperature();
  return temperature;
}

void addition_int(){
  Wire.begin(); // Initialize I2C communication
  
}

void lcd_int(){
  
  lcd.begin(20, 4);  // Initialize the LCD display with 16 columns and 2 rows

}

void rtc_int(){
  if (!isTimeSet()) {
    // Set the time on the RTC module
    setTime(17, 17, 0); // Set the time to 02:06:00
  }

  
}

float max_spo2_reading(){
  
  
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

float max_heartrate_reading(){
  
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
  beatAvg=0.0;
}
 

return beatAvg;
  }
