#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <FirebaseESP8266.h>
#endif

// Include the DHT sensor library
#include <DHT.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "vfsrvgrweg"
#define WIFI_PASSWORD "tarekadel12345....=ALIBADEEAHMEDKAMEL*20-201902719_O6U"

/* 2. Define the API Key */
#define API_KEY "AIzaSyCnrNZT1l47vCHHi1FzbTwlPa2SiVIHhCI"

/* 3. Define the RTDB URL */
#define DATABASE_URL "nodeesp-b0df4-default-rtdb.europe-west1.firebasedatabase.app"

/* 4. Define the user Email and password */
#define USER_EMAIL "pootyoop14@gmail.com"
#define USER_PASSWORD "pootyoop"

// Define the DHT sensor pin
#define DHT_PIN 2
#define DHT_TYPE DHT11

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

DHT dht(DHT_PIN, DHT_TYPE);

void setup()
{
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi. IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;

  Firebase.begin(&config, &auth);
}
int i=0;
void loop()
{
  /*
  // Read temperature and humidity from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("Failed to read data from DHT sensor!");
    return;
  }

  // Store the temperature and humidity values in Firebase
  if (Firebase.setFloat(fbdo, "/temperature", temperature))
  {
    Serial.println("Temperature data sent to Firebase.");
  }
  else
  {
    Serial.println("Failed to send temperature data to Firebase.");
    Serial.println(fbdo.errorReason());
  }

  if (Firebase.setFloat(fbdo, "/humidity", humidity))
  {
    Serial.println("Humidity data sent to Firebase.");
  }
  else
  {
    Serial.println("Failed to send humidity data to Firebase.");
    Serial.println(fbdo.errorReason());
  }
*/
/*
if (Serial.available()) {
  
      float receivedVariable = Serial.parseFloat();
Firebase.setFloat(fbdo, "/counter", receivedVariable);
delay(1000);


}
*/
Firebase.setInt(fbdo, "/counter", i);
i++;
  delay(1000); // Wait for 5 seconds before sending the next set of data
}
