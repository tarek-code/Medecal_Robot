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



/* 1. Define the WiFi credentials */
#define WIFI_SSID "Bebo"
#define WIFI_PASSWORD "14102020bebololo"

/* 2. Define the API Key */
#define API_KEY "AIzaSyBK0JtSVfPLujIPbZlPQN0PdZVPfWQ5NUA"

/* 3. Define the RTDB URL */
#define DATABASE_URL "fir-auth-app-4a0e7-default-rtdb.europe-west1.firebasedatabase.app"

/* 4. Define the user Email and password */
#define USER_EMAIL "pootyoop14@gmail.com"
#define USER_PASSWORD "pootyoop"



// Define Firebase Data object
FirebaseData fbdo;
FirebaseJson json;

FirebaseAuth auth;
FirebaseConfig config;



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
int Reading=0;

void loop()
{

if (Serial.available()) {
  i= Serial.read();
  switch(i){
    case 11:
      int g=1;
      while(g){
         Reading= Serial.read();
         Firebase.setInt(fbdo, "/Temp/0/value", Reading);
         g=0;
      }
    break;

    case 22:
      int g=1;
      while(g){
         Reading= Serial.read();
         Firebase.setInt(fbdo, "/hart/0/value", Reading);
         g=0;
      }
    break;
    case 33:
      int g=1;
      while(g){
         Reading= Serial.read();
         Firebase.setInt(fbdo, "/spo2/0/value", Reading);
         g=0;
      }
    break;
    case 44:
      int g=1;
      while(g){
         Reading= Serial.read();
         Firebase.setInt(fbdo, "/midcin/0/isTaken", Reading);
         g=0;
      }
    break;
    
  }
    
    
}



/*
// Read the int value from Firebase
int i=5;
  if (Firebase.getInt(fbdo, "/midcin/Medcine_Hour"))
  {
     i = fbdo.intData();
     //sending data to mega 
     
      Serial.println(i);
    
    
  }
  */
    
    
    
    
    /*
     * 
    // Store the value in the "/counter" path in Firebase
    if (Firebase.setInt(fbdo, "/counter", i))
    {
      Serial.println("Counter value updated in Firebase.");
    }
      */

  /*
  // Read the float value from Firebase
  float i=0.0;
  if (Firebase.getInt(fbdo, "/Pass"))
  {
     i = fbdo.floatData();
  }

    // Store the value in the "/counter" path in Firebase
    if (Firebase.setFloat(fbdo, "/counter", i))
    {
      Serial.println("Counter value updated in Firebase.");
    }
    */
    /*
String name;
    // Read the String value from Firebase
  if (Firebase.getString(fbdo, "/Pass"))
  {
    name = fbdo.stringData();
  }

    // Store the value in the "/counter" path in Firebase
    if (Firebase.setString(fbdo, "/counter", name))
    {
      Serial.println("Counter value updated in Firebase.");
    }
*/




  
  

  
/*
char character;

// Read the String value from Firebase
if (Firebase.getString(fbdo, "/Pass"))
{
  String stringValue = fbdo.stringData();
  if (stringValue.length() > 0)
  {
    character = stringValue.charAt(0);
    Serial.write(character);
  }
}

// Store the value in the "/counter" path in Firebase
if (Firebase.setString(fbdo, "/counter", String(character)))
{
  Serial.println("Counter value updated in Firebase.");
}
*/

/*
// send Sreing to firebase

 
String name="Tarek Adel";
// write int to firebase 
Firebase.setString(fbdo, "/counter", name);
*/

//still worke on it 
/*
if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    
    
    
    // Send data to Firebase
    Firebase.setString(fbdo, "/counter", receivedData);
    
    
  }
*/

  
/*
// send int value to firebase
int i=0;
Firebase.setInt(fbdo, "/counter", i);
i++;
*/


/*
// recive int value from mega & sending int to firebase
if (Serial.available()) 
{
    int receivedNumber = Serial.parseInt(); // Read the received integer from Serial
    Firebase.setInt(fbdo, "/counter", receivedNumber);
    Serial.print("Received number: ");
    Serial.println(receivedNumber);
  }
*/


/*
// send float to firebase

//float i=0.0;
// write int to firebase 
Firebase.setFloat(fbdo, "/counter", i);
i+=1.8;
*/

/*
// recive float value from mega & sending int to firebase
if (Serial.available()) 
{
    float receivedNumber = Serial.parseFloat(); // Read the received integer from Serial
    Firebase.setFloat(fbdo, "/counter", receivedNumber);
    Serial.print("Received number: ");
    Serial.println(receivedNumber);
  }
*/

  delay(1000); // Wait for 5 seconds before sending the next set of data
}
