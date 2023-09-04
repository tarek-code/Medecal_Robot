// Pin connected to the sensor output
const int sensorPin = 52;

// Threshold value for empty box detection


void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the sensor value
  int sensorValue = digitalRead(sensorPin);

  // Check if the box is empty or not
  if (!sensorValue ) {
    Serial.println("Box is not empty");
  } else {
    Serial.println("Box is empty");
  }

  //delay(1000); // Adjust the delay as needed
}
