// Include necessary libraries
#include <Wire.h>
#include "Adafruit_CCS811.h"
#include <MQ135.h>
#include "MICS6814.h"

// Define FSM states
enum State {
  IDLE,
  DETECTING_STATE,
  ALARM_STATE
};

State currentState = IDLE;
// Pin definitions
const int actuatorPin = 48; // Passive buzzer + fan pin
const int MQ135Pin = 13;    // MQ135 digital pin previously calibrated via the potentiometer
const int MiCS6814Pin_CO = 37;  // MiCS6814 analog pin
const int MiCS6814Pin_NO2 = 38; // MiCS6814 analog pin
const int MiCS6814Pin_NH3 = 45; // MiCS6814 analog pin


// Thresholds
const int MICS6814_CO_THRESHOLD = 3.5;     // Threshold for MiCS6814 CO in ppm : values retreived from WHO
const int MICS6814_NO2_THRESHOLD = 0.015;    // Threshold for MiCS6814 NO2 in ppm : values retreived from WHO
const int MICS6814_NH3_THRESHOLD = 0.57;    // Threshold for MiCS6814 NH3 : values retreived from oizom, a NH3 monitoring sensor manufacturer
//For the previously calibrated CO2 sensor MQ135 we put a threshold for digital state of 1000 ppm 
//(values retrieved from haut conseil de la santé publique)

// Variables
unsigned long previousMillis = 0;
const unsigned long interval = 5000; // 5 seconds interval
MQ135 mq135_sensor(MQ135Pin);
MICS6814 gas(MiCS6814Pin_CO, MiCS6814Pin_NO2, MiCS6814Pin_NH3);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // Initialize pins
  pinMode(actuatorPin, OUTPUT);
  pinMode(MQ135Pin, INPUT);
  pinMode(MiCS6814Pin_CO, INPUT);
  pinMode(MiCS6814Pin_NO2, INPUT);
  pinMode(MiCS6814Pin_NH3, INPUT);
  // Start in IDLE state
  currentState = IDLE;
  Serial.println("System in IDLE state");
}

void loop() {
  switch (currentState) {
    case IDLE:
      idleState();
      break;

    case DETECTING_STATE:
      detectingState();
      break;

    case ALARM_STATE:
      alarmState();
      break;
  }
}

// IDLE state: Initialize variables
void idleState() {
  Serial.println("Entering DETECTING state...");
  delay(1000); // Delay before transition
  currentState = DETECTING_STATE;
}

// DETECTING state: Read sensors and check thresholds
void detectingState() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read sensors
    int mq135Value = digitalRead(MQ135Pin); // Once the sensor is calibrated
    //int mq135Value =  mq135_sensor.getPPM(); //To calibrate for co2 values

    int mics6814Value_CO = gas.measure(CO);
    int mics6814Value_NO2 = gas.measure(NO2);
    int mics6814Value_NH3 = gas.measure(NH3);

    // Log sensor readings
    Serial.print("MQ135 Value: ");
    Serial.println(mq135Value);
    Serial.print("MiCS6814 CO: ");
    Serial.println(mics6814Value_CO);
    Serial.print("MiCS6814 NO2: ");
    Serial.println(mics6814Value_NO2);
    Serial.print("MiCS6814 NH3: ");
    Serial.println(mics6814Value_NH3);

    // Check thresholds
    if (mq135Value > 0 || mics6814Value_CO > MICS6814_CO_THRESHOLD ||
        mics6814Value_NO2 > MICS6814_NO2_THRESHOLD || mics6814Value_NH3 > MICS6814_NH3_THRESHOLD) {
      currentState = ALARM_STATE;
      Serial.println("Entering ALARM state...");
    }
    
  }
}

// ALARM state: Trigger alarm and notify via Serial Monitor
void alarmState() {
  digitalWrite(actuatorPin, HIGH); // Activate buzzer and fan
  Serial.println("ALARM: Hazardous gas levels detected!");
  
}
