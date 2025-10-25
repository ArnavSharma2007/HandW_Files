#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <TinyGPSPlus.h>
#include <WiFiNINA.h>
#include <Firebase_Arduino_WiFiNINA.h>

// --------------------- WIFI & FIREBASE CONFIG ---------------------
const char* ssid = "ARNAV";
const char* password = "00000000";
const char* firebaseHost = "handwband-default-rtdb.firebaseio.com"; 
const char* FIREBASE_API_KEY = "AIzaSyAGdi0EIOvXnuG9Q8azmN3SHlFLbRZ4tPU"; 

FirebaseData fbdo;

// --------------------- SENSORS ---------------------
MAX30105 particleSensor;
Adafruit_MLX90614 mlx;

TinyGPSPlus gps;
#define GPS_BAUD 9600

#define MAX_SAMPLES 25
uint32_t irBuffer[MAX_SAMPLES];
uint32_t redBuffer[MAX_SAMPLES];

// Rolling average buffers
#define HR_BUFFER_SIZE 5
int hrBuffer[HR_BUFFER_SIZE] = {0};
int spo2Buffer[HR_BUFFER_SIZE] = {0};
int hrIndex = 0, spo2Index = 0;
int hrCount = 0, spo2Count = 0;
int32_t spo2, heartRate;
int8_t validSPO2, validHeartRate;

// Temperature
#define NUM_TEMP_SAMPLES 10
#define TEMP_OFFSET 2.9
double lastTemp = 36.5;

// Timing
unsigned long lastSend = 0;
const unsigned long sendInterval = 5000;

// --------------------- BUZZER & BUTTON ---------------------
#define BUZZER_PIN 6
#define BUTTON_PIN 7
bool manualOverride = false;
unsigned long overrideStartTime = 0;
const unsigned long overrideDuration = 10000;
unsigned long lastButtonCheck = 0;
const unsigned long buttonCheckInterval = 50;

// --------------------- SETUP ---------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  // MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
  } else {
    particleSensor.setup();
  }

  // MLX90614
  if (!mlx.begin()) {
    Serial.println("MLX90614 not found!");
  }

  // GPS
  Serial1.begin(GPS_BAUD);

  // WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

  Firebase.begin(firebaseHost, FIREBASE_API_KEY, ssid, password);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase ready!");

  // Buzzer and button
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

// --------------------- LOOP ---------------------
void loop() {
  // GPS update
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --------------------- Button Handling ---------------------
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  // Check manual override expiry
  if (manualOverride && millis() - overrideStartTime > overrideDuration) {
    manualOverride = false;
  }

  if (buttonPressed) {
    static unsigned long lastPress = 0;
    if (millis() - lastPress > 1000) {
      lastPress = millis();

      if (manualOverride) {
      } else if (!manualOverride && buttonPressed) {
        if (Firebase.getString(fbdo, "/sensorData/state") && fbdo.stringData() == "EMERGENCY") {
          manualOverride = true;
          overrideStartTime = millis();
          Firebase.setString(fbdo, "/sensorData/state", "OK");
          digitalWrite(BUZZER_PIN, LOW);
          Serial.println("=False alarm! Override activated for 10 seconds.");
        } else {
          Firebase.setString(fbdo, "/sensorData/state", "EMERGENCY");
          Serial.println("Emergency triggered!");
        }
      }
    }
  }

  // --------------------- Data Collection ---------------------
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // MAX30105 burst read
    for (byte i = 0; i < MAX_SAMPLES; i++) {
      while (!particleSensor.available())
        particleSensor.check();

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }
    maxim_heart_rate_and_oxygen_saturation(irBuffer, MAX_SAMPLES, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Update rolling buffers
    if (validHeartRate) {
      hrBuffer[hrIndex] = heartRate;
      hrIndex = (hrIndex + 1) % HR_BUFFER_SIZE;
      if (hrCount < HR_BUFFER_SIZE) hrCount++;
    }
    if (validSPO2) {
      spo2Buffer[spo2Index] = spo2;
      spo2Index = (spo2Index + 1) % HR_BUFFER_SIZE;
      if (spo2Count < HR_BUFFER_SIZE) spo2Count++;
    }

    // Rolling averages
    int avgHR = 65;
    int avgSpO2 = 90;
    if (hrCount > 0) {
      int sum = 0;
      for (int i = 0; i < hrCount; i++) sum += hrBuffer[i];
      avgHR = sum / hrCount;
    }
    if (spo2Count > 0) {
      int sum = 0;
      for (int i = 0; i < spo2Count; i++) sum += spo2Buffer[i];
      avgSpO2 = sum / spo2Count;
    }

    // MLX90614 temperature
    double tempSum = 0;
    for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
      tempSum += mlx.readObjectTempC();
      delay(50);
    }
    double wristTemp = tempSum / NUM_TEMP_SAMPLES;
    double estimatedCoreTemp = wristTemp + TEMP_OFFSET;
    if (!isnan(estimatedCoreTemp)) lastTemp = estimatedCoreTemp;

    // --------------------- EMERGENCY LOGIC ---------------------
    String state = "OK";
    if (!manualOverride) { // only trigger auto emergency if not overridden
      if (avgHR < 50 || avgHR > 120) state = "EMERGENCY";
      if (!isnan(lastTemp) && (lastTemp < 35 || lastTemp > 39)) state = "EMERGENCY";
    }

    // --------------------- BUZZER CONTROL ---------------------
    if (state == "EMERGENCY") {
      digitalWrite(BUZZER_PIN, LOW);
    } else {
      digitalWrite(BUZZER_PIN, HIGH);
    }

    // --------------------- SEND TO FIREBASE ---------------------
    if (WiFi.status() == WL_CONNECTED) {
      Firebase.setFloat(fbdo, "/sensorData/HR", avgHR != -999 ? (float)avgHR : -999.0);
      Firebase.setFloat(fbdo, "/sensorData/SpO2", avgSpO2 != -999 ? (float)avgSpO2 : -999.0);
      Firebase.setFloat(fbdo, "/sensorData/BodyTemp", !isnan(lastTemp) ? (float)lastTemp : -999.0);
      Firebase.setFloat(fbdo, "/sensorData/GPS_lat", gps.location.isValid() ? (float)gps.location.lat() : 0.0);
      Firebase.setFloat(fbdo, "/sensorData/GPS_lng", gps.location.isValid() ? (float)gps.location.lng() : 0.0);
      Firebase.setString(fbdo, "/sensorData/state", state);
    }

    // Serial print
    Serial.print("HR: "); Serial.print(avgHR);
    Serial.print(", SpO2: "); Serial.print(avgSpO2);
    Serial.print(", Temp: "); Serial.print(lastTemp); Serial.print(" C");
    Serial.print(", GPS: "); Serial.print(gps.location.isValid() ? gps.location.lat() : 0.0);
    Serial.print(", "); Serial.print(gps.location.isValid() ? gps.location.lng() : 0.0);
    Serial.print(", State: "); Serial.println(state);
  }
}
