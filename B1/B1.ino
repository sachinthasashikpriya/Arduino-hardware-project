#include <pro2_inferencing.h>


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"   // MAX3010x library
#include "heartRate.h"  // Heart rate calculating algorithm
#include "Arduino_BMI270_BMM150.h"
#include <TinyGPS++.h>



// MAX3010x and heart rate variables
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define OLED1_ADDR 0x3C
#define OLED2_ADDR 0x3D

// Step counter parameters
#define STEP_THRESHOLD 1.2
int stepCount = 0;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Variables to store previous latitude and longitude
double previousLat = 0.0;
double previousLon = 0.0;
double totalDistance = 0.0;

// Timer variables
unsigned long previousMillis = 0;
const long interval = 3000;

// Battery voltage and percentage variables
const int analogPin = A0;
const float resistorValue = 10000.0;
const int numReadings = 5;
float voltage;
int percentage;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting up...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 initialization failed!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!display1.begin(SSD1306_SWITCHCAPVCC, OLED1_ADDR)) {
    Serial.println(F("SSD1306 display1 allocation failed"));
    while (1);
  }
  display1.display();
  delay(2000);
  display1.clearDisplay();

  if (!display2.begin(SSD1306_SWITCHCAPVCC, OLED2_ADDR)) {
    Serial.println(F("SSD1306 display2 allocation failed"));
    while (1);
  }
  display2.display();
  delay(2000);
  display2.clearDisplay();

  Serial1.begin(9600);

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  Serial.println("GPS Distance Calculation in Meters");
}

void loop() {
  measureStepCount();
  calculateDistanceFromGPS();
  updateHeartRate();
  measureBattery();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateDisplay1();
  }

  updateDisplay2();
}

void updateHeartRate() {
  long irValue = particleSensor.getIR();

  // Adjust the IR threshold for hand-worn sensors
  if (irValue > 50000) { // Increased threshold for hand-worn sensor
    if (checkForBeat(irValue)) {
      long now = millis();
      long delta = now - lastBeat;
      lastBeat = now;
      beatsPerMinute = 60 / (delta / 1000.0);

      // Adjust the valid heart rate range
      if (beatsPerMinute > 40 && beatsPerMinute < 180) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        long total = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          total += rates[x];
        }
        beatAvg = total / RATE_SIZE;
      }
    }
  } else {
    // Set a default average heart rate if IR value is below threshold
    beatAvg = 75; // Typical resting heart rate
  }
}

void measureStepCount() {
  static unsigned long lastStepTime = 0;
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    if (sqrt(x * x + y * y + z * z) > STEP_THRESHOLD) {
      if (millis() - lastStepTime > 300) {
        stepCount++;
        lastStepTime = millis();
      }
    }
  }
}

void calculateDistanceFromGPS() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    if (gps.location.isUpdated()) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();

        if (previousLat != 0.0 && previousLon != 0.0) {
          double distance = calculateDistance(previousLat, previousLon, currentLat, currentLon);
          totalDistance += distance;
          Serial.print("Distance: ");
          Serial.print(distance, 6);
          Serial.print(" m, Total Distance: ");
          Serial.print(totalDistance, 6);
          Serial.println(" m");
        }

        previousLat = currentLat;
        previousLon = currentLon;
      }
    }
  }
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double radius = 6371000.0;  // Radius of Earth in meters
  return radius * c;
}



void measureBattery() {
  long sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(analogPin);
    delay(5);
  }
  int avgSensorValue = sum / numReadings;

  float voltageDividerOutput = avgSensorValue * (3.3 / 1023.0);
  voltage = voltageDividerOutput * 2;

  percentage = map(voltage * 100, 300, 420, 0, 100);
  percentage = constrain(percentage, 0, 100);
}

void updateDisplay1() {
  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);

  display1.setCursor(0, 0);
  display1.print("Date: ");
  if (gps.date.isValid()) {
    display1.print(gps.date.year());
    display1.print("/");
    display1.print(gps.date.month());
    display1.print("/");
    display1.println(gps.date.day());
  } else {
    display1.println("N/A");
  }

  display1.setCursor(0, 16);
  display1.print("Time: ");
  if (gps.time.isValid()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    hour += 5;
    minute += 30;
    if (minute >= 60) {
      minute -= 60;
      hour += 1;
    }
    if (hour >= 24) {
      hour -= 24;
    }
    if (hour < 10) display1.print("0");
    display1.print(hour);
    display1.print(":");
    if (minute < 10) display1.print("0");
    display1.print(minute);
    display1.print(":");
    if (gps.time.second() < 10) display1.print("0");
    display1.print(gps.time.second());
    display1.print(".");
    if (gps.time.centisecond() < 10) display1.print("0");
    display1.println(gps.time.centisecond());
  } else {
    display1.println("N/A");
  }

  display1.setCursor(0, 32);
  display1.print("Location: ");
  if (gps.location.isValid()) {
    display1.print(gps.location.lat(), 6);
    display1.print(", ");
    display1.println(gps.location.lng(), 6);
  } else {
    display1.println("N/A");
  }

  display1.setCursor(0, 48);
  display1.print("Voltage: ");
  display1.print(voltage);
  display1.print(" V, Battery: ");
  display1.print(percentage);
  display1.println(" %");

  display1.display();
}

void updateDisplay2() {
  display2.clearDisplay();
  display2.setTextSize(2);
  display2.setTextColor(SSD1306_WHITE);

  display2.setCursor(0, 0);
  display2.print("BPM  : ");
  display2.println(beatAvg);

  display2.setCursor(0, 16);
  display2.print("Steps: ");
  display2.println(stepCount);

  display2.setCursor(0, 32);
  display2.print("Distance: ");
  display2.print(totalDistance, 2);
  display2.println(" m");

  display2.display();
}
