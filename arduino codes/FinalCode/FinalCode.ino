#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"   // MAX3010x library
#include "heartRate.h"  // Heart rate calculating algorithm
#include "Arduino_BMI270_BMM150.h"
#include <TinyGPS++.h>
#include <pro2_inferencing.h>
#define EI_CLASSIFIER_SENSOR EI_CLASSIFIER_SENSOR_ACCELEROMETER

#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arduino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

#include <ArduinoBLE.h>

// MAX3010x and heart rate variables
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Global Variables
float threshold = 0.0;
float confidence = 0.8;
bool flag = false;
String Label;

//distance treadshold
const double distanceThreshold = 1.0; // 1 meter threshold

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

// Bluetooth service activation
BLEService fitness_service("e267751a-ae76-11eb-8529-0242ac130003");

BLEIntCharacteristic exercise("2A19", BLERead | BLENotify);
BLEByteCharacteristic start("19b10012-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite);
BLEByteCharacteristic pause("6995b940-b6f4-11eb-8529-0242ac130003", BLERead | BLEWrite);

BLEDevice central;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Starting up...");

    // Initialize MAX3010x sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 initialization failed!");
        while (1);
    }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);

    // Initialize IMU
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // Initialize OLED displays
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

    // Initialize GPS
    Serial1.begin(9600);

    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1);
    }

    BLE.setLocalName("Get-Fit");
    BLE.setAdvertisedService(fitness_service);

    start.setValue(0);
    pause.setValue(0);

    fitness_service.addCharacteristic(exercise);
    fitness_service.addCharacteristic(start);
    fitness_service.addCharacteristic(pause);

    BLE.addService(fitness_service);
    BLE.advertise();

    Serial.println("Bluetooth device active, waiting for connections...");

    while (1) {
        central = BLE.central();
        if (central) {
            Serial.print("Connected to central: ");
            Serial.println(central.address());
            digitalWrite(LED_BUILTIN, HIGH);
            break;
        }
    }

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");

    Serial.println("GPS Distance Calculation in Meters");
}
/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop() {
    if (central.connected()) {
        start.read();
        pause.read();
    }
    if (start.value()) {
        flag = true;
        start.setValue(0);
    }
    if (pause.value()) {
        flag = false;
        pause.setValue(0);
    }

    measureStepCount();
    calculateDistanceFromGPS();
    updateHeartRate();
    measureBattery();

    if (flag == true) {
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 700);
            IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

            for (int i = 0; i < 3; i++) {
                if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                    buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
                }
            }

            buffer[ix + 0] *= CONVERT_G_TO_MS2;
            buffer[ix + 1] *= CONVERT_G_TO_MS2;
            buffer[ix + 2] *= CONVERT_G_TO_MS2;

            delayMicroseconds(next_tick - micros());
        }

        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            Serial.println("Failed to create signal from buffer");
            return;
        }

        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            Serial.println("Failed to run classifier");
            return;
        }

        Serial.println("Predictions:");
        Serial.println("DSP: " + String(result.timing.dsp) + " ms, Classification: " + String(result.timing.classification) + " ms, Anomaly: " + String(result.timing.anomaly) + " ms");

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            Serial.println(result.classification[ix].label + String(": ") + String(result.classification[ix].value));

            if (result.classification[ix].value > confidence) {
                Label = String(result.classification[ix].label);
                if (Label == "ARMCIRCLE" && central.connected()) {
                    Serial.println("idle");
                    exercise.writeValue(0);
                } else if (Label == "SQUAT" && central.connected()) {
                    exercise.writeValue(1);
                } else if (Label == "PUSHUP" && central.connected()) {
                    exercise.writeValue(2);
                }
            }
        }
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        updateDisplay1();
    }

    updateDisplay2();
}

void updateHeartRate() {
    long irValue = particleSensor.getIR();

    if (irValue > 50000) { // Increased threshold for hand-worn sensor
        if (checkForBeat(irValue)) {
            long now = millis();
            long delta = now - lastBeat;
            lastBeat = now;
            beatsPerMinute = 60 / (delta / 1000.0);

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
        beatAvg = 0; // Typical resting heart rate
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
            double currentLat = gps.location.lat();
            double currentLon = gps.location.lng();
            if (previousLat != 0.0 && previousLon != 0.0) {
                double distance = calculateDistance(previousLat, previousLon, currentLat, currentLon);
                if (distance > distanceThreshold) {
                   totalDistance += distance;
                   Serial.print("Distance: ");
                   Serial.print(distance, 6);
                   Serial.print(" m, Total Distance: ");
                   Serial.print(totalDistance, 6);
                   Serial.println(" m");
        }
            }
            previousLat = currentLat;
            previousLon = currentLon;
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
    double radius = 6371000.0;
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
    display1.setTextSize(2);
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

    /*display1.setCursor(0, 32);
    display1.print("Location: ");
    if (gps.location.isValid()) {
        display1.print(gps.location.lat(), 6);
        display1.print(",");
        display1.setCursor(0,48);
        display1.print("         ");
        display1.println(gps.location.lng(), 6);
    } else {
        display1.println("N/A");
    }*/
    display1.setCursor(0,32);
    display1.print("Battery:");
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
