#include "Arduino_BMI270_BMM150.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define STEP_THRESHOLD 1.5  // Threshold for step detection (tweak as needed)

int stepCount = 0;
float totalDistance = 0.0;  // in meters
float strideLength = 0.78;  // average stride length in meters (tweak as needed)

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.display();
  delay(2000); // Pause for 2 seconds

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Step Counter");
  display.display();

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);

    // Basic step detection
    if (sqrt(x * x + y * y + z * z) > STEP_THRESHOLD) {
      stepCount++;
      totalDistance = stepCount * strideLength;

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Steps: ");
      display.println(stepCount);
      display.print("Distance: ");
      display.print(totalDistance);
      display.println(" m");
      display.display();

      delay(300);  // Simple debounce delay
    }
  }
}
