/* --------------- LIBRARIES --------------- */
#include <DFRobot_MAX30102.h>
#include <Wire.h>
#include "rgb_lcd.h"

/* --------------- GLOBAL VARIABLES, CONSTANTS, AND FUNCTIONS --------------- */
const int numSamples = 100;

// Pulse Oximeter
DFRobot_MAX30102 pulseOximeter;
int32_t SPO2 = 0, heartRate = 0;
int8_t SPO2Valid = 0, heartValid = 0;

struct dataPulseOximeter {
  unsigned long timestamp;
  int32_t SPO2;
  int32_t heartRate;
};
int pulseIndex = 0;
dataPulseOximeter nightReadingsPulseOximeter[numSamples];

// Accelerometer
const int xPin = A3;
const int yPin = A2;
const int zPin = A1;
const int SAMPLES = 10;
const float RestVoltage = 1.641;  // Voltage at 0 gravity (g) at 3.3V
float xRest = 0;
float yRest = 0;
float zRest = 0;  // includes gravity

const float sensitivity = 0.3;    // Sensitivity in V/gravity (g) (1 g = 9.81 ms2)

// averages samples from ADXL334
float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(40);
  }
  //10-bit value converted to voltage between 0 and 5 volts
  return ((sum / (float)samples) * (5.0 / 1023.0));
}

struct dataAccelerometer {
  unsigned long timestamp;
  float magnitude;
};
int accelIndex = 0;
dataAccelerometer nightReadingsAccelerometer[numSamples];

// Sound Sensor
int readAveragedADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(40);
  }
  return (sum / samples);
}

struct dataSoundSensor {
  unsigned long timestamp;
  int sound;
};
int soundIndex = 0;
dataSoundSensor nightReadingsSound[numSamples];

// LED Light
const int ledPinRed = 3;
const int ledPinGreen = 5;
const int ledPinBlue = 6;
int colourCandle[3] = { 255, 197, 143 };

// Sound
int soundOn = 0;

// LCD Display
rgb_lcd lcd;

// Internal Logic
int mildInstability = 0, highInstability = 0, severeInstability = 0, errorInstability = 0;
int32_t thresholdHeartRate[3] = { 60, 100, 80 };
float thresholdAccel = (0.2 * 9.81);
int32_t thresholdSound = 600;

float SSI = 0;
float ssiPulse = 0;
float ssiAccel = 0;
float ssiSound = 0;


/* --------------- SETUP: INITIAL BEHAVIOUR  --------------- */
void setup() {
  Serial.begin(9600);  // Initialize Serial Monitor at 9600 baud

  // Pulse Oximeter
  Wire.begin();                     // Enable I2C (SDA: A4, SCL: A5)
  while (!pulseOximeter.begin()) {  //
    Serial.println("MAX30102 not found!");
    delay(1000);  // 2 sec delay before printing the message again
  }
  // Configuration: (Led Brightness (0-255), Sample Average, LED Mode (both IR and Red),
  // Sample Rate (Hz), Pulse Width (LED pulse width), ADC Range)
  pulseOximeter.sensorConfiguration(250, SAMPLEAVG_4, MODE_MULTILED,
                                    SAMPLERATE_50, PULSEWIDTH_411,
                                    ADCRANGE_16384);

  
  // Accelerometer
  xRest = readAveragedVoltage(xPin, 200);
  yRest = readAveragedVoltage(yPin, 200);
  zRest = readAveragedVoltage(zPin, 200);  // includes gravity

  // LED Warm Light
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);

  // LCD Display
  lcd.begin(20, 4);
  lcd.setRGB(255, 255, 255);
}

/* --------------- LOOP: ITERATIVE BEHAVIOUR  --------------- */
void loop() {
  /* --------------- SENSOR READINGS --------------- */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("READING..");
  delay(2000);

  // 1) Pulse Oximeter
  pulseOximeter.heartrateAndOxygenSaturation(&SPO2, &SPO2Valid, &heartRate, &heartValid);

  if (SPO2Valid && (SPO2 != -999) && heartValid && (heartRate != -999)) {
    nightReadingsPulseOximeter[pulseIndex] = { millis(), SPO2, heartRate };
    pulseIndex = (pulseIndex == numSamples - 1) ? 0 : pulseIndex + 1;
    Serial.println("HEART RATE: ");
    Serial.println(heartRate);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR: CORRECT");
    lcd.setCursor(0, 1);
    lcd.print("FINGER PLACEMENT");
    delay(2000);
  }

  // 2) Accelerometer
  double xVoltage = readAveragedVoltage(xPin, SAMPLES);
  double yVoltage = readAveragedVoltage(yPin, SAMPLES);
  double zVoltage = readAveragedVoltage(zPin, SAMPLES);

  float g_x = (xVoltage - xRest) / sensitivity;
  float g_y = (yVoltage - yRest) / sensitivity;
  float g_z = (zVoltage - zRest) / sensitivity;

  float accelX = g_x * 9.81;
  float accelY = g_y * 9.81;
  float accelZ = g_z * 9.81;

  float magnitude = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));  //averaged acceleration
  float movement = abs(magnitude - 9.81);

  if (movement < thresholdAccel) {  //disclude acceleration from gravity
    nightReadingsAccelerometer[accelIndex] = { millis(), movement };
    accelIndex = (accelIndex == numSamples - 1) ? 0 : accelIndex + 1;
    // Serial.println("ACCEL: ");
    // Serial.println(movement);
  }


  // 3) Sound Sensor | only read if sound is not playing!
  if (!soundOn) {
    int soundOut = readAveragedADC(A0, SAMPLES);
    nightReadingsSound[soundIndex] = { millis(), soundOut };
    soundIndex = (soundIndex == numSamples - 1) ? 0 : soundIndex + 1;
    // Serial.println("SOUND: ");
    // Serial.println(soundOut);
  }


  /* --------------- INTERNAL LOGIC : THRESHOLD AND SRI CALCULATION --------------- */
  // SLEEP STABILITY INDEX
  if (pulseIndex == 0 || nightReadingsPulseOximeter[pulseIndex - 1].heartRate == 0) ssiPulse = 0;
  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < thresholdHeartRate[2]) {
    ssiPulse = (abs(nightReadingsPulseOximeter[pulseIndex - 1].heartRate - thresholdHeartRate[0]) / thresholdHeartRate[0]) * 100;
  } else ssiPulse = (abs(nightReadingsPulseOximeter[pulseIndex - 1].heartRate - thresholdHeartRate[1]) / thresholdHeartRate[1]) * 100;
  ssiPulse = ((ssiPulse/100.0) * 35);
  if (accelIndex == 0 || nightReadingsAccelerometer[accelIndex - 1].magnitude == 0) ssiAccel = 0;
  else ssiAccel = (abs(nightReadingsAccelerometer[accelIndex - 1].magnitude - thresholdAccel) / thresholdAccel) * 100;
  ssiAccel = ((ssiAccel/100.0)*30);

  if (soundIndex == 0 || soundOn == 1 || nightReadingsSound[soundIndex - 1].sound == 0) ssiSound = 0;
  else ssiSound = (abs(nightReadingsSound[soundIndex - 1].sound - thresholdSound) / thresholdSound) * 100;
  ssiSound = ((ssiSound/100.0)*35);

  SSI = ssiPulse + ssiAccel + ssiSound;


  // Threshold Comparisons
  if ((pulseIndex == 0 || nightReadingsPulseOximeter[pulseIndex - 1].heartRate == 0) || 
  (accelIndex == 0 || nightReadingsAccelerometer[accelIndex - 1].magnitude == 0) || 
  (soundIndex == 0 || nightReadingsSound[soundIndex - 1].sound == 0)) {
    errorInstability = 1;
    severeInstability = 0;
    highInstability = 0;
    mildInstability = 0;

    SSI = 0;
  }

  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < 0.2 * thresholdHeartRate[0] || 
  nightReadingsPulseOximeter[pulseIndex - 1].heartRate > 0.2 * thresholdHeartRate[1] || 
  nightReadingsAccelerometer[accelIndex - 1].magnitude > 0.2 * thresholdAccel || 
  nightReadingsSound[soundIndex - 1].sound > 0.2 * thresholdSound) {
    // SEVERE INSTABILITY!
    errorInstability = 0;
    severeInstability = 1;
  }

  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < 0.15 * thresholdHeartRate[0] || nightReadingsPulseOximeter[pulseIndex - 1].heartRate > 0.15 * thresholdHeartRate[1] || nightReadingsAccelerometer[accelIndex - 1].magnitude > 0.15 * thresholdAccel || nightReadingsSound[soundIndex - 1].sound > 0.15 * thresholdSound) {
    // HIGH INSTABILITY!
    errorInstability = 0;
    highInstability = 1;
    severeInstability = 0;
  }

  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < 0.1 * thresholdHeartRate[0] || nightReadingsPulseOximeter[pulseIndex - 1].heartRate > 0.1 * thresholdHeartRate[1] || nightReadingsAccelerometer[accelIndex - 1].magnitude > 0.1 * thresholdAccel || nightReadingsSound[soundIndex - 1].sound > 0.1 * thresholdSound) {
    // MILD INSTABILITY!
    errorInstability = 0;
    mildInstability = 1;
    highInstability = 0;
    severeInstability = 0;
  }

  else {
    severeInstability = 0;
    highInstability = 0;
    mildInstability = 0;
    errorInstability = 0;

    // reset light
    analogWrite(ledPinBlue, 0);
    analogWrite(ledPinBlue, 0);
    analogWrite(ledPinBlue, 0);
  }

  // /* --------------- ACTUATORS --------------- */
  if (severeInstability | highInstability) {
    //activate sound
    Serial.println("PATEINT UNSTABLE");
    Serial.println("PLAY MUSIC");
    soundOn = 1;
  }

  if (Serial.available() > 0) {
    int receivedValue = Serial.parseInt();
    if (receivedValue == 1) {
      soundOn = 0;
    }
  }

  if (severeInstability | highInstability | mildInstability) {
    //activate light (leds)
    Serial.println("PATIENT UNSTABLE");
    Serial.println("WARM LIGHT");

    //try warm light: (R,G,B) = 255, 147, 41
    analogWrite(ledPinRed, colourCandle[0]);
    analogWrite(ledPinGreen, colourCandle[1]);
    analogWrite(ledPinBlue, colourCandle[2]);
  }

  // /* --------------- PRINT TO LCD --------------- */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Heart Rate: ");
  lcd.setCursor(0, 1);
  lcd.print(nightReadingsPulseOximeter[pulseIndex - 1].heartRate);
  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Acceleration: ");
  lcd.setCursor(0, 1);
  lcd.print(nightReadingsAccelerometer[accelIndex - 1].magnitude);
  delay(2000);
  lcd.clear();

  if (soundOn == 0){
    lcd.setCursor(0, 0);
    lcd.print("Sound: ");
    lcd.setCursor(0, 1);
    lcd.print(nightReadingsSound[soundIndex - 1].sound);
    delay(2000);
    lcd.clear();
  }

  lcd.setCursor(0, 0);
  lcd.print("SLEEP STABILITY");
  lcd.setCursor(0, 1);
  lcd.print("INDEX: ");
  lcd.print(SSI);
  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);
  if (mildInstability) {
    lcd.print("MILD");
    lcd.setCursor(0, 1);
    lcd.print("INSTABILITY");
  } else if (highInstability) {
    lcd.print("HIGH");
    lcd.setCursor(0, 1);
    lcd.print("INSTABILITY");
  } else if (severeInstability) {
    lcd.print("SEVERE");
    lcd.setCursor(0, 1);
    lcd.print("INSTABILITY");
  } else if (errorInstability) {
    lcd.print("ERROR");
  } else {
    lcd.print("PATIENT STABLE");
  }
}
