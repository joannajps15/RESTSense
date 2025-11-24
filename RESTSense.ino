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
  int32_t heartRate;
};
int pulseIndex = 0;
dataPulseOximeter nightReadingsPulseOximeter[numSamples];

// Accelerometer
const int xPin = A3;
const int yPin = A2;
const int zPin = A1;
const int SAMPLES = 25;

const float sensitivity = 0.3;    // Sensitivity in V/gravity (g) (1 g = 9.81 ms2)

const float RestVoltage = 1.641;
float xRest = 0, yRest = 0, zRest = 0;  // includes gravity
double xVoltage = 0, yVoltage = 0, zVoltage = 0;
float g_x = 0, g_y = 0, g_z = 0;
float magnitude = 0, movement = 0;

// averages samples from ADXL334
float readAveragedVoltage(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(40);
  }
  //10-bit value converted to voltage between 0 and 5 volts
  return ((sum / (float)samples) * (3.3 / 1023.0));
}

struct dataAccelerometer {
  unsigned long timestamp;
  float magnitude;
};
int accelIndex = 0;
dataAccelerometer nightReadingsAccelerometer[numSamples];

// Sound Sensor
int soundOut = 0;

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
int ledTimer = 0;

// Sound
int soundOn = 0;

// LCD Display
rgb_lcd lcd;

// Push Button
const int pushButton = 2;

// Internal Logic
int mildInstability = 0, highInstability = 0, severeInstability = 0;
int32_t thresholdHeartRate[3] = { 60, 100, 80 };
float thresholdAccel = (0.9);
int32_t thresholdSound = 90;

float SSI = 0;
float ssiPulse = 0;
float ssiAccel = 0;
float ssiSound = 0;

// End Print

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
  xRest = readAveragedVoltage(xPin, SAMPLES);
  yRest = readAveragedVoltage(yPin, SAMPLES);
  zRest = readAveragedVoltage(zPin, SAMPLES); 

  // LED Warm Light
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);

  // LCD Display
  lcd.begin(20, 4);
  lcd.setRGB(255, 255, 255);

  // Push Button
  pinMode(pushButton, INPUT);
}

/* --------------- LOOP: ITERATIVE BEHAVIOUR  --------------- */
void loop() {
  /* --------------- SENSOR READINGS --------------- */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PRESS BUTTON");
  lcd.setCursor(0, 1);
  lcd.print("TO QUIT");
  delay(2000);

  if(digitalRead(pushButton)){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PROGRAM COMPLETE");
  
    Serial.println("END");
    if (pulseIndex != 0){
      Serial.println("PULSE");
      for (int i = 0; i < pulseIndex; i++){
        Serial.println(nightReadingsPulseOximeter[i].timestamp);
        Serial.println(nightReadingsPulseOximeter[i].heartRate);
      }
    }

    if (accelIndex != 0){
      Serial.println("ACCEL");
      for (int i = 0; i < accelIndex; i++){
        Serial.println(nightReadingsAccelerometer[i].timestamp);
        Serial.println(nightReadingsAccelerometer[i].magnitude);
      }
    }

    if (soundIndex != 0){
      Serial.println("SOUND");
      for (int i = 0; i < soundIndex; i++){
        Serial.println(nightReadingsSound[i].timestamp);        
        Serial.println(nightReadingsSound[i].sound);
      }
    }

    Serial.println("COMPLETE");

    while(1);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("READING..");
  delay(2000);

  // 1) Pulse Oximeter
  pulseOximeter.heartrateAndOxygenSaturation(&SPO2, &SPO2Valid, &heartRate, &heartValid);

  if (SPO2Valid && (SPO2 != -999) && heartValid && (heartRate > 0)) {
    nightReadingsPulseOximeter[pulseIndex] = { millis(), heartRate };
    pulseIndex = (pulseIndex == numSamples - 1) ? 0 : pulseIndex + 1;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR: CORRECT");
    lcd.setCursor(0, 1);
    lcd.print("FINGER PLACEMENT");
    delay(2000);
  }
  Serial.println("HEART RATE: ");
  Serial.println(heartRate);

  // 2) Accelerometer
  xVoltage = readAveragedVoltage(xPin, SAMPLES);
  yVoltage = readAveragedVoltage(yPin, SAMPLES);
  zVoltage = readAveragedVoltage(zPin, SAMPLES);

  g_x = (xVoltage - xRest) / sensitivity;
  g_y = (yVoltage - yRest) / sensitivity;
  g_z = (zVoltage - zRest) / sensitivity;

  // g_x = ((xVoltage - RestVoltage) / sensitivity) * 9.81;
  // g_y = ((yVoltage - RestVoltage) / sensitivity) * 9.81;
  // g_z = ((zVoltage - RestVoltage) / sensitivity) * 9.81;

  magnitude = (sqrt((g_x * g_x) + (g_y * g_y) + (g_z * g_z)));  //averaged acceleration

  if (magnitude > thresholdAccel) {  //disclude acceleration from gravity, we record!
    nightReadingsAccelerometer[accelIndex] = { millis(), magnitude };
    accelIndex = (accelIndex == numSamples - 1) ? 0 : accelIndex + 1;
  }
  Serial.println("ACCEL: ");
  Serial.println(magnitude);

  // 3) Sound Sensor | only read if sound is not playing!
  if (!soundOn) {
    soundOut = readAveragedADC(A0, SAMPLES);
    nightReadingsSound[soundIndex] = { millis(), soundOut };
    soundIndex = (soundIndex == numSamples - 1) ? 0 : soundIndex + 1;
  }
  Serial.println("SOUND: ");
  Serial.println(soundOut);

  /* --------------- INTERNAL LOGIC : THRESHOLD AND SRI CALCULATION --------------- */
  // SLEEP STABILITY INDEX

  if (pulseIndex == 0) { 
    ssiPulse = 0;
  } 
  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < thresholdHeartRate[2]) {
    ssiPulse = (abs(nightReadingsPulseOximeter[pulseIndex - 1].heartRate - thresholdHeartRate[0]) / thresholdHeartRate[0]) * 100;
  } 
  else ssiPulse = (abs(nightReadingsPulseOximeter[pulseIndex - 1].heartRate - thresholdHeartRate[1]) / thresholdHeartRate[1]) * 100;
  ssiPulse = 35 - int((ssiPulse/100.0) * 35);
  
  if (accelIndex == 0) ssiAccel = 0;
  else ssiAccel = (abs(nightReadingsAccelerometer[accelIndex - 1].magnitude - thresholdAccel) / thresholdAccel) * 100;
  ssiAccel = 30 - int((ssiAccel/100.0)*30);

  if (soundIndex == 0 || soundOn == 1 || nightReadingsSound[soundIndex - 1].sound == 0) ssiSound = 0;
  else ssiSound = ((abs(nightReadingsSound[soundIndex - 1].sound - thresholdSound)) / thresholdSound) * 100;
  ssiSound = 35 - int((ssiSound/100.0)*35);
  Serial.println("ssiSound");
  Serial.println(ssiSound);

  if ((pulseIndex == 0) && (accelIndex == 0) && (soundIndex == 0)) SSI = 0;
  else SSI = ssiPulse + ssiAccel + ssiSound;
  Serial.println("SSI");
  Serial.println(SSI);

  // Threshold Comparisons
  if ((pulseIndex == 0) && (accelIndex == 0) && (soundIndex == 0)) {
    // errorInstability = 1;
    severeInstability = 0;
    highInstability = 0;
    mildInstability = 0;
  }

  // SEVERE INSTABILITY --> GREATER THAN 20%
  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < 0.80 * thresholdHeartRate[0] || 
  nightReadingsPulseOximeter[pulseIndex - 1].heartRate > 1.20 * thresholdHeartRate[1] || 
  nightReadingsAccelerometer[accelIndex - 1].magnitude > 1.20 * thresholdAccel || 
  nightReadingsSound[soundIndex - 1].sound > 1.20 * thresholdSound) {
    // SEVERE INSTABILITY!
    // errorInstability = 0;
    severeInstability = 1;    
    highInstability = 0;
    mildInstability = 0;
  }

  // HIGH INSTABILITY --> 15-20%
  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < 0.9 * thresholdHeartRate[0] || 
  nightReadingsPulseOximeter[pulseIndex - 1].heartRate > 1.1 * thresholdHeartRate[1] || 
  nightReadingsAccelerometer[accelIndex - 1].magnitude > 1.1 * thresholdAccel || 
  nightReadingsSound[soundIndex - 1].sound > 1.1 * thresholdSound) {
    // HIGH INSTABILITY!
    // errorInstability = 0;
    highInstability = 1;
    severeInstability = 0;
    mildInstability = 0;
  }

  // LOW INSTABILITY --> 10-15%
  else if (nightReadingsPulseOximeter[pulseIndex - 1].heartRate < thresholdHeartRate[0] || 
  nightReadingsPulseOximeter[pulseIndex - 1].heartRate >  thresholdHeartRate[1] || 
  nightReadingsAccelerometer[accelIndex - 1].magnitude > thresholdAccel || 
  nightReadingsSound[soundIndex - 1].sound > thresholdSound) {
    // MILD INSTABILITY!
    // errorInstability = 0;
    mildInstability = 1;
    highInstability = 0;
    severeInstability = 0;
  }

  else {
    severeInstability = 0;
    highInstability = 0;
    mildInstability = 0;
    // errorInstability = 0;

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
  // } else if (errorInstability) {
  //   lcd.print("ERROR");
  } else {
    lcd.print("PATIENT STABLE");
  }
  delay(2000);
}
