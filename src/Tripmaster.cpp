#include <Arduino.h>
#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

#include <HT1621.h>
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
HT1621 compassLCD;
HT1621 tripLCD;

//constants
const unsigned int WHEEL_SENSOR = 2;
const unsigned int TRIP_UP = 4;
const unsigned int TRIP_DOWN = 5;
const unsigned int TRIP_RESET = 6;
const unsigned int LED = 13;
const unsigned int RESET_TIMEOUT = 4000;
const unsigned int SECOND = 1000;
const unsigned int TWO_SECONDS = 2000;
const unsigned int THREE_SECONDS = 3000;
const double WHEEL_TRIP = 1.67572552; // wheel circuit in meters ((21 in * pi) * 0.0254 m)
const float DECLINATION_ANGLE_DEG=6; // for Łódź
const float DECLINATION_ANGLE_MIN=8; // for Łódź
const float TRIP_STEP = 0.01;

double trip = 0.0;
unsigned long lastWheelPulseInMillis = 0;
unsigned long lastResetBtnPressedInMillis = 0;
unsigned int batteryLevel = 0;

//methods
void updateCompass();
void handleButtons();
void wheelPulse();
float radToDeg(float);
float applyDeclinationAngle(float);
int modulo(int, int);

//app
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(TRIP_UP, INPUT_PULLUP);
  pinMode(TRIP_DOWN, INPUT_PULLUP);
  pinMode(TRIP_RESET, INPUT_PULLUP);
  pinMode(WHEEL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_SENSOR), wheelPulse, FALLING);
  
  compassLCD.begin(12, 11, 10); // (cs, wr, Data, backlight)
  compassLCD.clear();
  tripLCD.begin(9, 8, 7); // (cs, wr, Data, backlight)
  tripLCD.clear();
  
  Serial.begin(9600);
  Wire.begin();

  
  while (!compass.begin())
  {
    compassLCD.print((char*) "--00001-");
    delay(500);
  }
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(-32, 59);

  compassLCD.print((char*) "HELLO");
  tripLCD.print((char*) "HELLO");
  digitalWrite(LED, LOW);
  delay(1500);
}

void loop() {  
  updateCompass();
  handleButtons();
  tripLCD.print(trip, 2);
}

void updateCompass() {
  Vector norm = compass.readNormalize();
  float heading = radToDeg(atan2(norm.YAxis, norm.XAxis));
  heading = applyDeclinationAngle(heading);
  compassLCD.print(heading, (char*) "%5u*", 0);
}

void handleButtons() {
  unsigned int tripDownBtnState = digitalRead(TRIP_DOWN);
  unsigned int tripUpBtnState = digitalRead(TRIP_UP);
  unsigned int tripResetBtnState = digitalRead(TRIP_RESET);
  
  if (tripUpBtnState == LOW) {
    trip += TRIP_STEP; // increase 10m
  }
  if (tripDownBtnState == LOW && trip >= TRIP_STEP) {
    trip -= TRIP_STEP; // decrease 10m
  }
  
  if (tripResetBtnState == HIGH) { //reset btn not pressed
    lastResetBtnPressedInMillis = millis(); //reset clock when not pressed
    batteryLevel = 0;
  }
  unsigned long pressingTime = millis() - lastResetBtnPressedInMillis;
  if (trip > 0.0 && pressingTime >= RESET_TIMEOUT) {
    trip = 0.0;
    batteryLevel = 0;
  }

  if (pressingTime > SECOND)  {
    batteryLevel = 1;
  }
  if (pressingTime > TWO_SECONDS)  {
    batteryLevel = 2;
  }
  if (pressingTime > THREE_SECONDS)  {
    batteryLevel = 3;
  }
  if (pressingTime > RESET_TIMEOUT)  {
    batteryLevel = 0;
  }
  tripLCD.setBatteryLevel(batteryLevel);
}

void wheelPulse() {
  unsigned long wheelPulseInMillis = millis();
  if (wheelPulseInMillis - lastWheelPulseInMillis > 20) {
    trip += 0.01; 
    tripLCD.print(trip, 2);
  }
  lastWheelPulseInMillis = wheelPulseInMillis;
}

float radToDeg(float rad) {
  return rad * 180/M_PI;
}

float applyDeclinationAngle(float degHeading) {
  // Formula: modulo((deg + (min / 60.0)) / (180 / M_PI), 360);
  return  modulo(round(degHeading + (DECLINATION_ANGLE_DEG + (DECLINATION_ANGLE_MIN / 60.0))), 360);
}

int modulo(int x, int N){
    return (x % N + N) %N;
}