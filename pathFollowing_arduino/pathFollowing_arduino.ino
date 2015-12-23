#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <SPI.h>

//Pins
//  Motors
const byte M1 = 4;
const byte E1 = 5;
const byte M2 = 7;
const byte E2 = 6;
//  Interrupts
const byte LItr = 3;
const byte RItr = 2;
//  Romeo A7 Buttons
const int SBts[5] = {30, 150, 360, 535, 760};

//Positions & Motions
float x = 0, y = 0, th = 0;
float Tx = 0, Ty = 0, Tth = 0;
float Kp = 0.7, Ka = 15, Kb = -0.2;
float Wl = 255, Wr = 255;
float leftCount = 0, rightCount = 0;
float lastLeftCount = 10, lastRightCount = 10;

//IMU
float ypr[3]; // yaw pitch roll
FreeIMU my3IMU = FreeIMU();
float initialYaw = 0;

//Configuration
const float l = 175 / 2, r = 25; //l: half of wheel distance, r: radius of wheel :in mm
const float mm_per_count = 1.33; //encoder
const float Vmax = 300;      //mm/s
const float Vmin = 160;       //145
const float Wmax = 100;      //deg/s

//Accuracy
const float resolution = 0.1;  //s
const float angTolerance = 5;     //deg

//Display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//timer
unsigned long timer;
int current = 0;
boolean stopping = false;
int stopTime = 0;

//path
const int pathPts = 4;
int path[pathPts][2] = {{300, 0}, {300, 300}, {0, 300}, {0, 0}};//square Path

void setup() {
  Wire.begin();
  Serial.begin(115200);
  //initialize
  //  Motor
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  //  Encoder
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(LItr, HIGH);
  digitalWrite(RItr, HIGH);
  attachInterrupt(1, leftEncoder, CHANGE);
  attachInterrupt(0, rightEncoder, CHANGE);
  //  LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("calib e-compass");

  my3IMU.init(); // the parameter enable or disable fast mode
  my3IMU.getYawPitchRoll(ypr);
  initialYaw = ypr[0];
  delay(2000);
  timer = millis();

  Tx = path[0][0];
  Ty = path[0][1];
  if (pathPts > 1) {
    Tth = inclination(Tx, Ty, path[1][0], path[1][1]);
  }
}

void loop() {
  //  if (Serial.available() > 0) {
  //    char inchar = Serial.read();
  //    switch (inchar) {
  //      //"GXX,YY" => go to (XX,YY)
  //      case 'G':
  //      case 'g':
  //        if (stopping) {
  //          stopping = false;
  //          break;
  //        }
  //        Tx = Serial.parseFloat();
  //        Ty = Serial.parseFloat();
  //        break;
  //      //stop
  //      case 'S':
  //      case 's':
  //        stopping = true;
  //        break;
  //      case 'T':
  //      case 't':
  //        stopping = false;
  //        Tx = x;
  //        Ty = y;
  //        Tth = th + 90;
  //        break;
  //    }
  //  }
//  int key = get_key(analogRead(A7));
//  if (key != -1) {
//    if (key == SBts[0]) {
//    } else if (key == SBts[1]) {
//    } else if (key == SBts[4]) {
//      stopping = !stopping;
//    }
//  }
  if (millis() - timer > resolution * 1000 && millis() > 15000) {
    //th gen from compass
    x += xChange(leftCount, rightCount, mm_per_count, l, initialYaw);
    y += yChange(leftCount, rightCount, mm_per_count, l, initialYaw);
    th += angleChange(leftCount, rightCount, mm_per_count, l, initialYaw);
    th = pAngle(th);
//        my3IMU.getYawPitchRoll(ypr);
//        th = pAngle(initialYaw - ypr[0]);

    //update location
    float p = dist(x, y, Tx, Ty);
    float a = pAngle(inclination(x, y, Tx, Ty) - th);
    float b = pAngle(-th - a + Tth);

    float v = constrain(Kp * p, Vmin, Vmax);
    v = map(v, 0, Vmax, 0, 255 * r);
    float w = constrain(Ka * a + Kb * b, -Wmax, Wmax);
    w = map(w, -Wmax, Wmax, -255 * r / l, 255 * r / l);

    //polar->2wheel
    Wl = constrain((v - w * l) / r, -255, 255);
    Wr = constrain((v + w * l) / r, -255, 255);

    //    Serial.print(x);
    //    Serial.print("  ");
    //    Serial.println(y);
    Serial.print(initialYaw);
    Serial.print(",");
    Serial.println(th);

    //Shift to next target point
    if (p < Vmin * resolution) {  //arrive point
      Wl = 0;
      Wr = 0;
      stopTime = 500;
      if (current + 1 < pathPts) { //find next point
        current++;
        Tx = path[current][0];
        Ty = path[current][1];
        if (current + 1 < pathPts) {
          Tth = inclination(Tx, Ty, path[current + 1][0], path[current + 1][1]);
        }
      } else
        stopping = true;
    }
    //actuate
    if (!stopping && stopTime <= 0) {
      digitalWrite(M1, Wl > 0 ? LOW : HIGH);
      digitalWrite(M2, Wr > 0 ? LOW : HIGH);
      analogWrite(E1, abs(Wl));
      analogWrite(E2, abs(Wr));
    } else {
      analogWrite(E1, 0);
      analogWrite(E2, 0);
      stopTime = max(stopTime - 100, 0);
    }
    //lcd display
    if (leftCount != 0 || rightCount != 0) {
      if (leftCount != lastLeftCount || rightCount != lastRightCount) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("L:" + String(round(Wl)));
        lastLeftCount = leftCount;
        lcd.setCursor(7, 0);
        lcd.print("R:" + String(round(Wr)));
        lastRightCount = rightCount;
        lcd.setCursor(0, 1);
        //lcd.print("v:" + String(round(disp(leftCount, rightCount, mm_per_count) / resolution)) + "mm/s");
        lcd.print("theta:" + String(th));
      }
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("x:" + String(round(x)));
      lastLeftCount = leftCount;
      lcd.setCursor(7, 0);
      lcd.print("y:" + String(round(y)));
      lastRightCount = rightCount;
      lcd.setCursor(0, 1);
      lcd.print("theta:" + String(th));
    }
    lcd.setCursor(14, 1);
    lcd.print(current + 1);
    timer = millis();
    leftCount = 0;
    rightCount = 0;
  }
  else if (millis() <= 15000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("calib e-compass");
    my3IMU.getYawPitchRoll(ypr);
    initialYaw = ypr[0];
    lcd.setCursor(0, 1);
    lcd.print("theta: " + String(ypr[0]));
  }
}

//encoders
void leftEncoder() {
  if (Wl >= 0)
    leftCount++;
  else
    leftCount--;
}
void rightEncoder() {
  if (Wr >= 0)
    rightCount++;
  else
    rightCount--;
}
