#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Pins
//  Motors
const byte M1 = 4;
const byte E1 = 5;
const byte M2 = 7;
const byte E2 = 6;
//  Interrupts
const byte LItr = 3;
const byte RItr = 2;

//Positions & Motions
float x = 0, y = 0, th = 90;
float Tx = 0, Ty = 0, Tth = 0;
float v = 0, w = 0;
float Kp = 1.35, Ka = 10, Kb = 0;
float Wl = 255, Wr = 255;
float leftCount = 0, rightCount = 0;
float lastLeftCount = 10, lastRightCount = 10;

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

//path
const int pathPts = 4;
float path[pathPts][2] = {{250,0}, {250,250}, {0,250}, {0,0}};

void setup() {
  timer = millis();
  Serial.begin(115200);
  //initialize
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(LItr, HIGH);
  digitalWrite(RItr, HIGH);
  attachInterrupt(1, leftEncoder, CHANGE);
  attachInterrupt(0, rightEncoder, CHANGE);
  lcd.init();
  lcd.backlight();
  //path setting
  //  for (int i = 0; i <= 10; i++) {
  //    path[i][0] = 150 * sin(PI / 5 * i);
  //    path[i][1] = 150 - 150 * cos(PI / 5 * i);
  //  }
  //  addSquare(200, true);
  Tx = path[0][0];
  Ty = path[0][1];
}

void loop() {
  if (Serial.available() > 0) {
    char inchar = Serial.read();
    switch (inchar) {
      //"GXX,YY" => go to (XX,YY)
      case 'G':
      case 'g':
        if (stopping) {
          stopping = false;
          break;
        }
        Tx = Serial.parseFloat();
        Ty = Serial.parseFloat();
        break;
      //stop
      case 'S':
      case 's':
        stopping = true;
        break;
    }
  }
  if (millis() - timer > resolution * 1000 && !stopping) {
    //th gen from compass
    x += xChange(leftCount, rightCount, mm_per_count, l);
    y += yChange(leftCount, rightCount, mm_per_count, l);
    th += angleChange(leftCount, rightCount, mm_per_count, l);
    th = pAngle(th);

    //update location
    float p = dist(x, y, Tx, Ty);
    float a = pAngle(inclination(x, y, Tx, Ty) - th);
    float b = pAngle(-th - a + Tth);

    v = constrain(Kp * p, Vmin, Vmax);
    v = map(v, 0, Vmax, 0, 255 * r);
    w = constrain(Ka * a + Kb * b, -Wmax, Wmax);
    w = map(w, -Wmax, Wmax, -255 * r / l, 255 * r / l);

    //polar->2wheel
    Wl = constrain((v - w * l) / r, -255, 255);
    Wr = constrain((v + w * l) / r, -255, 255);

    Serial.print(x);
    Serial.print("  ");
    Serial.print(y);
    Serial.print("  ");
    Serial.println(th);

    if (p < Vmin * resolution) {  //arrive point
      Wl = 0;
      Wr = 0;
      current++;
      if (current <= pathPts) {   //find next point
        Tx = path[current][0];
        Ty = path[current][1];
        Tth = inclination(x, y, Tx, Ty);
      }
    }
    //actuate
    digitalWrite(M1, Wl > 0 ? LOW : HIGH);
    digitalWrite(M2, Wr > 0 ? LOW : HIGH);
    analogWrite(E1, abs(Wl));
    analogWrite(E2, abs(Wr));
    //lcd display
    if (leftCount != 0 && rightCount != 0) {
      if (leftCount != lastLeftCount || rightCount != lastRightCount) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("L:" + String(round(Wl)));
        lastLeftCount = leftCount;
        lcd.setCursor(7, 0);
        lcd.print("R:" + String(round(Wr)));
        lastRightCount = rightCount;
        lcd.setCursor(0, 1);
        lcd.print("v:" + String(round((leftCount + rightCount) * mm_per_count / 2 / resolution)) + "mm/s");
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
    lcd.print(current);
    timer = millis();
    leftCount = 0;
    rightCount = 0;
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