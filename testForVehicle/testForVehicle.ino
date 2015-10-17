#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long timer;
byte value = 0;
unsigned long leftCount = 0, rightCount = 0;
unsigned long totalLeftCount = 0, totalRightCount = 0;
const float cc = 2 * PI * 25; //in mm;
const float mmPerCount = 1

void setup() {
  timer = millis();
  Serial.begin(115200);
  for (int i = 4; i <= 7 ; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  attachInterrupt(1, leftEncoder, CHANGE);
  attachInterrupt(0, rightEncoder, CHANGE);
  lcd.init();
  lcd.backlight();
}

void loop() {
  while (Serial.available() > 0) {
    //    Serial.println(Serial.read());
    value = (byte)Serial.parseInt();
  }
  Serial.println(value);
  digitalWrite(4, LOW);
  digitalWrite(7, LOW);

  for (int i = 0; i < 2; i++) {
    analogWrite(5 + i, value);
  }
  delay(1000);
  value = 0;
  for (int i = 0; i < 2; i++) {
    analogWrite(5 + i, value);
  }
  float v = (leftCount + rightCount) / 2 * mmPerCount;
  if (value == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("L:" + String(round(leftCount)));
    lcd.setCursor(7, 0);
    lcd.print("R:" + String(round(rightCount)));
    leftCount = 0;
    rightCount = 0;
    lcd.setCursor(0, 1);
    lcd.print("v:" + String(round(v)) + "mm/s");
    Serial.println((totalLeftCount + totalRightCount)*mmPerCount/2);
  }
}
void leftEncoder() {
  leftCount++;
  totalLeftCount++;
}
void rightEncoder() {
  rightCount++;
  totalRightCount++;
}

