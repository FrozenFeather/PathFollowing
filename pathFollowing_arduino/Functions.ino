//Maths
float dist(float x1, float y1, float x2, float y2) {
  // return distance between two points
  return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}
float inclination(float x1, float y1, float x2, float y2) {
  // return direction of point
  return pAngle(degrees(atan2(y2 - y1, x2 - x1)));
}
float pAngle(float a) {
  // return principal degree-angle (in (-180, 180])
  if (a > 180) a -= 360;
  if (a <= -180) a += 360;
  return a;
}

//Kinematics
float disp(float lCount, float rCount, float mmpc) {
  //Calculate distance from encoder
  return (lCount + rCount) * mmpc / 2;
}
float angleChange(float lCount, float rCount, float mmpc, float hl, float iY) {
  //Calculate direction change from encodery

  my3IMU.getYawPitchRoll(ypr);
  float cAC = pAngle(iY - ypr[0] - th);
  float eAC = pAngle(degrees((rCount - lCount) * mmpc / 2 / hl));
  //return pAngle(degrees((rCount - lCount) * mmpc / 2 / hl));
  return eAC;
}
float xChange(float lCount, float rCount, float mmpc, float hl, float iY) {
  //Derive x-co from polar
  float d = disp(lCount, rCount, mmpc);
  float a = angleChange(lCount, rCount, mmpc, hl, iY);
  if ((rCount - lCount)*a < 0) {  //Identify if motion is cw or acw(as pAngle is taken)
    a = 2 * PI + a;
  }
  return d * cos(radians(th + pAngle(a / 2)));
}
float yChange(float lCount, float rCount, float mmpc, float hl, float iY) {
  //Derive y-co from polar
  float d = disp(lCount, rCount, mmpc);
  float a = angleChange(lCount, rCount, mmpc, hl, iY);
  if ((rCount - lCount)*a < 0) {
    a = 2 * PI + a;
  }
  return  d * sin(radians(th + pAngle(a / 2)));
}

//Romeo Buttons
int get_key(unsigned int input) {
  // Convert ADC value to key number
  int k;
  for (k = 0; k < 5; k++) {
    if (input < SBts[k]) {
      return k;
    }
  }
  if (k >= 5)
    k = -1;     // No valid key pressed
  return k;
}

