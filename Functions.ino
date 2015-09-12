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
float angleChange(float lCount, float rCount float mmpc, float hl) {
  //Calculate direction change from encoder
  return pAngle(degrees((rCount - lCount) * mmpc / 2 / hl));
}
float xChange(float lCount, float rCount, float mmpc, float hl) {
  //Derive x-co from polar
  float d = disp(lCount, rCount, mmpc);
  float a = angleChange(lCount, rCount, mmpc, hl);
  return d * cos(radians(th+a));
}
float yChange(float lCount, float rCount, float mmpc, float hl) {
  //Derive y-co from polar
  float d = disp(lCount, rCount, mmpc);
  float a = angleChange(lCount, rCount, mmpc, hl);
  return  d * sin(radians(th+a));
}
