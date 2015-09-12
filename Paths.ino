int pathPtNo = 0;
void addLine(int fx, int fy){
  pathPtNo+=1;
  path[pathPtNo][0] = fx;
  path[pathPtNo][1] = fy;
}

void addArc(int fx, int fy, int r, boolean major, boolean clockwise){
}

void addSquare(int sLength, boolean cw){//only for test
  for(int i=0; i<4; i++){
    float nX = x, nY = y, nth = th;
    nX += sLength*cos(radians(nth));
    nY += sLength*sin(radians(nth));
    addLine(nX, nY);
    nth += (cw?-90:90);
    nth = pAngle(nth);
  }
}
