import processing.serial.*;

float x=0, y=0, th=0;
float Tx=0, Ty=0, Tth=0;
float p=0, a=0, b=0;
float v=0, w=0;
float Kp=2.7, Ka = 5, Kb = 0;
float Lx=x, Ly=y;
float lastMouseX=-1, lastMouseY=-1;

ArrayList<float[]> path = new ArrayList<float[]>();

boolean drawing = true;
boolean followingPath = false;

float maxW = 0;

Serial myPort;

void setup() {
  frameRate(5);
  //  size(displayWidth, displayHeight);
  size(800, 600);
  backgroundSetup();
  //println(Serial.list());
  if (Serial.list().length>0)
    myPort = new Serial(this, Serial.list()[0], 115200);
}

void draw() {
  p = dist(x, y, Tx, Ty);
  a = degrees(atan2(Ty-y, Tx-x))-th;
  if (a>180) a-=360;
  if (a<-180) a+=360;
  b = -th-a+Tth;
  if (b>180) b-=360;
  if (b<-180) b+=360;

  v = Kp * p;
  w = Ka * a + Kb * b;
  //  v = Kv * dist(Tx, Ty, x, y);
  //  Tth = degrees(atan2(Ty-y, Tx-x));
  //  if(Tth>180) Tth-=360;
  //  float Dth = Tth-th;
  //  if (Dth>180) Dth-=360;
  //  if (Dth<-180) Dth+=360;
  //  w = Kw * (Dth);
  //  
  th += w / 20;
  x += v*cos(radians(th))/20;
  y += v*sin(radians(th))/20;

  stroke(255, 0, 0);
  noFill();
  drawTriangle(width/2+x, height/2-y, 10, th);
  stroke(255, 255, 0);
  line(width/2+x, height/2-y, width/2+Lx, height/2-Ly);
  Lx = x;
  Ly = y;

  float Wl = (v+8*w)/5;
  float Wr = (v+8*w)/5;
  //println(Wl, Wr);
  maxW = max(max(maxW, Wl), Wr);

  if (followingPath) {
    if (dist(Tx, Ty, x, y) < 5) {
      if (path.size() == 1) {
        path.remove(0);
        followingPath = false;
        drawing = true;
        //       println(maxW);
        return;
      }
      path.remove(0);
      Tx = path.get(0)[0];
      Ty = path.get(0)[1];
      if (Serial.list().length>0)
        myPort.write("G" + str(Tx) + " " + str(Ty));
      if (path.size()>1) {
        //        Tth = th;
        Tth = atan2(path.get(1)[1], path.get(1)[0]);
        if (Tth>180) Tth-=360;
      }
    }
  }
}

void mouseClicked() {
  if (drawing) {
    float p[] = {
      mouseX-width/2, height/2-mouseY
    };
    if (lastMouseX == -1) {
      backgroundSetup();
      path = new ArrayList<float[]>();
      stroke(0, 255, 0);
      line(mouseX, mouseY, width/2+x, height/2-y);
      path.add(p);
    } else {
      stroke(0, 255, 0);
      line(mouseX, mouseY, lastMouseX, lastMouseY);
      path.add(p);
    }
    lastMouseX = mouseX;
    lastMouseY = mouseY;
    //    backgroundSetup();
    //    path = new ArrayList<float[]>();
    //  }
  }
}

void mouseDragged() {
  //stroke(0, 255, 0);
  //  line(mouseX, mouseY, pmouseX, pmouseY);
  //  float p[] = {
  //    pmouseX-width/2, height/2-pmouseY
  //  };
  //  path.add(p);
}
void mouseReleased() {
  float p[] = {
    mouseX-width/2, height/2-mouseY
  };
  path.add(p);
}

void keyPressed() {
  switch(keyCode) {
  case UP:
    Tth = 90;
    break;
  case DOWN:
    Tth = -90;
    break;
  case LEFT:
    Tth = 180;
    break;
  case RIGHT:
    Tth = 0;
    break;
  }
  if (key == ' ') {
    followingPath = true;
    drawing = false;
    println(path.size());
    print("{");
    for (int i = 0; i<path.size (); i++) {
      print("{");
      print((int)((x-path.get(i)[0])*0.7));
      print(',');
      print((int)((path.get(i)[1]-y)*0.7));
      if (i == path.size()-1) {
        print("},");
      } else {
        print("}};");
      }
    }
  }
}

void drawTriangle(float _x, float _y, float _r, float _th) {
  float[] t1 = {
    _x+_r*cos(radians(_th)), _y-_r*sin(radians(_th))
    };
  float[] t2 = {
    _x+_r*cos(radians(_th+120)), _y-_r*sin(radians(_th+120))
    };
  float[] t3 = {
    _x+_r*cos(radians(_th+240)), _y-_r*sin(radians(_th+240))
    };
    ellipse(t1[0], t1[1], 5, 5);
  triangle(t1[0], t1[1], t2[0], t2[1], t3[0], t3[1]);
}

void backgroundSetup() {
  background(0);
  stroke(255, 0, 0);
  noFill();
  drawTriangle(width/2+x, height/2-y, 10, th);

  fill(255);
  textSize(20);
  textAlign(CENTER, CENTER);
  text("Kρ = " + Kp, width/6, 40);
  text("Kα = " + Ka, 3*width/6, 40);
  text("Kβ = " + Kb, 5*width/6, 40);
  stroke(255, 255, 0);
  strokeWeight(5);
  line(560, 580, 770, 580);
  strokeWeight(1);
  for (int i = 560; i<=770; i+=70) {
    line(i, 580, i, 565);
    text(str((i-560)/70*50), i, 550);
  }
}

