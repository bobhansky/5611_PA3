
public class Arm{
  public float len;   // length
  public float ang;   // angle
  public Vec2 start;  // start Position

  public Arm(float l, float a){
    len = l;
    ang = a;
  }
}

// **************************** global variables

Vec2 torsoCenter = new Vec2(450,300);
int armNum = 5;
Vec2 rootR = new Vec2(450,300);
Vec2 rootL = rootR;//new Vec2(torsoCenter.x - 75, torsoCenter.y);
Vec2 endR;
Vec2 endL;
Arm[] armsR = new Arm[armNum];
Arm[] armsL = new Arm[armNum];
float piHalf = 1.5707963;
float[] angleSumR = new float[armNum];   // angleSumR[2]   =  angle0 + angle1 + angle2;
float[] angleSumL = new float[armNum];

int controlArm = 1;

// **************************** global variables ENDS

void setup(){
  size(800,600,P3D);
  surface.setTitle("proj3");

  armsR[0] = new Arm(50, 0.3);
  armsR[1] = new Arm(50, 0.3);
  armsR[2] = new Arm(70, 0.3);
  armsR[3] = new Arm(40, 0.3);
  armsR[4] = new Arm(30, 0.3);


  armsL[0] = armsR[0]; //new Arm(50, 0.3);
  armsL[1] = armsR[1];
  armsL[2] = new Arm(70, 0.3);
  armsL[3] = new Arm(40, 0.3);
  armsL[4] = new Arm(30, 0.3);
}


void solve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;

  // update from end to rootR
  // Right Arm
  for(int i = armNum-1; i >= 0; i--){
    startToGoal = goal.minus(armsR[i].start);
    if (i== 0 && startToGoal.length() < .0001) return;    // important
    startToEndEffector = endR.minus(armsR[i].start);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    
    angleDiff = acos(dotProd);
    // limit angular velocicy
    if(angleDiff > piHalf/50) angleDiff = piHalf/50;

    if(controlArm == 1){
      if (cross(startToGoal,startToEndEffector) < 0)
        armsR[i].ang += angleDiff ;  // smoother motion
      else
        armsR[i].ang -= angleDiff ;
    }

    // angle limits
    if(i == 1){
      if(angleSumR[i-1] +  armsR[i].ang > angleSumR[i-1] + 0.9 * piHalf){
        armsR[i].ang =  0.9 * piHalf;
      } 
      else if (angleSumR[i-1] + armsR[i].ang<=angleSumR[i-1] - 0.9 * piHalf) {
        armsR[i].ang =  -0.9 * piHalf;
      } 
    }
    else if(i == 2){
      if(angleSumR[i-1] +  armsR[i].ang > angleSumR[i-1] + 0.9* piHalf){
        armsR[i].ang =  0.9 * piHalf;
      } 
      else if (angleSumR[i-1] + armsR[i].ang<=angleSumR[i-1] + 0.3 * piHalf) {
        armsR[i].ang =  0.3 * piHalf;
      } 
    }

    else if(i > 2 ){
      if(angleSumR[i-1] +  armsR[i].ang > angleSumR[i-1] + 0.2* piHalf){
        armsR[i].ang =  0.2 * piHalf;
      } 
      else if (angleSumR[i-1] + armsR[i].ang<=angleSumR[i-1] - 0.5 * piHalf) {
        armsR[i].ang =  -0.5 * piHalf;
      } 
    }
    else if(i == 0){
     if(armsR[i].ang >= -HALF_PI) armsR[i].ang = -HALF_PI;
     if(armsR[i].ang <= - piHalf) armsR[i].ang = -piHalf;
    }

    updatearmsR();
  }

  // update from end to rootL
  // Left Arm
  for(int i = armNum-1; i >= 0; i--){
    startToGoal = goal.minus(armsL[i].start);
    if (i== 0 && startToGoal.length() < .0001) return;    // important
    startToEndEffector = endL.minus(armsL[i].start);
    dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
    dotProd = clamp(dotProd,-1,1);
    
    angleDiff = acos(dotProd);
    // limit angular velocicy
    if(angleDiff > piHalf/50) angleDiff = piHalf/50;

    if(controlArm == -1){
      if (cross(startToGoal,startToEndEffector) < 0)
        armsL[i].ang += angleDiff ;  // smoother motion
      else
        armsL[i].ang -= angleDiff ;
    }

    // angle limits

    if(i == 2){
      if(angleSumL[i-1] +  armsL[i].ang > angleSumL[i-1] - 0.3* piHalf){
        armsL[i].ang =  -0.3 * piHalf;
      } 
      else if (angleSumL[i-1] + armsL[i].ang <= angleSumL[i-1] - 0.9 * piHalf) {
        armsL[i].ang =  -0.9 * piHalf;
      } 
    }

    else if(i > 2 ){
      if(angleSumL[i-1] +  armsL[i].ang > angleSumL[i-1] + 0.5 * piHalf){
        armsL[i].ang =  0.5 *piHalf;
      } 
      else if (angleSumL[i-1] + armsL[i].ang <= angleSumL[i-1] - 0.2* piHalf) {
        armsL[i].ang = - 0.2 * piHalf;
      } 
    }
    else if(i == 0){
      // if(armsL[i].ang >= 3*piHalf) armsL[i].ang = 3*piHalf;
      // if(armsL[i].ang <=  piHalf) armsL[i].ang = piHalf;
      // println(armsR[0].ang);
    }

    updatearmsL();
  }
}


void updatearmsR(){
  // update angleSumR
    angleSumR[0] = armsR[0].ang;
    armsR[0].start = rootR;

    for(int i = 1; i < armNum ; i++){
      angleSumR[i] = angleSumR[i-1] + armsR[i].ang;
      float l = armsR[i-1].len;
      armsR[i].start = new Vec2(cos(angleSumR[i-1]) * l, sin(angleSumR[i-1]) * l).plus(armsR[i-1].start);
    }
    endR = new Vec2(cos(angleSumR[armNum-1])*armsR[armNum-1].len, sin(angleSumR[armNum-1])*armsR[armNum-1].len).plus(armsR[armNum-1].start);
}

void updatearmsL(){
  // update angleSumL
    angleSumL[0] = armsL[0].ang;
    armsL[0].start = rootL;

    for(int i = 1; i < armNum ; i++){
      angleSumL[i] = angleSumL[i-1] + armsL[i].ang;
      float l = armsL[i-1].len;
      armsL[i].start = new Vec2(cos(angleSumL[i-1]) * l, sin(angleSumL[i-1]) * l).plus(armsL[i-1].start);
    }
    endL = new Vec2(cos(angleSumL[armNum-1])*armsL[armNum-1].len, sin(angleSumL[armNum-1])*armsL[armNum-1].len).plus(armsL[armNum-1].start);
}


float armW = 20;
void draw(){
  directionalLight(255, 255, 255, 1, 1, -1);  // upper left specular
  directionalLight(255, 255, 255, -1, 1, 1);  
  ambientLight(51, 102, 126);
  background(250,250,250);

  // update User interaction
  if (leftPressed){
    torsoCenter.x -=2;
    rootL.x -= 2;
    rootR.x -=2;
  }
  if (rightPressed) {
    torsoCenter.x +=2;
    rootL.x += 2;
    rootR.x +=2;
  }
  if (upPressed) {
    torsoCenter.y -=2;
    rootL.y -= 2;
    rootR.y -=2;
  }
  if (downPressed)  {
    torsoCenter.y +=2;
    rootL.y += 2;
    rootR.y +=2;
  }


  updatearmsR();
  updatearmsL();
  solve();
  
  // torso
  fill(23,151,112);
  pushMatrix();
  int x = 70, y = 100, z = 50;
  translate(torsoCenter.x-35 , torsoCenter.y, -35);
   //rect(-75, 0, 75, 200);
   //box(x, y, z);
  popMatrix();
  
  // darw each arm
  for(int i = 0; i < armNum; i++){

    fill(200,0,20);
    // // draw right
    // draw root
    if(i == 0){
        fill(23,151,112);
        pushMatrix();
        translate(rootR.x , rootR.y-10, -3);
        rotate(angleSumR[i]);
        box(70, 10, 20);
        popMatrix();
    }
    else{
      pushMatrix();
      translate(armsR[i].start.x, armsR[i].start.y );
      rotate(angleSumR[i]);
      rect(0, -armW/2, armsR[i].len, armW);
      // box(armsR[i].len, armW, armW);
      popMatrix();
    }

    //draw root sphere
    pushMatrix();
    translate(armsR[1].start.x, armsR[1].start.y, -5);
    noStroke();
    sphere(10);
    stroke(1);
    popMatrix();

    // draw left
    if(i==0 || i ==1) continue;
    pushMatrix();
    translate(armsL[i].start.x, armsL[i].start.y);
    rotate(angleSumL[i]);
    rect(0, -armW/2, armsL[i].len, armW);
    //box(armsR[i].len, armW/2, armsR[i].len);
    popMatrix();
  }

  fill(100,200,20);
  pushMatrix();
   noStroke();
  translate(mouseX,mouseY);
  sphere(20);
  popMatrix();
  stroke(1);
  //ellipse(mouseX, mouseY, 20, 20);
  //println(controlArm);

}




boolean leftPressed, rightPressed, upPressed, downPressed;
void keyPressed(){
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true; 
  if (keyCode == DOWN) downPressed = true;
}


void keyReleased(){
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false; 
  if (keyCode == DOWN) downPressed = false;

  if(key == 'c') controlArm = -controlArm;

}

