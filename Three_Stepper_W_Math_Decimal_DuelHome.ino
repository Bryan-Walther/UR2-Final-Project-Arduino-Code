/*
 * Bryan Walther
 * Unified Robotics 2 Project
 * Due Date: December 12th 2018
 *  
 * Run 3 Stepper motors with L298N
 * Take in Multiple Serial inouts using the <### ### ###> notation
 * * Includes up to one decimal point of precision
 * Calculate the required steps off of given distances
 * Read a series of commands from serial and execute appropriate responce
 * Home Elbow and Shoulder stepper motors at the same time to reduce stress on both
 */

/*=====================================================
 * 
 * Phisical dimentions and arm Variables
 * 
 * ====================================================
 */

//Phisical Arm Constants (Inch)
const double lengthLower = 12.25;
const double lengthUpper = 11.271;

const double baseLength = 3.625;
const double baseHeight = 3.54;
////Lower Arm (radians)
const double alpha1 = 0.078128;
const double lower1 = 2.002;
const double lower2 = 3.511;
////Upper Arm
const double L1 = 2.002;
const double L2 = 5.386;
const double L3 = 1.639;
const double phi1 = asin(L1/L2);

//Motor Assemblies
const double maElbow = 1.743;
const double maShoulder = 1.64;
const double NutAssemb = 0.35;

//inch to steps
const int inchToStep = 200*13;

//Zero Inch Pos for Arms
const double elbowZeroInch = 3.766;
const double shoulderZeroInch = 1.575;

//Degrees per Step
const double degToStep = 1.8;

//Basic distance and Constants
double DistX;
double DistY;
//Angle Variables (Reference drawing, these angles are the ones that are affected by the screws directly)
double bigAlpha;
double Phi2;

//Screw Lengths (inch)
double screwLower;
double screwUpper;

/*=====================================================
 * 
 * Control Variables
 * 
 * ====================================================
 */

//Shoulder Control Pins
const int ControlS1 = 2; 
const int ControlS2 = 3; 
const int ControlS3 = 4; 
const int ControlS4 = 5;

//Elbow Control Pins
const int ControlE1 = 6; 
const int ControlE2 = 7; 
const int ControlE3 = 8; 
const int ControlE4 = 9; 

//Base control Pins
const int ControlB1 = 10;
const int ControlB2 = 11;
const int ControlB3 = 12;
const int ControlB4 = 13;

//Limit switches pins
const int LimitE = A0;
const int LimitS = A1;
const int LimitB = A2;

//Magnet Pin
const int MagnetPin = A3;

//Elbow Control Variables
bool reverseE = 0; //Reverse: 1 = Up, 0 = down
bool StateEa = 1;
bool StateEb = 1;

int ETargetStep = 0;
int ECurrentStep = 0;
int EUpperStep = 5200; //?

//Shoulder Control Variables
bool reverseS = 0; //Reverse: 1 = Up, 0 = down
bool StateSa = 1;
bool StateSb = 1;

int STargetStep = 0;
int SCurrentStep = 0;
int SUpperStep = 3400; //?

//Base Control Variables
bool reverseB = 1; //Reverse: 1 = Counter-Clockwise, 0 = Clockwise
bool StateBa = 1;
bool StateBb = 1;

int BTargetStep = 0;
int BCurrentStep = 0;
int BUpperStep = 3000;//100; //?

//Delay Speed
const int speedMS = 5;
const int BaseSpeed = 45;

//Serial MultiByte messages test
const int ArrayLength = 3;
double readValues[ArrayLength]; //{xxx yyy TTT}

//Command Variables
bool doMath = false;
bool motionComplete = false;
bool nextCommand = false;
char commandGiven = ' ';
bool magnetState = false;

//Setup: Set pin modes and start serial communication
void setup() {
  pinMode(ControlE1, OUTPUT);
  pinMode(ControlE2, OUTPUT);
  pinMode(ControlE3, OUTPUT);
  pinMode(ControlE4, OUTPUT);

  pinMode(ControlS1, OUTPUT);
  pinMode(ControlS2, OUTPUT);
  pinMode(ControlS3, OUTPUT);
  pinMode(ControlS4, OUTPUT);

  pinMode(ControlS1, OUTPUT);
  pinMode(ControlS2, OUTPUT);
  pinMode(ControlS3, OUTPUT);
  pinMode(ControlS4, OUTPUT);

  pinMode(LimitE, INPUT);
  pinMode(LimitS, INPUT);
  pinMode(LimitB, INPUT);

  pinMode(MagnetPin, OUTPUT);
  digitalWrite(MagnetPin, LOW);
  
  Serial.begin(9600);
}

//Main loop: Check for next command, set to read next command, set to send command to Robot
void loop() {
  if(nextCommand){
    promptNextCommand();
  }
  
  if(Serial.available()>0){
    if(Serial.read() == '<'){ //if start with '<', is valid command
      //Check for commands being sent
      
      readSerialByteArray();

      //command was a new position, run math to location, else ignore math portion
      if(doMath){
        mathSerialToSteps();
        mathStepAngle();
      }
      //Display(); //used during Debug
    }
  }
  switch(commandGiven){
    case 'P'://psotion steppers
      positionSteppers();

      // if all motors finished their actions, prompt next command
      if(motionComplete){
          nextCommand = true;
          commandGiven = ' ';
      }
      break;
    case 'M':    //Turn On Magnet
      magnetState = !magnetState;
      digitalWrite(MagnetPin, magnetState);
      Serial.println(magnetState);
      commandGiven = ' ';
      nextCommand = true;
      break;
    case 'H': //home robot
      HomeJoints();
      commandGiven = ' ';
      nextCommand = true;
      break;
    default:// wait
      break;  
  }
}

//Does what you think it will do....
void promptNextCommand(){
  Serial.println("<N>");
  nextCommand = false;
}

//Display current position and target Steps (debug only)
/*
void Display(){
  Serial.println("Motor Elbow:");
  Serial.println(readValues[0]);
  Serial.print(ETargetStep);
  Serial.print(", ");
  Serial.println(ECurrentStep);

  Serial.println("Motor Shoulder:");
  Serial.println(readValues[1]);
  Serial.print(STargetStep);
  Serial.print(", ");
  Serial.println(SCurrentStep);

  Serial.println("Motor Base:");
  Serial.println(readValues[2]);
  Serial.print(BTargetStep);
  Serial.print(", ");
  Serial.println(BCurrentStep);

  Serial.println(" ");
}
*/

//region moveStepper to Target Step
void positionSteppers(){
  positionBase();
  positionElbow();
  positionShoulder();
}

void positionElbow(){
  if(ECurrentStep > ETargetStep){ //Bring Arm down
    reverseE = 0;
    //check that is not at limit switch
    if(!digitalRead(LimitE)){ //Safe to move arm down
      moveElbow();
      ECurrentStep--;
      
    }
    else{ //Ran into Limit switch, return to Zero point
      reverseE = 1;
      for(int i = 0; i < 200; i++){
        moveElbow();
      }
      ECurrentStep = 0;
      ETargetStep = 0;
    }
  }
  else if(ECurrentStep < ETargetStep){//Bring Arm Up
    reverseE = 1;
    //Check that is below upper limit
    if(ETargetStep > EUpperStep){ //Command was sent too high, correct to max reach
      ETargetStep = EUpperStep;
    }
    //no issues reamin, Safe to move arm up
    moveElbow();
    ECurrentStep++;
  }

  //if target step was reached, state that is finished
  if(ECurrentStep == ETargetStep){
    motionComplete = true;
  }
  else{
    motionComplete = false;
  }
}

void positionShoulder(){
  if(SCurrentStep > STargetStep){ //Bring Arm up
    reverseS = 0;
    //check if not at Limit switch
    if(!digitalRead(LimitS)){ //Safe to move arm
      moveShoulder();
      SCurrentStep--;
    }
    else{//limit reached, return to zero point
      reverseS = 1;
      for(int i = 0; i < 200; i++){
        moveShoulder();
      }
      SCurrentStep = 0;
      STargetStep = 0;
    }
  }
  else if(SCurrentStep < STargetStep){ //Bring Arm Down
    reverseS = 1;
    //check that below Upper limit
    if(STargetStep > SUpperStep){ //Command was sent too high, correct to max reach
      STargetStep = SUpperStep;
    }
    //no issues remain, Safe to move arm up
    moveShoulder();
    SCurrentStep++;
  }

  //target reached, state that is finished
  if(SCurrentStep == STargetStep){
    motionComplete = true;
  }
  else{
    motionComplete = false;
  }
}

void positionBase(){
  if(BCurrentStep > BTargetStep){ //Bring Arm CCW
    reverseB = 1;
    //check if not at Limit switch
    if(!digitalRead(LimitB)){ //Safe, move arm
      moveBase();
      BCurrentStep--;
    }
    else{//limit reached, move arm to zero point
      reverseB = 0;
      for(int i = 0; i < 10; i++){
        moveBase();
      }
      BCurrentStep = 0;
      BTargetStep = 0;
    }
  }
  else if(BCurrentStep < BTargetStep){ //Bring Arm CW
    //check that below Upper limit
    reverseB = 0;
    if(BTargetStep > BUpperStep){ //Command was sent too high, correct to max reach
      BTargetStep = BUpperStep;
    }
     //no issues, Safe to move arm up
    moveBase();
    BCurrentStep++;
  }

  //if target reached, state that movement is finished
  if(BCurrentStep == BTargetStep){
    motionComplete = true;
  }
  else{
    motionComplete = false;
  }
}
//endRegion moveSteppers


//region ALL THE MATH!!!
void mathSerialToSteps(){ //determain elbow and shoulder steps based off of given inch distances
  //Names are in reference to skeletal model
  DistX = readValues[0] + baseLength;
  DistY = readValues[1] - baseHeight;

  double DistC = sqrt(sq(DistX) + sq(DistY));

  lower angle
  double d1 = atan(DistY/DistX);
  double d2 = acos((sq(lengthLower) + sq(DistC) - sq(lengthUpper)) / (2*lengthLower*DistC));
  bigAlpha = alpha1 + d1 + d2; //d1+d2 = Theta 1

  //Upper interior angle
  double Theta2 = acos((sq(lengthLower) + sq(lengthUpper) - sq(DistC)) / (2*lengthLower*lengthUpper));
  Phi2 = PI - (Theta2 + phi1);

  //determain total screw length (including assemblies)
  screwLower = sqrt(sq(lower1) + sq(lower2) - 2*lower1*lower2*cos(bigAlpha));
  screwUpper = sqrt(sq(L2) + sq(L3) - 2*L2*L3*cos(Phi2));

  //determain screw lengths (w/o assemblies)
  double elbowDistFromZero = elbowZeroInch - (screwUpper - maElbow - NutAssemb);
  double shoulderDistFromZero = shoulderZeroInch - (screwLower - maShoulder - NutAssemb);

  //convert inch distance to step rotations
  ETargetStep = elbowDistFromZero * inchToStep;
  STargetStep = shoulderDistFromZero * inchToStep;
}

void mathStepAngle(){//determain base steps based off of given degree from centerline
  int BaseAngle = readValues[2] + 90;
  BTargetStep = BaseAngle / degToStep;
}
//endregion MATH!!

//region ReadSerialCommands
void readSerialByteArray(){ //read incoming serial messages, once char at a time
  float parsedValue;
  int arrayIndex = 0;
  char readByte;
  bool firstChar = true;
  int negMult = 1;
  bool decimalValue = false;
  do{
    if(Serial.available() > 0){
      readByte = Serial.read();


      if(readByte == '-' && firstChar){ //for negative values, remeber there was a negative sign (multiply to number later)
        negMult = -1;
      }
      if(readByte > 47 && readByte < 58){//if value is a number (0-9)
        if(firstChar){//zero out current value
          readValues[arrayIndex] = 0;
          firstChar = false;
        }
        parsedValue = (readByte - '0') * negMult;
        if(decimalValue){ //at max, one decimal point can be sent as well, caught by this part here
          readValues[arrayIndex] = (readValues[arrayIndex]) + (parsedValue)/10;
        }
        else{//not a negative value, shift all current number up one position and append new value
          readValues[arrayIndex] = (readValues[arrayIndex]*10) + parsedValue;
        }
        if(!doMath){
          doMath = true; //new position was sent, will need to redo math for positions
        }
        commandGiven = 'P'; //new command was a position, treat acordingly
      }
      else if(readByte == '.'){ //decimal catch
        decimalValue = true;
      }
      else if(readByte == ' '){ //next value was sent, moves from <XXX YYY TTT>
        arrayIndex++;
        negMult = 1; //reset segatives and decimal catches
        decimalValue = false;
        firstChar = true;
        
        if(arrayIndex == ArrayLength){//too many values sent, ignore all following (only 3 numbers recived max)
          readByte = '>';
        }
      }
      else if(readByte == 'H'){ //come command passed, respond accordingy
        commandGiven = 'H';
        doMath = false; //no new math
        readByte = '>';
      }
      else if(readByte == 'M'){ //magnet command passed, respond acordingly
        commandGiven = 'M';
        doMath = false;
        readByte = '>';
      }
    }
  }while(readByte != '>'); //as long as end command char was not sent, keep reading
}
//endregion ReadSerialCommands

//region HomeSteppers
void HomeJoints(){
  //allows elbow and shoulder to be homed at the same time, reduces strain on both joints
  bool eHomed = false;
  bool sHomed = false;
  while(!eHomed || !sHomed){ //if either is still moving
    if(!eHomed){ //if elbow needs to continue
        eHomed = homeElbow(); 
    }
    if(!sHomed){//if shoulder needs to continue
      sHomed = homeShoulder();
    }
  }

  //home base afterwards, no point doing this one with the others
  homeBase();
}

bool homeElbow(){
  reverseE = 0;
  if(!digitalRead(LimitE)){ //as long as limit not reached, bring arm in
    moveElbow();
    return false;
  }
  else{ //limit reached, move one rotation and assign current location to zero. set finished movement to true;
    reverseE = 1;
    for(int i = 0; i < 200; i++){
      moveElbow();
    }
    ECurrentStep = 0;
    ETargetStep = 0;
    return true;
  }
}

bool homeShoulder(){
  reverseS = 0;
  if(!digitalRead(LimitS)){//until limit reached, bring arm up
    moveShoulder();
    return false;
  }
  else{//limit reached, move arm one rotation and set as zero
    reverseS = 1;
    for(int i = 0; i < 200; i++){
      moveShoulder();
    }
    SCurrentStep = 0;
    STargetStep = 0;
    return true;
  }
}

void homeBase(){
  reverseB = 1;
  while(!digitalRead(LimitB)){ //move until limit was reached
    moveBase();
  }
  reverseB = 0;
  for(int i = 0; i < 2; i++){ //move 2 steps (3.6 degrees) and set as zero
    moveBase();
  }
  BCurrentStep = 0;
  BTargetStep = 0;
}
//endregion HomeSteppers

//region MoveElbowSteppers
void moveElbow(){//pull of some xor magic right here. Zach's magic =  confusion but it works
  if(reverseE ^ (StateEa ^ StateEb)){
    changeEa();
  }
  else{
    changeEb();
  }
  delay(speedMS);
}

void changeEa(){
  StateEa = !StateEa;//write confusion Zach magic here, not entirely shure how it works, but it does
  digitalWrite(ControlE1, StateEa);
  digitalWrite(ControlE2, !StateEa);
}

void changeEb(){
  StateEb = !StateEb;
  digitalWrite(ControlE3, StateEb);
  digitalWrite(ControlE4, !StateEb);
}
//endregion MoveElbowSteppers

//region MoveShoulderStepper
void moveShoulder(){//Same xor zach magic, but this time for the Shoulder
  if(reverseS ^ (StateSa ^ StateSb)){
    changeSa();
  }
  else{
    changeSb();
  }
  delay(speedMS);
}

void changeSa(){
  StateSa = !StateSa;
  digitalWrite(ControlS1, StateSa);
  digitalWrite(ControlS2, !StateSa);
}

void changeSb(){
  StateSb = !StateSb;
  digitalWrite(ControlS3, StateSb);
  digitalWrite(ControlS4, !StateSb);
}
//endregion MoveShoulderStepper

//region MoveBaseStepper
void moveBase(){//You guessed it, xor Zach magic part three for base stepper
  if(reverseB ^ (StateBa ^ StateBb)){
    changeBa();
  }
  else{
    changeBb();
  }
  delay(speedMS+BaseSpeed);
}

void changeBa(){
  StateBa = !StateBa;
  digitalWrite(ControlB1, StateBa);
  digitalWrite(ControlB2, !StateBa);
}

void changeBb(){
  StateBb = !StateBb;
  digitalWrite(ControlB3, StateBb);
  digitalWrite(ControlB4, !StateBb);
}
//endregion MoveBaseStepper

