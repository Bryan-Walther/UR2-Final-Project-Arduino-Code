# Unified Robotics II Robot Arm C# Code

Overview
-
Design, Build, and Program a small robotic arm capable of locating, picking up, and sorting triangles and squares.

Source Code for the Arduino and Motor control elements of UR2 Term Project. These sections were programmed in Arduino IDE using C++.

Additional resources can be found in the other repositories on my account.

Functionallity
-
Designed for an Arduino Uno.

Pin Layout:

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

Contributor
-
Bryan Walther: Student, Lawrence Technological University
