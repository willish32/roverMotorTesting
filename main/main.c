// /*  Rover motor controller
//     William Whitfield 12/26/2021

//     This program runs on Espressif's ESP32-DevKitC to control the motors of
//     the planetary rover
//     It takes in a velocity setpoint and then maintains the velocity for the
//     motors using a PID controller and telemtetry from the encoders
//     It will also send back telemetry to the Nvidia Jetson for the localization
//     algorithim

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "sdkconfig.h"
// #include "assignments.h"

// void app_main(void)
// {

// }
// */

// //Rover subsystem test for all six motors and two encoders
// //Rover Team
// //Powers the motor simulating forward motion
// //Takes input from optical rotary encoder and
// //returns speed in RPM,RAD/S,DEG/S
// //Sample code: https://automaticaddison.com/how-to-calculate-the-velocity-of-a-dc-motor-with-encoder/
// //+ is CCW
// //- is CW

// #include "pinAssignments.h"

// // ROTARY ENCODER DECLARATIONS
// /*#define LCLK 2 //left encoder white cable (A)
//   #define LDT 4 //left encoder green cable (B)
//   #define RCLK 3 //right encoder white cable (A)
//   #define RDT 13  //right encoder green cable (B)*/

// #define ENC_PULS_REV 100
// #define mSpeed 70

// int speedSet = 70;

// volatile long Lcount = 0; //keep track of pulses
// volatile long Rcount = 0; //keep track of pulses
// int currentStateLCLK;
// int currentStateRCLK;
// int lastStateLCLK;
// int lastStateRCLK;
// String LcurrentDir = "";
// String RcurrentDir = "";

// int interval = 1000; //1 sec interval for measurments
// //int Rinterval = 1000; //1 sec interval for measurments
// long prevMillis = 0; //counters for milliseconds
// //long RprevMillis = 0; //counters for milliseconds
// long currMillis = 0;
// //long RcurrMillis = 0;

// float Lrpm = 0;
// float Rrpm = 0;
// float Lang_velo_rad = 0; //angular velocity
// float Rang_velo_rad = 0; //angular velocity
// float Lang_velo_deg = 0; //angular velocity in degrees
// float Rang_velo_deg = 0; //angular velocity in degrees

// const float rpm_to_radians = 0.10471975512;
// const float rad_to_deg = 57.29578;

// // MOTOR DECLARATIONS
// /*int LmotorPin_1 = 9;
//   int LmotorPin_2 = 11;
//   int RmotorPin_1 = 6;
//   int RmotorPin_2 = 5;*/

// long currTime = 0;  //keep track of time to run the test cases
// long setTime = 0;
// int testCase = 0; //keep track of test case: 0-FWD 1-REV 2-Left 3-Right

// void setup() {
//   //ENCODER SETUP
//   // Setup Serial Monitor
//   Serial.begin(9600);

//   // Set encoder pins as inputs
//   pinMode(FL_ENCODER_A, INPUT);
//   pinMode(FL_ENCODER_B, INPUT);
//   pinMode(FR_ENCODER_A, INPUT);
//   pinMode(FR_ENCODER_B, INPUT);

//   // everytime the pin goes high, pulse
//   attachInterrupt(digitalPinToInterrupt(FL_ENCODER_A), LdetectPulse, RISING);
//   attachInterrupt(digitalPinToInterrupt(FR_ENCODER_A), RdetectPulse, RISING);

//   // Read the initial state of CLK
//   lastStateLCLK = digitalRead(FL_ENCODER_A);
//   lastStateRCLK = digitalRead(FR_ENCODER_A);

//   //MOTOR SETUP *change to an array for future*
//   pinMode(FL_MOTOR_1, OUTPUT);
//   pinMode(FL_MOTOR_2, OUTPUT);
//   pinMode(FR_MOTOR_1, OUTPUT);
//   pinMode(FR_MOTOR_2, OUTPUT);
//   pinMode(ML_MOTOR_1, OUTPUT);
//   pinMode(ML_MOTOR_2, OUTPUT);
//   pinMode(MR_MOTOR_1, OUTPUT);
//   pinMode(MR_MOTOR_2, OUTPUT);
//   pinMode(BL_MOTOR_1, OUTPUT);
//   pinMode(BL_MOTOR_2, OUTPUT);
//   pinMode(BR_MOTOR_1, OUTPUT);
//   pinMode(BR_MOTOR_2, OUTPUT);

//   //make sure all motor pins start at 0
//   analogWrite(FL_MOTOR_1, 0);
//   analogWrite(FL_MOTOR_2, 0);
//   analogWrite(FR_MOTOR_1, 0);
//   analogWrite(FR_MOTOR_2, 0);
//   analogWrite(ML_MOTOR_1, 0);
//   analogWrite(ML_MOTOR_2, 0);
//   analogWrite(MR_MOTOR_1, 0);
//   analogWrite(MR_MOTOR_2, 0);
//   analogWrite(BL_MOTOR_1, 0);
//   analogWrite(BL_MOTOR_2, 0);
//   analogWrite(BR_MOTOR_1, 0);
//   analogWrite(BR_MOTOR_2, 0);

//   while (! Serial); //wait until serial output is open
// }

// void loop() {
//   //begin the test
//   if (testCase == 0)  {
//     setRightMotor();
//     setLeftMotor();
//     Serial.println("Going Forward...");
//   }
// //  else if (testCase == 1) {
// //    goReverse();
// //    Serial.println("Going Reverse...");
// //  }
// //  else if (testCase == 2) {
// //    goLeft();
// //    Serial.println("Going Left...");
// //  }
// //  else if (testCase == 3) {
// //    goRight();
// //    Serial.println("Going Right...");
// //  }
//   else  {
//     speedSet = 0;
//     setRightMotor();
//     setLeftMotor();
//     Serial.println("Test Complete");
//   }
//   testCase++; //increment test case for next run

//   //loop for 5 seconds before changing the test case
//   setTime = millis();
//   while (currTime < setTime + 6000) {
//     //record time
//     currMillis = millis();

//     // If one second has passed, print the number
//     if (currMillis - prevMillis > interval) {
//       prevMillis = currMillis;

//       // Calculate revolutions per minute
//       Lrpm = (float)(Lcount * 60 / ENC_PULS_REV);
//       Lang_velo_rad = Lrpm * rpm_to_radians;
//       Lang_velo_deg = Lang_velo_rad * rad_to_deg;

//       Rrpm = (float)(Rcount * 60 / ENC_PULS_REV);
//       Rang_velo_rad = Rrpm * rpm_to_radians;
//       Rang_velo_deg = Rang_velo_rad * rad_to_deg;

//       //direction for left motor is opposite of right motor, negate outputs
//       Serial.println("LEFT MOTOR");
//       Serial.print(" Pulses: ");
//       Serial.println(-Lcount);
//       Serial.print(" Speed: ");
//       Serial.print(-Lrpm);
//       Serial.println(" RPM");
//       Serial.print(" Angular Velocity: ");
//       Serial.print(-Lang_velo_rad);
//       Serial.print(" rad per second");
//       Serial.print("\t");
//       Serial.print(-Lang_velo_deg);
//       Serial.println(" deg per second");
//       Serial.println();

//       Serial.println("RIGHT MOTOR");
//       Serial.print(" Pulses: ");
//       Serial.println(Rcount);
//       Serial.print(" Speed: ");
//       Serial.print(Rrpm);
//       Serial.println(" RPM");
//       Serial.print(" Angular Velocity: ");
//       Serial.print(Rang_velo_rad);
//       Serial.print(" rad per second");
//       Serial.print("\t");
//       Serial.print(Rang_velo_deg);
//       Serial.println(" deg per second");
//       Serial.println();

//       Lcount = 0;
//       Rcount = 0;
//       currTime = millis();
//     }
//   }
// }


// void LdetectPulse() {

//   int Lval = digitalRead(LDT);

//   if (Lval == LOW) {
//     LcurrentDir = "CWW";
//   }
//   else {
//     LcurrentDir = "CW";
//   }
//   if (LcurrentDir == "CW") {
//     Lcount++;
//   }
//   else {
//     Lcount--;
//   }
// }

// void RdetectPulse() {

//   int Rval = digitalRead(RDT);

//   if (Rval == LOW) {
//     RcurrentDir = "CWW";
//   }
//   else {
//     RcurrentDir = "CW";
//   }
//   if (RcurrentDir == "CW") {
//     Rcount++;
//   }
//   else {
//     Rcount--;
//   }
// }

// /*For motors:
//    Powering motor pin 1 on left side goes forward
//    Powering motor pin 2 on right side goes forward

//    Powering motor pin 2 on left side goes backwards
//    Powering motor pin 1 on right side goes backwards
// */

// void setRightMotors() {
//   //if speed is zero or positive, go forward
//   if (speedSet >= 0 ) {
//     analogWrite(FL_MOTOR_2, 0);
//     analogWrite(FR_MOTOR_1, 0);
//     analogWrite(ML_MOTOR_2, 0);
//     analogWrite(MR_MOTOR_1, 0);
//     analogWrite(BL_MOTOR_2, 0);  
//     analogWrite(BR_MOTOR_1, 0);
//     analogWrite(FL_MOTOR_1, speedSet);
//     analogWrite(FR_MOTOR_2, speedSet);
//     analogWrite(ML_MOTOR_1, speedSet);
//     analogWrite(MR_MOTOR_2, speedSet);
//     analogWrite(BL_MOTOR_1, speedSet);
//     analogWrite(BR_MOTOR_2, speedSet); 
//     //else, go reverse
//     else {
//     int speedSetrev = -1 * speedSet;
//     analogWrite(FL_MOTOR_1, 0);
//     analogWrite(FR_MOTOR_2, 0);
//     analogWrite(ML_MOTOR_1, 0);
//     analogWrite(MR_MOTOR_2, 0);
//     analogWrite(BL_MOTOR_1, 0);
//     analogWrite(BR_MOTOR_2, 0);   
//     analogWrite(FL_MOTOR_2, speedSetrev);
//     analogWrite(FR_MOTOR_1, speedSetrev);
//     analogWrite(ML_MOTOR_2, speedSetrev);
//     analogWrite(MR_MOTOR_1, speedSetrev);
//     analogWrite(BL_MOTOR_2, speedSetrev);  
//     analogWrite(BR_MOTOR_1, speedSetrev);  
//     }
//   }
// }

// void setLeftMotors(){
//       //if speed is zero or positive, go forward
//   if (speedSet >= 0 ) {
//     analogWrite(FL_MOTOR_2, 0);
//     analogWrite(FR_MOTOR_1, 0);
//     analogWrite(ML_MOTOR_2, 0);
//     analogWrite(MR_MOTOR_1, 0);
//     analogWrite(BL_MOTOR_2, 0);  
//     analogWrite(BR_MOTOR_1, 0);
//     analogWrite(FL_MOTOR_1, speedSet);
//     analogWrite(FR_MOTOR_2, speedSet);
//     analogWrite(ML_MOTOR_1, speedSet);
//     analogWrite(MR_MOTOR_2, speedSet);
//     analogWrite(BL_MOTOR_1, speedSet);
//     analogWrite(BR_MOTOR_2, speedSet); 
//     //else, go reverse
//     else {
//     int speedSetrev = -1 * speedSet;
//     analogWrite(FL_MOTOR_1, 0);
//     analogWrite(FR_MOTOR_2, 0);
//     analogWrite(ML_MOTOR_1, 0);
//     analogWrite(MR_MOTOR_2, 0);
//     analogWrite(BL_MOTOR_1, 0);
//     analogWrite(BR_MOTOR_2, 0);   
//     analogWrite(FL_MOTOR_2, speedSetrev);
//     analogWrite(FR_MOTOR_1, speedSetrev);
//     analogWrite(ML_MOTOR_2, speedSetrev);
//     analogWrite(MR_MOTOR_1, speedSetrev);
//     analogWrite(BL_MOTOR_2, speedSetrev);  
//     analogWrite(BR_MOTOR_1, speedSetrev);  
//     }
//   }
// }