#include <Servo.h>
#include "trajectory.h"


/**
 * If the acceleration and deceleration are different
 * FORMAT: Trajectory(max velocity, acceleration, deceleration)
 */

Trajectory servoTrajectory(60, 40, 34);

// angles in degree 
// length of links of robot arm
float L0 = 0.095;
float L1 = 0.113;       
float L2 = 0.10;

// Joint angles
float angle1 ;      
float angle2 ;             


unsigned long updateTimer = 0;
int moveNumber = 0;
int servoNumber = 0;
int motor_forward = 6;
int motor_backward = 9;

volatile float pi = 3.14159265359;

Servo myservo1;
Servo myservo2;

// --- Define global variables ---
// The controller will be updated at a rate of 100Hz
#define UPDATE_FREQUENCY 100
#define UPDATE_TIME (1000 / UPDATE_FREQUENCY)


void setup() {
  pinMode(motor_forward, OUTPUT);
  pinMode(motor_backward, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting program");
  
  // Attaches the servo on pin 3&11 to the servo object
  myservo1.attach(3);
  myservo2.attach(11);
 
  // We want the servo to start at an angle of 90°
  myservo1.write(90);
  myservo2.write(90);
  
  Serial.println("Inverse Kinematics begins");
  inverseKinematics();

  updateTimer = millis();
}

/* * * * * * * * * * * * * * * * * * * * * * *
 * LOOP
 * * * * * * * * * * * * * * * * * * * * * * */
void loop() {

  // Update the servo position at regular intervals
  if (millis() - updateTimer >= UPDATE_TIME) {
    updateTimer += UPDATE_TIME;

    // Update the controller
    float currentAngle = servoTrajectory.update();

    // Set the new servo position; the function only takes integer numbers
    if(servoNumber == 1)
    {
        myservo1.write(round(currentAngle));
    }
    else if(servoNumber == 2)
    {
        myservo2.write(round(currentAngle));
    }

    // Output the target position, along with the current position and velocity
    Serial.print("Target: ");
    Serial.print(servoTrajectory.getTarget());
    Serial.print(", Angle: ");
    Serial.print(servoTrajectory.getPos());
    Serial.print(", Velocity: ");
    Serial.println(servoTrajectory.getVel());

    // Only once the servo has reached the desired position, complete the next move
    if (servoTrajectory.ready()) {
      nextMove();
    }
  }
}

void motor_open(){
 analogWrite(motor_forward, 255);
 delay(840);
 analogWrite(motor_forward, 0);  
}

void motor_close(){
 analogWrite(motor_backward, 255);
 delay(840);
 analogWrite(motor_backward, 0);  
}

void inverseKinematics(){
float temp;    
float rad_angle1;  
float rad_angle2; 
float x;
float z;   
   Serial.println("Enter the value x ");
      while(Serial.available()==0);
      x=Serial.parseFloat();
      Serial.print("x is "); 
      Serial.println(x);
      delay(1000);
      
      Serial.println("Enter the value z ");
      while(Serial.available()==0);
      z = Serial.parseFloat() - L0;
      Serial.print("z is "); 
      Serial.println((z+L0));
      delay(1000);
      
      temp = (sq(x) + sq(z) - sq(L1) - sq(L2))/(2*L1*L2);
      Serial.print("temp is  ");
      Serial.println(temp);  
      rad_angle2 = atan2(sqrt(1-sq(temp)),temp);
      rad_angle1 = atan2(x,z) - atan2((L2*sin(rad_angle2)),(L1+ L2*cos(rad_angle2)));

      angle1 = (rad_angle1*180)/pi+90;
      angle2 = (rad_angle2*180)/pi+90;
      
     
 Serial.print("angle1 is  "); 
 Serial.println(angle1);
 Serial.print("angle2 is "); 
 Serial.println(angle2);
 delay(2000);
 }

/* * * * * * * * * * * * * * * * * * * * * * *
 * NEW MOVEMENT COMMANDS
 * * * * * * * * * * * * * * * * * * * * * * */
void nextMove() {
  float t = 0;
  switch (moveNumber) {
    case 0:
      // First we move Link - 1 to the desired angle
      t = round(5*(abs(90-angle1))/180);
      servoTrajectory.reset(90);
      servoNumber = 1;
      servoTrajectory.setTargetPos(angle1,abs(t));
      break;

    case 1:
      // First we move Link - 2 to the desired angle
      t = round(5*(abs(90-angle2))/180);
      servoTrajectory.reset(90);
      servoNumber = 2;
      servoTrajectory.setTargetPos(angle2,abs(t));
      break;
      
    case 2:
    // Gripper close
    motor_close();
    delay(1000);
    break;
   
    case 3: 
    // Link 2 back to original pose
    t = round(5*(abs(90-angle2))/180);
    servoTrajectory.reset(angle2);
    servoNumber = 2;
    servoTrajectory.setTargetPos(90,abs(t));
    break;
    
    case 4:
    // Link 1 back to original pose
    t = round(5*(abs(90-angle1))/180);
    servoTrajectory.reset(angle1);
    servoNumber = 1;
    servoTrajectory.setTargetPos(90,abs(t));
    break;
    
    case 5:
    // Gripper open and close
    motor_open();
    delay(1000);
    break;
    
    default:
      // If all other moves have completed, stop the program
      Serial.println("All moves completed");
      while(1) {}
  }

  moveNumber++;
}
