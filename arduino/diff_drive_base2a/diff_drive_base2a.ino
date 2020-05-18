/* ROBOT 2WD FIRMWARE
 * 
 * Minimal diff drive robot base firmware
 * works with ros diff_drive_controller 
 * using the appropriate hardware inferface node
 * 
 * subscribe to wheel_control to get motor power
 * publish to wheel_left & wheel_right to send encoder tickets
 * 
 * version with standart message Float32MultiArray/WheelControl
 * 
 */

// MOTOR LEFT
#define ENA 5
#define IN1 4
#define IN2 7

// MOTOR RIGHT
#define ENB 6
#define IN3 8
#define IN4 11

// MOTOR LEFT
#define leftMotorEn ENA
#define leftMotorForward IN1
#define leftMotorBackward IN2

// MOTOR RIGHT
#define rightMotorEn ENB
#define rightMotorForward IN3
#define rightMotorBackward IN4

// ENCODER LEFT
#define encoder0int  0
#define encoder0PinA  2
#define encoder0PinB  9

// ENCODER RIGHT
#define encoder1int  1
#define encoder1PinA  3
#define encoder1PinB  10

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

volatile long int encoder0Pos = 0;
volatile long int encoder1Pos = 0;
long int encoder0PosLast = 0;
long int encoder1PosLast = 0;

#define ENCODER_DELAY 20
unsigned long encoderPublisherTimer;

int leftMotorPwmOut = 0;
int rightMotorPwmOut = 0;

std_msgs::Int32 wheel_msg;

ros::Publisher wheelLeftPublisher("wheel_left", &wheel_msg);
ros::Publisher wheelRightPublisher("wheel_right", &wheel_msg);

void wheelControlCallback(const std_msgs::Float32MultiArray& wheel_control_msg) {
  leftMotorPwmOut = (int)(wheel_control_msg.data[0] * 255);
  rightMotorPwmOut = (int)(wheel_control_msg.data[1] * 255);
}

ros::Subscriber<std_msgs::Float32MultiArray> wheelControlSubscriber("wheel_control", &wheelControlCallback);

void setup() {

  // MOTOR LEFT
  pinMode(leftMotorEn, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  
  // MOTOR RIGHT
  pinMode(rightMotorEn, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  // ENCODER LEFT
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);
  attachInterrupt(encoder0int, doEncoder0, CHANGE);

  // ENCODER RIGHT
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);
  attachInterrupt(encoder1int, doEncoder1, CHANGE);

  // Setup the ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(wheelControlSubscriber);
  nh.advertise(wheelLeftPublisher);
  nh.advertise(wheelRightPublisher);

  // Setup the encoder publisher timer
  encoderPublisherTimer = millis();
}

void loop() {
  long int dticks;

  // Update motors
  bodyMotorsControl();

  if (millis() >= encoderPublisherTimer) {
    // publish left wheel ticks
    dticks = encoder0Pos - encoder0PosLast;
    encoder0PosLast = encoder0Pos;
    wheel_msg.data = dticks;
    wheelLeftPublisher.publish(&wheel_msg);
    
    // publish righ wheel ticks
    dticks = encoder1Pos - encoder1PosLast;
    encoder1PosLast = encoder1Pos;
    wheel_msg.data = dticks;
    wheelRightPublisher.publish(&wheel_msg);

    // update timer
    encoderPublisherTimer += ENCODER_DELAY;
  }

  nh.spinOnce();
  
  delay(2);
}

//
// encoder update
//

void doEncoder0() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

//
// motor control
//

void bodyMotorsControl() {

  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {
    //setBodyMotorLeftBrake();
    setBodyMotorLeftStop();
    analogWrite(leftMotorEn, 0);
  } else if(leftMotorPwmOut > 0) {
    setBodyMotorLeftForward();
    analogWrite(leftMotorEn, leftMotorPwmOut);
  } else {
    setBodyMotorLeftBackward();
    analogWrite(leftMotorEn, abs(leftMotorPwmOut));
  }

  // set motor right direction & speed
  if(rightMotorPwmOut == 0) {
    //setBodyMotorRightBrake();
    setBodyMotorRightStop();
    analogWrite(rightMotorEn, 0);
  } else if(rightMotorPwmOut > 0) {
    setBodyMotorRightForward();
    analogWrite(rightMotorEn, rightMotorPwmOut);
  } else {
    setBodyMotorRightBackward();
    analogWrite(rightMotorEn, abs(rightMotorPwmOut));
  }
}

void setBodyMotorLeftForward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
}

void setBodyMotorLeftBackward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
}

void setBodyMotorRightForward() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void setBodyMotorRightBackward() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void setBodyMotorLeftStop() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
}

void setBodyMotorRightStop() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void setBodyMotorLeftBrake() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, HIGH);
}

void setBodyMotorRightBrake() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
}
