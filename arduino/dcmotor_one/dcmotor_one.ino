#include <ros.h>
#include <rospy_tutorials/Floats.h>

#define encoder0int  0
#define encoder0PinA  2
#define encoder0PinB  9

#define IN1 4
#define IN2 7
#define ENA 5

ros::NodeHandle  nh;
rospy_tutorials::Floats joint_state;

double pos = 0, vel = 0, output = 0, temp = 0;
unsigned long lastTime, now, lasttimepub;
volatile long int encoder0Pos = 0, last_pos = 0;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg) {
  output = cmd_msg.data[0]; 
}

ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_arduino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &joint_state);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  // bridge-H
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
    
  // encoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(encoder0int, doEncoder0, CHANGE);  
}

void loop(){
  pos = (encoder0Pos * 360) / 740 ;
  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange >= 500 ) {
      temp = (360.0 * 1000 * (encoder0Pos - last_pos)) / (740.0 * (now - lastTime));
      
      // to guard encoder0Pos at boundary i.e., after max limit it will rest. Then lastPos will be greater than encoder0Pos
      if ((encoder0Pos < -2 || encoder0Pos > 2) && temp >= -60 && temp <= 60 )
          vel = temp;
      lastTime = now;
      last_pos = encoder0Pos;
  }

  pwmOut(output);
  
  if ((now - lasttimepub) > 100) {
    joint_state.data_length = 3;
    joint_state.data[0] = pos;
    joint_state.data[1] = vel;
    joint_state.data[2] = output;
    pub.publish(&joint_state);
    lasttimepub = now;
  }

  nh.spinOnce();

}

void doEncoder0() {
  if (encoder0Pos > 740 || encoder0Pos < -740) encoder0Pos = 0;
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void pwmOut(float out) {                                
  if (out > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, out);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(out));
  }
}
