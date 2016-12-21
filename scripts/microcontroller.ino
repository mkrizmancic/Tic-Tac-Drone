#include "ros.h"
#include "tic_tac_drone/CtrlValue.h"

const int throttle = 3;         // output pin throttle
const int yaw = 6;         // output pin yaw
const int pitch = 9;         // output pin pitch
const int roll = 10;       // output pin roll

ros::NodeHandle nh;

void cb(tic_tac_drone::CtrlValue& msg){
  analogWrite(throttle, 255 * msg.throttle);
  analogWrite(yaw, 255 * msg.yaw);
  analogWrite(pitch, 255 * msg.pitch);
  analogWrite(roll, 255 * msg.roll);
}

ros::Subscriber<tic_tac_drone::CtrlValue> sub("manual_output", &cb);


void setup() {
  // put your setup code here, to run once:
  pinMode(throttle, OUTPUT);
  pinMode(yaw, OUTPUT);
  pinMode(pitch, OUTPUT);
  pinMode(roll, OUTPUT);

  analogWrite(throttle, 0);
  analogWrite(yaw, 127);
  analogWrite(pitch, 127);
  analogWrite(roll, 127);

  delay (3000);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delayMicroseconds(10);
}
