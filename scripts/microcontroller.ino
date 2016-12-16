
#include "ros.h"
#include "std_msgs/Float64.h"


int x = 5;         // output pin throttle
int y = 6;         // output pin yaw
int z = 9;         // output pin pitch
int fi = 10;       // output pin roll

const int T = 100; // period in microseconds (f = 10000Hz)

ros::NodeHandle nh;



void cbX(std_msgs::Float64& msg){
  int duty_cycle = msg.data;
  digitalWrite(x, HIGH);
  delayMicroseconds(int(duty_cycle*T));
  digitalWrite(x, LOW);
  delayMicroseconds(int(T - duty_cycle*T));
}

ros::Subscriber<std_msgs::Float64> subX("manual_output/throttle", &cbX);




void cbY(std_msgs::Float64& msg){
  int duty_cycle = msg.data;
  digitalWrite(y, HIGH);
  delayMicroseconds(int(duty_cycle*T));
  digitalWrite(y, LOW);
  delayMicroseconds(int(T - duty_cycle*T));
}

ros::Subscriber<std_msgs::Float64> subY("manual_output/yaw", &cbY);




void cbZ(std_msgs::Float64& msg){
  int duty_cycle = msg.data;
  digitalWrite(z, HIGH);
  delayMicroseconds(int(duty_cycle*T));
  digitalWrite(z, LOW);
  delayMicroseconds(int(T - duty_cycle*T));
}

ros::Subscriber<std_msgs::Float64> subZ("manual_output/pitch", &cbZ);




void cbFI(std_msgs::Float64& msg){
  int duty_cycle = msg.data;
  digitalWrite(fi, HIGH);
  delayMicroseconds(int(duty_cycle*T));
  digitalWrite(fi, LOW);
  delayMicroseconds(int(T - duty_cycle*T));
}

ros::Subscriber<std_msgs::Float64> subFI("manual_output/roll", &cbFI);


void setup() {
  // put your setup code here, to run once:
  pinMode(x, OUTPUT);
  pinMode(y, OUTPUT);
  pinMode(z, OUTPUT);
  pinMode(fi, OUTPUT);
  nh.initNode();
  nh.subscribe(subX);
  nh.subscribe(subY);
  nh.subscribe(subZ);
  nh.subscribe(subFI);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delayMicroseconds(10);
}
