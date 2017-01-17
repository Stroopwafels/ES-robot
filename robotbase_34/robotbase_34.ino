#include <ros.h>
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define HB_1REV   7
#define HB_1EN    24
#define HB_1FWD   6
#define HB_2REV   3
#define HB_2EN    25
#define HB_2FWD   2

double vmax = 0.16;
unsigned long t = millis();


ros::NodeHandle nh;

void messageCb(const geometry_msgs::Twist &twist) { // const std_msgs::Empty& toggle_msg){
  t = millis();
  convert2motor(twist);
}

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &messageCb );

void setup()
{ 
  pinMode(HB_1REV, OUTPUT);
  pinMode(HB_1EN, OUTPUT);
  pinMode(HB_1FWD, OUTPUT);
  pinMode(HB_2REV, OUTPUT);
  pinMode(HB_2EN, OUTPUT);
  pinMode(HB_2FWD, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

void convert2motor(geometry_msgs::Twist twist) {

  //double vmax = 0.16;
  
  double lx = twist.linear.x;
  double az = twist.angular.z; 

  double rotsp = az * 0.0475;

  double vl = -rotsp + lx;
  double vr = rotsp + lx;

  digitalWrite(HB_1EN, HIGH);
  digitalWrite(HB_2EN, HIGH);

  clip_v(&vl, &vr);
  if ((millis()-t) < 100) {
    if (vl >= 0) {
      analogWrite(HB_1FWD, 255*vl/vmax);
      analogWrite(HB_1REV, 0);
    } else {
      analogWrite(HB_1FWD, 0);
      analogWrite(HB_1REV, -255*vl/vmax);
    }
    if (vr >= 0) {      
      analogWrite(HB_2FWD, 255*vr/vmax);
      analogWrite(HB_2REV, 0);
    } else {
      analogWrite(HB_2FWD, 0);
      analogWrite(HB_2REV, -255*vr/vmax);
    }
  } else {
    analogWrite(HB_1FWD, 0);
    analogWrite(HB_1REV, 0);
    analogWrite(HB_2FWD, 0);
    analogWrite(HB_2REV, 0);
  }

}

void clip_v(double *vl, double *vr) {
  if (abs(*vl) > vmax) {
    *vr = *vr * vmax/ abs(*vl);
    *vl = *vl * vmax/ abs(*vl);
  }

  if (abs(*vr) > vmax) {
    *vl = *vl * vmax / abs(*vr);
    *vr = *vr * vmax / abs(*vr);
  }
}

// 0.16 m/s is maximum speed

