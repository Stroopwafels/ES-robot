#include <ros.h>
#include <std_msgs/Empty.h>

#define LED 13

ros::NodeHandle  nh;

unsigned long t;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED, HIGH-digitalRead(LED));   // blink the led
  unsigned long t_now = millis();
  nh.loginfo("called now");
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup()
{ 
  pinMode(LED, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  t = millis();
}

void loop()
{  
  nh.spinOnce();
  delay(1);

}
