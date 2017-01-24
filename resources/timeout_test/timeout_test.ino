#include <ros.h>
#include <std_msgs/Empty.h>
// #include <avr/interrupt.h>
// #include <avr/io.h>

#define LED 13

ros::NodeHandle  nh;

// unsigned long t;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED, HIGH-digitalRead(LED));   // blink the led
  // unsigned long t_now = millis();
  nh.loginfo("called now");
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup()
{ 
  pinMode(LED, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  // t = millis();
  UBRR0H = 0;//(BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register 
  UBRR0L = 8; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ10); // Use 8-bit character sizes 
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);   // Turn on the transmission, reception, and Receive interrupt 

  interrupts();
}

void loop()
{  
  nh.spinOnce();
  delay(1);

}

// stackoverflow.com/questions/10201590/arduino-serial-interrupts
// http://www.engblaze.com/we-interrupt-this-program-to-bring-you-a-tutorial-on-arduino-interrupts/