// PFE - Course de vehicules autonomes
// Author : Tom DA SILVA-FARIA

/*
Necessary Arduino libs : 
- VarSpeedServo : servomotor control https://github.com/netlabtoolkit/VarSpeedServo
- rosserial : Setting up Arduino as a ROS Subscriber : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
*/

#include <VarSpeedServo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;
float speed_command; 
float angle_command;
bool is_paused = false ;
 
int pin_PWM = 3   ; int pin_SRV = 10 ;
int pin_REL = 8   ; int current_REL = HIGH ;
int max_PWM = 255 ; 

VarSpeedServo servo ; int speedServo = 50 ; // Lower speed = less reactivity but also less current drawing
float angle_message = 0 ; float angle_scaled = 0 ; 
float min_msg = -1.0 ; float max_msg = 1.0  ; 
float min_ang = 65.0 ; float max_ang = 35.0 ; float zero_ang = 51.0 ;  // Right max : 65 -- Left max : 35 

void speedCb( const std_msgs::Float32& speed_msg){
  speed_command = speed_msg.data ; 
  
  // Reverting motor rotation 
  if((speed_command < 0) && (current_REL == HIGH)){
    current_REL = LOW ; 
    digitalWrite(pin_REL, LOW) ;  
  }else if((speed_command >= 0) && (current_REL == LOW)){
    current_REL = HIGH ; 
    digitalWrite(pin_REL, current_REL) ;   
  }

  // PWM for motor speed control 
  if(!is_paused){analogWrite(pin_PWM, int(abs(speed_command)*max_PWM));}
  else{analogWrite(pin_PWM, 0) ;}
  delay(10) ; 
}

void angleCb(const std_msgs::Float32& angle_msg){

  // Servomotor control for vehicle direction 
  angle_message = angle_msg.data ; 
  
  if(angle_message > 1){angle_message = 1;}
  else if(angle_message < -1){angle_message = -1 ;} 
  
  if(angle_message == 0){angle_scaled = zero_ang ;}
  else{angle_scaled = ((angle_message - min_msg)/(max_msg - min_msg)) * (max_ang - min_ang) + min_ang ;}
  
  servo.write(int(angle_scaled), speedServo) ; // To high speed = strong current drawing = Arduino Reboot 
  delay(10) ; 
}

void pauseCb(const std_msgs::Bool& pause_msg){
  // Pause message received
  is_paused = pause_msg.data ;
  if (is_paused){analogWrite(pin_PWM, 0) ;}
  delay(10) ; 
}

// ROS subscribers 
ros::Subscriber<std_msgs::Float32> sub_speed("/SpeedCommand", &speedCb );
ros::Subscriber<std_msgs::Float32> sub_angle("/AngleCommand", &angleCb );
ros::Subscriber<std_msgs::Bool> sub_stop("/PausePub", &pauseCb) ;

void setup(){
  
  // ROS initialisation 
  nh.initNode();
  nh.subscribe(sub_speed);
  nh.subscribe(sub_angle);
  nh.subscribe(sub_stop) ;
  Serial.begin(9600); // Test : How does the baudrate affect the rosserial connexion ? 

  // Servomotor initialisation 
  servo.attach(pin_SRV) ;
  servo.write(50) ;

  // Motor initialisation 
  pinMode(pin_PWM, OUTPUT) ; 
  pinMode(pin_REL, OUTPUT) ; 
  digitalWrite(pin_REL, current_REL) ;
  analogWrite(pin_PWM, 0) ; 
  
  delay(10) ;   
}

void loop(){
  nh.spinOnce();
  delay(1);  
}
