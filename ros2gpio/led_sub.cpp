#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <ros2gpio/led.h>  //include led message 
#include "wiringPi.h"

using namespace std;

int LED1_ON;
int LED2_ON;
int LED3_ON;
int LED4_ON;

//Callback function
void messageHandler(const ros2gpio::led &msg){
	//double one = 1.0;
	//double zero = 0.0;
	switch(msg.LED1) {
		case 1:
			LED1_ON = 1;
			break;
		default:
			LED1_ON = 0;
	}
	switch(msg.LED2) {
		case 1:
			LED2_ON = 1;
			break;
		default:
			LED2_ON = 0;
	}
	switch(msg.LED3) {
		case 1:
			LED3_ON = 1;
			break;
		default:
			LED3_ON = 0;
	}
	switch(msg.LED4) {
		case 1:
			LED4_ON = 1;
			break;
		default:
			LED4_ON = 0;
	}
}

int main(int argc, char **argv){
	//Initialize the ROS system
	ros::init(argc, argv, "led_sub");
	ros::NodeHandle nh;
	//set up wiringPi
	wiringPiSetup();
	pinMode(7, OUTPUT);
	pinMode(0, OUTPUT);
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	//Create a subscriber
	ros::Subscriber sub = nh.subscribe("led/control", 1000, &messageHandler);
	while(ros::ok()){
		ros::spinOnce();
		digitalWrite(7, LED1_ON);
		digitalWrite(0, LED2_ON);
		digitalWrite(2, LED3_ON);
		digitalWrite(3, LED4_ON);
		//cout << "Received and processed msg" << endl;
	}
}
