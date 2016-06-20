#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include "wiringPi.h"

using namespace std;

int LED_ON;

//Callback function
void messageHandler(const geometry_msgs::Twist &msg){
	double one = 1.0;
	//double zero = 0.0;
	if(msg.linear.x == one){
		LED_ON = 1;
	}else{
		LED_ON = 0;
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
	ros::Subscriber sub = nh.subscribe("led/cmd_vel", 1000, &messageHandler);
	while(ros::ok()){
		ros::spinOnce();
		if(LED_ON == 1)
			digitalWrite(7, HIGH);
		else if (LED_ON == 0)
			digitalWrite(7, LOW);
		cout << "Received and processed msg" << endl;
	}
}
