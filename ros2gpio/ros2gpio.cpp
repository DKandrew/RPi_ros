#include <ros/ros.h>
#include <stdio.h>
#include "wiringPi.h"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "ros2gpio");

	ros::NodeHandle nh;
	wiringPiSetup();
		  
	pinMode(7, OUTPUT);

	/*
	pinMode(11, INPUT);
	
	int sw = digitalRead(40);
	*/
	int i = 0;
	while(i <= 5){
		digitalWrite(7, HIGH) ; delay(500);
		digitalWrite(7, LOW)  ; delay(500);
	/*
	  cout << "Waiting for input" << endl;
	  delay(1000);
	  sw = digitalRead(40);
	  cout << "current sw is " << sw << endl;
	*/
	  i++;
	}
	
	cout << "LED is processed " << endl;

}