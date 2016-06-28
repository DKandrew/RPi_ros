#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <wiringPiSPI.h>
#include <ros/ros.h>
#include <a2d_spi/a2d.h> //include a2d.msg file

using namespace std;

void printdata(uint8_t * arr){
	for(int i=0; i<3; i++){
		cout << unsigned(arr[i]) << " ";
	}
	cout << endl;
}

uint16_t readVoltage(uint8_t * arr) {
	//Read the second bit and the third bit 
	uint8_t second = arr[1] & 0x03;
	uint8_t third = arr[2];
	uint16_t sum = 0x0000;
	sum = sum + second;
	sum = (sum << 8) + third;
	return sum;
}

int main(int argc, char**argv){
	//Initialize ROS system
	ros::init(argc, argv, "spi_ctrl");
	ros::NodeHandle nh;
	//Create a publisher object
	ros::Publisher pub = nh.advertise<a2d_spi::a2d>("a2d/control", 1000);
	//Frequency Control
	ros::Rate rate(8);
	//Setup wiringPi SPI
	int channel = 0;
	if(wiringPiSPISetup(channel,1000000) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Main
	int loop_cnt = 0;
	while(ros::ok()){
		uint8_t a = 0x01;
		uint8_t b = 0x90;
		uint8_t data[3] = {a,b,0};
		wiringPiSPIDataRW(channel, data, 3);
		//Create voltage message
		a2d_spi::a2d msg;
		msg.volt = readVoltage(data);
		//Publish the message.
		pub.publish(msg);
		//Send a message to rosout with details
		ROS_INFO_STREAM("A/D voltage message sent!");
		//Wait
		rate.sleep();
	}
}



