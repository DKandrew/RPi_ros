#include <ros/ros.h>
#include <iostream>
#include "pca9685.h"
#include "py_algorithm.h"
#include "wiringPi.h"
#include <wiringPiSPI.h>
#include <stdlib.h>

#define PI 3.14159265

using namespace std; 

// Helper function: convert data into voltage
// Input: array to be converted
// Output: the voltage 
uint16_t readVoltage(uint8_t * arr) {
	//Read the second bit and the third bit 
	uint8_t second = arr[1] & 0x03;
	uint8_t third = arr[2];
	uint16_t sum = 0x0000;
	sum = sum + second;
	sum = (sum << 8) + third;
	return sum;
}

// Using SPI to listen the output of the A/D converter. 
// Input: channel to read
// Output: Voltage from A/D
uint16_t SPI_ctrl(int channel){
	uint8_t a = 0x01;
	uint8_t b = 0x90;
	uint8_t data[3] = {a,b,0};
	wiringPiSPIDataRW(channel, data, 3);
	return readVoltage(data);	
}

//I2C control to three servos
void I2C_ctrl(pca9685 *pwm, vector<int> & pos){
	pwm->set_pwm(pos);
}

int main(int argc, char**argv){
	//Initialize ROS system
	ros::init(argc, argv, "main_Control");
	ros::NodeHandle nh;
	//Frequency Control
	int sampleRate = 90;
	ros::Rate rate(sampleRate);
	//Setup wiringPi SPI
	int channel = 0;
	if(wiringPiSPISetup(channel,1000000) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup I2C
	int addr = 0x40;
	pca9685 pwm = pca9685(addr);
	int pos1, pos2, pos3;
	//Servo position vector
	int servo_num = 3;
	vector<int> pos (servo_num);
	//Set PWM frequency
	pwm.set_pwm_freq(100);
	//Setup GPIO
	wiringPiSetup();
	pinMode(7, OUTPUT);
	int LED_ON = 0;
	//Main
	//int input0, input1;
	//cout << "position of input0: "; cin >> input0;
	double c_x,c_y,r,theta;
	theta = 0;
	cout << "position of center x: "; cin >> c_x;
	cout << "position of center y: "; cin >> c_y;
	cout << "Radius: "; cin >> r;
	double x,y;

	while(ros::ok()){
		//Read from SPI
		int channel = 1;
		uint16_t volt = SPI_ctrl(channel);
		//Computer algorithm
		x=c_x; y=c_y+r*sin(theta); 
		theta += PI/180;
		double* temp_result=inverse_kinematics(x,y);
		int* result = angle2PWM(temp_result[0], temp_result[1]);
		cout << "x: " << x << " y: " << y << endl;
		// I2C Control 
		pos[0] = 720;
		pos[1] = result[1];
		pos[2] = result[0];
		
		
		I2C_ctrl(&pwm, pos);
		// GPIO control
		if(LED_ON == 0){
			LED_ON = 1;
		}
		else{
			LED_ON = 0;
		}
		digitalWrite(7, LED_ON);
		//Wait
		rate.sleep();
		
	}
}

