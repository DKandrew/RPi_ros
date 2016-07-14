#include <ros/ros.h>
#include <iostream>
#include "pca9685.h"
#include "py_algorithm.h"
#include "wiringPi.h"
#include <wiringPiSPI.h>
#include <stdlib.h>

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
	int sampleRate;
	cout << "Sample Rate is ? " << endl;
	cin >> sampleRate;
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
	srand(0);
	//Servo position vector
	int servo_num = 12;
	vector<int> pos (servo_num);
	//Set frequency to 60hz, good for servos
	pwm.set_pwm_freq(60);
	//Setup GPIO
	wiringPiSetup();
	pinMode(7, OUTPUT);
	int LED_ON = 0;
	//Main
	while(ros::ok()){
		//Read from SPI
		int channel = 1;
		uint16_t volt = SPI_ctrl(channel);
		//Computer algorithm 5 times
		for(int i=0; i<10; i++){
			double array[3] = {1,2,3};
			double result[3] = {0};
			inverse_kinematics(array, result);
		}
		// I2C Control 
		/* 
		pos[0] = rand()%450 +150;		//Generate random number
		pos[1] = rand()%450 +150;
		pos[2] = rand()%450 +150;
		pos[3] = rand()%450 +150;		
		pos[4] = rand()%450 +150;
		pos[5] = rand()%450 +150;
		pos[6] = rand()%450 +150;		
		pos[7] = rand()%450 +150;
		pos[8] = rand()%450 +150;
		pos[9] = rand()%450 +150;		
		pos[10] = rand()%450 +150;
		pos[11] = rand()%450 +150;
		*/
		pos[0] = 200;
		pos[1] = 300;
		pos[2] = 400;
		pos[3] = 500;	
		pos[4] = 600;
		pos[5] = 700;
		pos[6] = 800;	
		pos[7] = 900;
		pos[8] = 1000;
		pos[9] = 1100;	
		pos[10] = 1200;
		pos[11] = 1300;
		
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

