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
	double c_x,c_y,r,theta;
	theta = 0;
	//cout << "position of center x: "; cin >> c_x;
	//cout << "position of center y: "; cin >> c_y;
	//cout << "Radius: "; cin >> r;
	double x,y;
	int up = 0;	//I added
	x = -8;	//I added
	y = -5;	//I added
	double increment = 0.5;

	while(ros::ok()){
		//Read from SPI
		int channel = 1;
		uint16_t volt = SPI_ctrl(channel);
		
		//Original Working Code
		
		//------------------------------------------------------------------
		//Computer algorithm
		/*
		x=c_x; y=c_y+r*sin(theta); 
		theta += 10*PI/180;
		double* temp_result=inverse_kinematics(x,y);
		int* result = angle2PWM(temp_result[0], temp_result[1]);
		cout << "x: " << x << " y: " << y << " theta: "<<theta<< endl;
		// I2C Control 
		pos[0] =720;
		pos[1] = result[1]; //Port 1 connects to B servo 
		pos[2] = result[0]; //Port 2 connects to A servo 
		
		I2C_ctrl(&pwm, pos);
		//-------------------------------------------------------------------
		*/
		//End of Original working code
		
		
		//My New Code===============================
		
		if(y < -19)
			up = 1;		//Go Up
		if(y > -10)
			up = 0;		//Go Down
			
		if(up)
			y += increment;
		else
			y -= increment;
		
		
		//x = -8; y = -10;
		double* temp_result = inverse_kinematics(x, y);
		//cout << "Rsult================= " << endl;
		//cout << "q5: " << temp_result[0] << " alpha: " << temp_result[1] << " beta: "<<temp_result[2] << " gamma: "<<temp_result[3] << " q1: "<<temp_result[4] << endl;
		//cout << "psi: " << temp_result[5] << " xi: " << temp_result[6] << " a: "<<temp_result[7] << " b: "<<temp_result[8] << " delta1: "<<temp_result[9] << endl;
		//cout << "delta2: " << temp_result[10] << " q2: " << temp_result[11] << " phi0: "<<temp_result[12] << " q4: "<<temp_result[13] << " q3: "<<temp_result[14] << endl;
		cout << "q1: " << temp_result[0] << " q2: " << temp_result[1] << endl;
		temp_result = angle_temp_correction(temp_result[0], temp_result[1]);
		//cout << "Corrected q1: " << temp_result[0] << " Corrected q2: " << temp_result[1] << endl;
		//double* forward = forward_kinematics(temp_result[0], temp_result[1]);
		//cout << "P5x: " << forward[0] << " P5y: " << forward[1] << endl;
		
		
		int* result = angle2PWM(temp_result[0], temp_result[1]);
		pos[0] =720;
		pos[1] = result[1]; //Port 1 connects to B servo 
		pos[2] = result[0]; //Port 2 connects to A servo		
		
		I2C_ctrl(&pwm, pos);
		if(y < -19)
			ros::Duration(1).sleep();
		
	
		
		//End of My New Code=======================
		
		
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

