#include <ros/ros.h>
#include <iostream>
#include "pca9685.h"
#include "py_algorithm.h"
#include "wiringPi.h"
#include <wiringPiSPI.h>
#include <stdlib.h>
#include "pid.h"

//IMU
#include <fstream>
#include <sstream>
#include <string>

#define PI 3.14159265

using namespace std; 

//IMU Stuffs=============================================
std::string Convert (float number){
		std::ostringstream buff;
		buff<<number;
		return buff.str();
}

void notification_pip(int channel , int dataLen){
	uint8_t data[dataLen];
	data[0]=0x05;
	wiringPiSPIDataRW(channel, data, dataLen); 
}

void reset_imu(int channel){
	int dataLen = 7;
	unsigned char data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x40, 0x00, 0xc1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

void config_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'30'xbus-mid,'00'datalength,'D1'checksum
	//expected response: FA FF FF FF + '31'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xD1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

void SetOutputConfiguration(int channel){
	//int dataLen = 11;
	//int dataLen = 15;
	
	uint8_t data6[15] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x08, 0x40, 0x40,
							 0x03, 0xE8, 0x80, 0x40,
							 0x03, 0xE8, 0x23};
		uint8_t data8[15] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x08, 0x80, 0x20,
							 0x00, 0x64, 0x40, 0x20,
							 0x00, 0x64, 0x71};
		uint8_t data9[15] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x08, 0x40, 0x40,
							 0x00, 0x64, 0x80, 0x20,
							 0x03, 0xE8, 0xCA};
	
	
	
	uint8_t data5[19] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x0c, 0x20, 0x30,
							 0x00, 0x64, 0x40, 0x20,
							 0x00, 0x64, 0x80, 0x20,
							 0x00, 0x64, 0xb9};
	
	
	 uint8_t data4[19] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x0c, 0x20, 0x30,
							 0x00, 0x64, 0x40, 0x40,
							 0x03, 0xE8, 0x80, 0x40,
							 0x03, 0xE8, 0x6B};
	
	
	unsigned char data1[11] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x40, 0x40,
							 0xFF, 0xFF, 0xBF};
	     unsigned char data2[11] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x40, 0x20,
							 0x00, 0x64, 0x79};
	unsigned char data7[11] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x80, 0x40,
							 0x03, 0xE8, 0x92};	
     unsigned char data3[11] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x40, 0x40,
							 0x03, 0xE8, 0xD2};
	 int dataLen = 15;
	wiringPiSPIDataRW(channel, data8, dataLen);
}

void measurement_imu(int channel)
{
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'10'xbus-mid,'00'datalength,'F1'checksum
	//expected response: FA FF FF FF + '11'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0xF1};
	wiringPiSPIDataRW(channel, data, dataLen); 
}

float* readMeasurement_imu(int channel){
	int dataLen = 51;
	//int dataLen = 21;
	unsigned char data[dataLen] = {
								0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
								};
	wiringPiSPIDataRW(channel, data, dataLen);
	
	long roll_angle = ((long)data[9]<<24) | ((long)data[10]<<16) | ((long)data[11]<<8) | (long)data[12];
	long pitch_angle  = ((long)data[13]<<24) | ((long)data[14]<<16) | ((long)data[15]<<8) | (long)data[16];  
	long yaw_angle = ((long)data[17]<<24) | ((long)data[18]<<16) | ((long)data[19]<<8) | (long)data[20];
	
	long acc_x = ((long)data[24]<<24) | ((long)data[25]<<16) | ((long)data[26]<<8) | (long)data[27];
	long acc_y = ((long)data[28]<<24) | ((long)data[29]<<16) | ((long)data[30]<<8) | (long)data[31];
	long acc_z = ((long)data[32]<<24) | ((long)data[33]<<16) | ((long)data[34]<<8) | (long)data[35];
	
	long w1 = ((long)data[39]<<24) | ((long)data[40]<<16) | ((long)data[41]<<8) | (long)data[42];
	long w2 = ((long)data[43]<<24) | ((long)data[44]<<16) | ((long)data[45]<<8) | (long)data[46];
	long w3 = ((long)data[47]<<24) | ((long)data[48]<<16) | ((long)data[49]<<8) | (long)data[50];
	
	float x =  *((float*)&roll_angle);
	float y =  *((float*)&pitch_angle);
	float z =  *((float*)&yaw_angle);
	
	float ax = *((float*)&acc_x);
	float ay = *((float*)&acc_y);
	float az = *((float*)&acc_z);
	float wx = *((float*)&w1);
	float wy = *((float*)&w2);
	float wz = *((float*)&w3);
	
	float result[9] = {x,y,z,ax,ay,az,wx,wy,wz};

	return result;
}
//=======================================================


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
	int sampleRate = 100;  //it was 1000 before added IMU //Debug :  initially 90
	ros::Rate rate(sampleRate);
	//Setup wiringPi SPI
	int channel = 0;
	if(wiringPiSPISetup(channel,1000000) == -1)
	{
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup I2C
	int addr = 0x40;
	pca9685 pwm = pca9685(addr);
	int pos1, pos2, pos3;
	//Servo position vector
	int servo_num = 12;
	vector<int> pos (servo_num);
	//Set PWM frequency
	pwm.set_pwm_freq(100);
	//Setup GPIO
	wiringPiSetup();
	int pinNumber = 27;
	pinMode(pinNumber, OUTPUT);
	int LED_ON = 0;
	
	//IMU Stuffs=================
	//int sampleRate=100;
    int readlength=20000;
	//ros::Rate rate(sampleRate);
	//Setup wiringPi SPI for IMU 
	  //	SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
	  //	SPI_MODE1 = 1,  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
	  //	SPI_MODE2 = 2,  // CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
	  //	SPI_MODE3 = 3,  // CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	//int channel = 0; // Pi has 2 channels: 0 and 1
	int spi_freq = 1000000;
	int spi_mode = 3; // IMU needs mode3. // Encoder works for all the modes. (in mode 0, it reads different value.)
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup IMU
	int channel_imu = 0;
	//reset_imu(channel_imu);
	//config_imu(channel_imu);
	//SetOutputConfiguration(channel_imu);
	//measurement_imu(channel_imu);
	//Open channel 1 for encoder
	channel = 1;
	// spi_mode = 3;
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Main
	float* result_imu; 
	
	 for(int i=0;i<10000000;i++);
	config_imu(channel_imu);
	  for(int i=0;i<10000000;i++);
	SetOutputConfiguration(channel_imu);   //SetOutputConfig is the bug,with it sending 1000hz data type, imu blows up but save the config. at least. with 100hz it's still fine tho.
	 for(int i=0;i<100000000;i++);
	reset_imu(channel_imu);
	  for(int i=0;i<10000000;i++);
	measurement_imu(channel_imu);
	  for(int i=0;i<10000000;i++);


	std::string AngVel1[readlength];
	std::string Acc[readlength];
	std::string AngVel2[readlength];
	std::string AngVel3[readlength];
	int i=0;
	
	//End of IMU============================
	
	
	//Main
	double c_x,c_y,r,theta;
	theta = 0;
	//cout << "position of center x: "; cin >> c_x;
	//cout << "position of center y: "; cin >> c_y;
	//cout << "Radius: "; cin >> r;
	
	double x,y;
	int up = 0;	//I added
	double init_x = -11;	//I added
	double init_y = -15;	//I added
	x = init_x;	//I added
	y = init_y;	//I added
	double increment = 0.02;
	double range_low = -15;
	double range_high = -8.5;
	
	double* temp_result;
	int* result;
	int option;// = -1;
	double angle_knee;// = 154;
	double angle_hip;// = 20;	
	double angle_shoulder;// = 30;
	
	//pid control initialization
	double dt = 0.01;
	double max = 10;
	double min = -10; 
	double Kp = 1;
	double Kd = 1;
	double Ki = 0;
	
	pid PID = pid(dt, max, min, Kp, Kd, Ki);
	
	double angle_front = 0;
	double angle_back = 0;	

	while(ros::ok())
	{	
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
		
		
		//My New Code=======================================================================================
		while(option != 0 || option != 1 || option != 2 || option != 3 || option != 4 || option != 5)
		{
			cout<<"================"<<endl;		
			cout<<"Choose an Option"<<endl;
			cout<<"0: Debugging mode"<<endl;
			cout<<"1: Linear Motion"<<endl;
			cout<<"2: Move Forward"<<endl;
			cout<<"3: Flying Mode"<<endl;
			cout<<"4: Angle Control"<<endl;
			cout<<"5: Angle Control"<<endl;
			cout<<"================"<<endl;
			cout<<"Option: "; cin >> option;
			cout<<endl;
			option = (int)option;
			cout<<"Choosen Option: "<<option<<endl;
			if(option == 0 || option == 1 || option == 2 || option == 3 || option == 4 || option == 5)
				break;
			else
				cout<<"Please Select from 0, 1, 2, 3 4 and 5"<<endl;				 
		}		
		
		
		//Option0: Debugging Mode		
		if(option == 0)
		{
			cout<<"<Debugging Mode Selected>"<<endl;
			angle_knee = 180;
			angle_hip = 90;	
			angle_shoulder = 0;		
			temp_result = angle_temp_correction(angle_knee*(PI/180), angle_hip*(PI/180));
			result = angle2PWM(temp_result[0], temp_result[1], angle_shoulder*(PI/180));
			pos[0] = result[2];
			pos[1] = result[1]; //Port 1 connects to B servo 
			pos[2] = result[0]; //Port 2 connects to A servo
			I2C_ctrl(&pwm, pos);
		}
		
		//Option1: Linear Motion Code
		if(option == 1)
		{
			cout<<"<Linear Motion Mode>"<<endl;
			int limit_i = 2000;
			x = init_x;
			y = init_y;	
			for(int i = 0; i < limit_i; ++i)
			{
				if(y < range_low)
					up = 1;		//Go Up
				if(y > range_high)
					up = 0;		//Go Down
					
				if(up)
					y += increment;
				else
					y -= increment;		

				temp_result = inverse_kinematics(x, y);							
				//cout << "q1: " << (180/PI)*temp_result[0] << " q2: " << (180/PI)*temp_result[1] << endl;
				temp_result = angle_temp_correction(temp_result[0], temp_result[1]);
				cout << "Corrected q1: " << temp_result[0] << " Corrected q2: " << temp_result[1] << endl;
				
				result = angle2PWM(temp_result[0], temp_result[1], 0*(PI/180));
				pos[0] = result[2];
				pos[1] = result[1]; //Port 1 connects to B servo 
				pos[2] = result[0]; //Port 2 connects to A servo		
				
				I2C_ctrl(&pwm, pos);
				//ros::Duration(0.001).sleep();
				
				if(y < -15 || y > -8.5)
					ros::Duration(1).sleep();
				cout<<"i: "<<i<<"/"<<limit_i<<endl;
			}
		}
		
		//Option2: Walking Motion Code
		if(option == 2)
		{
			cout<<"<Walking Mode>"<<endl;
			x = -11;
			y = -8.5;
			double angle_shoulder_back = -30;
			double angle_shoulder_front = 30;
			angle_shoulder = angle_shoulder_back;
			int direction = 1;
			int limit_j = 800;
			double shoulder_inc = (range_high - range_low)/increment;
			double shoulder_interval = (angle_shoulder_front - angle_shoulder_back)/(2*shoulder_inc);
			
			for(int j = 0; j < limit_j;++j)
			{
				//Leg Height Condition
				if(y < range_low)
					up = 1;		//Go Up
				if(y > range_high)
					up = 0;		//Go Down				
				//Leg Height Control		
				if(up)
					y += increment;
				else
					y -= increment;
					
				//Shoulder Angle Condition
				if(angle_shoulder > angle_shoulder_front)
					direction = 0;	//Go back
				if(angle_shoulder < angle_shoulder_back)
					direction = 1;	//Go Front
				//Shoulder Angle Control	
				if(direction == 1)
					angle_shoulder += shoulder_interval;
				else
					angle_shoulder -= shoulder_interval;
				
				//Calculate Angles for Servo Motors	
				temp_result = inverse_kinematics(x, y);
				temp_result = angle_temp_correction(temp_result[0], temp_result[1]);						
				result = angle2PWM(temp_result[0], temp_result[1], angle_shoulder*(PI/180));
				
				
				pos[0] = result[2];
				pos[1] = result[1]; //Port 1 connects to B servo 
				pos[2] = result[0]; //Port 2 connects to A servo		
				
				I2C_ctrl(&pwm, pos);
				//ros::Duration(0.2).sleep();
				
				if(y < -15 || y > -8.5)
					ros::Duration(0.5).sleep();
			}
			
		}
		
		//Option3: Flying Mode //x = -21.4686; y = 3.96442;
		if(option == 3)
		{	
			cout<<"<Flying Mode>"<<endl;
			angle_knee = 155;
			angle_hip = 19;
			int limit_k = 100;
			int change = 0;
			for(int k = 0; k < limit_k; ++k)
			{
				if(change == 0)
				{
					angle_shoulder = 0;
					change = 1;
				}
				else if(change == 1)
				{
					angle_shoulder = 0;
					change = 0;
				}
				cout<<"k: "<<k<<"/"<<limit_k<<endl;
			}
				
			temp_result = angle_temp_correction(angle_knee*(PI/180), angle_hip*(PI/180));
			result = angle2PWM(temp_result[0], temp_result[1], angle_shoulder*(PI/180));
			pos[0] = result[2];
			pos[1] = result[1]; //Port 1 connects to B servo 
			pos[2] = result[0]; //Port 2 connects to A servo
			I2C_ctrl(&pwm, pos);
			//ros::Duration(10).sleep();			
		}
		
		//Option4: Angle Control 
		if(option == 4)
		{
			//cout<<"<Angle Control Mode Selected>"<<endl;
			//cout<<"Enter Knee Angle in Degree: ";
			//cin >> angle_knee;
			//cout<<endl;
			//cout<<"Enter Hip Angle in Degree: ";
			//cin >> angle_hip;
			//cout<<endl;
			cout<<"Enter Shoulder Angle in Degree: ";
			cin >> angle_shoulder;
			cout<<endl;
			
			angle_knee = double(angle_knee);
			angle_hip = double(angle_hip);	
			angle_shoulder = double(angle_shoulder);
			
			angle_knee = 155;
			angle_hip = 19;
					
			temp_result = angle_temp_correction(angle_knee*(PI/180), angle_hip*(PI/180));
			result = angle2PWM(temp_result[0], temp_result[1], angle_shoulder*(PI/180));
			pos[0] = result[2];
			pos[1] = result[1]; //Port 1 connects to B servo 
			pos[2] = result[0]; //Port 2 connects to A servo
			I2C_ctrl(&pwm, pos);
		}		
		
		if(option == 5)
		{
			//IMU
			result_imu = readMeasurement_imu(channel_imu);
			float roll, pitch, yah, wx, wy, wz;
			roll = result_imu[0]; 
			pitch = result_imu[1]; 
			yah = result_imu[2]; 
			wx = result_imu[3];
			wy = result_imu[4];
			wz = result_imu[5];
			Acc[i]=Convert(yah);
			AngVel1[i]=Convert(wx);
			AngVel2[i]=Convert(wy);
			AngVel3[i++]=Convert(wz);
			printf("w1: %f w2: %f w3: %f \n", wx, wy, wz);
			printf("Roll: %f Pitch: %f Yah: %f \n", roll, pitch, yah);
			//IMU
			
			//Servo Control
			double setpoint = 0;
			double inc = 0;
			inc = PID.calc(setpoint, pitch);
			angle_front += inc;
			angle_back -+ inc;		
			
			temp_result = angle_temp_correction(angle_front*(PI/180), angle_back*(PI/180));
			result = angle2PWM(temp_result[0], temp_result[1], 0);
			pos[0] = result[0];
			pos[1] = result[0]; //Port 1 connects to B servo 
			pos[2] = result[1]; //Port 2 connects to A servo
			pos[3] = result[1];
			I2C_ctrl(&pwm, pos);
			
			option = 5;	
		}
		
		//End of My New Code===================================================================
		
		
		/*
		//Code to test the Raspberrypi Zero runtime  --By Qinru
		cout<<"<Walking Mode>"<<endl;
		x = -11;
		y = -8.5;
		double angle_shoulder_back = -30;
		double angle_shoulder_front = 30;
		angle_shoulder = angle_shoulder_back;
		int direction = 1;
		int limit_j = 20;
		double shoulder_inc = (range_high - range_low)/increment;
		double shoulder_interval = (angle_shoulder_front - angle_shoulder_back)/(2*shoulder_inc);
		
		for(int j = 0; j < limit_j;++j)
		{
			//Leg Height Condition
			if(y < range_low)
				up = 1;		//Go Up
			if(y > range_high)
				up = 0;		//Go Down				
			//Leg Height Control		
			if(up)
				y += increment;
			else
				y -= increment;
				
			//Shoulder Angle Condition
			if(angle_shoulder > angle_shoulder_front)
				direction = 0;	//Go back
			if(angle_shoulder < angle_shoulder_back)
				direction = 1;	//Go Front
			//Shoulder Angle Control	
			if(direction == 1)
				angle_shoulder += shoulder_interval;
			else
				angle_shoulder -= shoulder_interval;
			
			//Calculate Angles for Servo Motors	
			temp_result = inverse_kinematics(x, y);
			temp_result = angle_temp_correction(temp_result[0], temp_result[1]);						
			result = angle2PWM(temp_result[0], temp_result[1], angle_shoulder*(PI/180));
			
			
			//pos[0] = result[2];
			//pos[1] = result[1]; //Port 1 connects to B servo 
			//pos[2] = result[0]; //Port 2 connects to A servo		
			
			
			
			//if(y < -15 || y > -8.5)
				//ros::Duration(0.5).sleep();
		}
		pos[0] = result[2];
		pos[1] = result[1]; 
		pos[2] = result[0]; 	
		pos[3] = result[2];
		pos[4] = result[1]; 
		pos[5] = result[0]; 
		pos[6] = result[2];
		pos[7] = result[1]; 
		pos[8] = result[0]; 
		pos[9] = result[2];
		pos[10] = result[1]; 
		pos[11] = result[0]; 
		I2C_ctrl(&pwm, pos);
		//End of testing Raspberry Pi zero===============
		*/
		
		// GPIO control
		if(LED_ON == 0){
			LED_ON = 1;
		}
		else{
			LED_ON = 0;
		}
		digitalWrite(pinNumber, LED_ON);
		//Wait
		rate.sleep();
		
	}
}

//cout commands
//cout << "q5: " << temp_result[0] << " alpha: " << temp_result[1] << " beta: "<<temp_result[2] << " gamma: "<<temp_result[3] << " q1: "<<temp_result[4] << endl;
//cout << "psi: " << temp_result[5] << " xi: " << temp_result[6] << " a: "<<temp_result[7] << " b: "<<temp_result[8] << " delta1: "<<temp_result[9] << endl;
//cout << "delta2: " << temp_result[10] << " q2: " << temp_result[11] << " phi0: "<<temp_result[12] << " q4: "<<temp_result[13] << " q3: "<<temp_result[14] << endl;

//double* forward = forward_kinematics(temp_result[0], temp_result[1]);
//double* forward = forward_kinematics((PI/180)*155.2, (PI/180)*27);
//cout << "P5x: " << forward[0] << " P5y: " << forward[1] << " q3: " << forward[2]<<" q4: " << forward[3] << " q5: " << forward[4]<<endl;
				
