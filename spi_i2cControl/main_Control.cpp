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

void notification_pip(int channel){
	int dataLen = 7;
	uint8_t data[dataLen] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
	int dataLen = 19;
	/*
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x20, 0x30,
							 0x00, 0x64, 0x89};
	*/
	/*
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x08, 0x40, 0x20,
							 0x00, 0x64, 0x80, 0x20,
							 0x00, 0x64, 0x71};
	*/
	
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x0c, 0x20, 0x30,
							 0x00, 0x64, 0x40, 0x20,
							 0x00, 0x64, 0x80, 0x20,
							 0x00, 0x64, 0xb9};
	
	wiringPiSPIDataRW(channel, data, dataLen);
}

void measurement_imu(int channel){
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

int readEncoder(int channel){
	int dataLen = 2;
	unsigned char data[dataLen] = {0xff, 0xff};
	wiringPiSPIDataRW(channel, data, dataLen);
	int EncReading = (((unsigned int) (data[0] & 0x7f)) << 6) + (((unsigned int) (data[1] & 0xf8)) >> 2);
	return EncReading;
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
    int readlength=20000;
	int spi_freq = 1000000;
	int spi_mode = 3; // IMU needs mode3. // Encoder works for all the modes. (in mode 0, it reads different value.)
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup IMU
	int channel_imu = 0;
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

	int i=0;
	//End of IMU============================
	
	
	//Main
	double x,y,z;
	int up = 0;	//I added
	double init_x = 0;
	double init_y = -11;	//I added
	double init_z = -15;	//I added
	x = init_x;
	y = init_y;	//I added
	z = init_z;	//I added
	double increment = 0.02;
	double range_low = -15;
	double range_high = -8.5;
	
	double* temp_result_RF;
	double* temp_result_LF;
	int* result;
	int option = -1;
	double angle_knee;// = 154;
	double angle_hip;// = 20;	
	double angle_shoulder;// = 30;
	
	//pid control initialization
	double dt = 1.0/(double)sampleRate;
	double max = 10;
	double min = -10; 
	double Kp = 0.005;
	double Kd = 0.01;
	double Ki = 0.0;
	
	pid PID_pitch = pid(dt, max, min, Kp, Kd, Ki);
	pid PID_roll = pid(dt, max, min, Kp, Kd, Ki);
	
	double angle_front = 0;
	double angle_back = 0;	
	double angle_left = 0;
	double angle_right = 0;
	double angle_LF = 0;
	double angle_RF = 0;
	double angle_LH = 0;
	double angle_RH = 0;

	while(ros::ok())
	{	
		//Read from SPI
		int channel = 1;
		uint16_t volt = SPI_ctrl(channel);	
		
		/*
		while(option != 0 || option != 1 || option != 2 || option != 3 || option != 4 || option != 5 || option != 6)
		{
			cout<<"================"<<endl;		
			cout<<"Choose an Option"<<endl;
			cout<<"0: Debugging mode"<<endl;
			cout<<"1: Linear Motion"<<endl;
			cout<<"2: Move Forward"<<endl;
			cout<<"3: Flying Mode"<<endl;
			cout<<"4: Angle Control"<<endl;
			cout<<"5: IMU Test"<<endl;
			cout<<"6: Trot Test"<<endl;
			cout<<"================"<<endl;
			cout<<"Option: "; cin >> option;
			cout<<endl;
			option = (int)option;
			cout<<"Choosen Option: "<<option<<endl;
			if(option == 0 || option == 1 || option == 2 || option == 3 || option == 4 || option == 5 || option == 6)
				break;
			else
				cout<<"Please Select from 0, 1, 2, 3, 4, 5 and 6"<<endl;				 
		}		
		*/
		option = 5;
		//Option0: Debugging Mode		
		if(option == 0)
		{
			cout<<"<Debugging Mode Selected>"<<endl;
			angle_knee = 180;
			angle_hip = 90;	
			angle_shoulder = 0;
			double* LF = new double[3];	
			temp_result_LF = angle_temp_correction_LF(0*(PI/180),90*(PI/180), 180*(PI/180));
			LF[0] = temp_result_LF[0];
			LF[1] = temp_result_LF[1];
			LF[2] = temp_result_LF[2];
			
			double* RF = new double[3];			
			temp_result_RF = angle_temp_correction_RF(angle_knee*(PI/180), angle_hip*(PI/180), angle_shoulder*(PI/180));
			RF[0] = temp_result_RF[0];
			RF[1] = temp_result_RF[1];
			RF[2] = temp_result_RF[2];
			
			int* LF_PWM = new int[3];
			result = angle2PWM(LF[0], LF[1], LF[2]);
			LF_PWM[0] = result[0];
			LF_PWM[1] = result[1];
			LF_PWM[2] = result[2];
				
			int* RF_PWM = new int[3];
			result = angle2PWM(RF[0], RF[1], RF[2]);
			RF_PWM[0] = result[0];
			RF_PWM[1] = result[1];
			RF_PWM[2] = result[2];
			
			cout<<"LF_PWM Knee: "<<LF_PWM[0]<<"  LF_PWM Hip: "<<LF_PWM[1]<<"  LF_PWM Shoulder: "<<LF_PWM[2]<<endl;
			cout<<"RF_PWM Knee: "<<RF_PWM[0]<<"  RF_PWM Hip: "<<RF_PWM[1]<<"  RF_PWM Shoulder: "<<RF_PWM[2]<<endl;
					
			
			//Write to Servo Motors			
			pos[0] = LF_PWM[0];
			pos[1] = LF_PWM[1];
			pos[2] = LF_PWM[2];
			pos[3] = RF_PWM[0];
			pos[4] = RF_PWM[1];
			pos[5] = RF_PWM[2];
			pos[6] = RF_PWM[0];
			pos[7] = RF_PWM[1];
			pos[8] = RF_PWM[2];
			pos[9] = LF_PWM[0];
			pos[10] = LF_PWM[1];
			pos[11] = LF_PWM[2];
			I2C_ctrl(&pwm, pos);				
		}
		
		//Option1: Linear Motion Code
		if(option == 1)
		{
			cout<<"<Linear Motion Mode>"<<endl;
			int limit_i = 2000;
			x = init_x;
			y = init_y;
			z = init_z;	
			for(int i = 0; i < limit_i; ++i)
			{
				if(z < range_low)
					up = 1;		//Go Up
				if(z > range_high)
					up = 0;		//Go Down
					
				if(up)
					z += increment;
				else
					z -= increment;		

				temp_result_RF = inverse_kinematics(x, y, z);							
				//cout << "q1: " << (180/PI)*temp_result[0] << " q2: " << (180/PI)*temp_result[1] << endl;
				temp_result_RF = angle_temp_correction_RF(temp_result_RF[0], temp_result_RF[1], temp_result_RF[2]);
				cout << "Corrected q1: " << temp_result_RF[0] << " Corrected q2: " << temp_result_RF[1] << endl;
				
				result = angle2PWM(temp_result_RF[0], temp_result_RF[1], temp_result_RF[2]);
				pos[0] = result[0];
				pos[1] = result[1]; //Port 1 connects to B servo 
				pos[2] = result[2]; //Port 2 connects to A servo		
				
				I2C_ctrl(&pwm, pos);
				//ros::Duration(0.001).sleep();
				
				if(z < -15 || z > -8.5)
					ros::Duration(1).sleep();
				cout<<"i: "<<i<<"/"<<limit_i<<endl;
			}
		}
		
		//Option2: Walking Motion Code
		if(option == 2)
		{
			cout<<"<Walking Mode>"<<endl;
			y = -11;
			z = -8.5;
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
				if(z < range_low)
					up = 1;		//Go Up
				if(z > range_high)
					up = 0;		//Go Down				
				//Leg Height Control		
				if(up)
					z += increment;
				else
					z -= increment;
					
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
				temp_result_RF = inverse_kinematics(x, y, z);
				temp_result_RF = angle_temp_correction_RF(temp_result_RF[0], temp_result_RF[1], temp_result_RF[2]);						
				result = angle2PWM(temp_result_RF[0], temp_result_RF[1], temp_result_RF[2]);
				
				
				pos[0] = result[0];
				pos[1] = result[1]; //Port 1 connects to B servo 
				pos[2] = result[2]; //Port 2 connects to A servo		
				
				I2C_ctrl(&pwm, pos);
				//ros::Duration(0.2).sleep();
				
				if(z < -15 || z > -8.5)
					ros::Duration(0.5).sleep();
			}			
		}
		
		//Option3: Flying Mode
		if(option == 3)
		{	
			double dihedral;
			cout<<"<Flying Mode>"<<endl;
			cout<<"Enter Desired Dihedral Angle: "; cin >> dihedral;
			dihedral = (double)dihedral;
			cout<<"Enter Desired Tension Angle: "; cin >> angle_shoulder;
			angle_shoulder = (double)angle_shoulder;
			temp_result_LF = glide_stretch(dihedral, angle_shoulder, 170);
			cout<<"Knee: "<<(180/PI)*temp_result_LF[0]<<"  Hip: "<<(180/PI)*temp_result_LF[1]<<"  Shoulder: "<<(180/PI)*temp_result_LF[2]<<endl;
			double* LF = new double[3];			
			LF[0] = PI - temp_result_LF[0];
			LF[1] = PI - temp_result_LF[1];
			LF[2] = PI - temp_result_LF[2];

			temp_result_RF = glide_stretch(dihedral, angle_shoulder, 170);
			cout<<"Knee: "<<(180/PI)*temp_result_RF[0]<<"  Hip: "<<(180/PI)*temp_result_RF[1]<<"  Shoulder: "<<(180/PI)*temp_result_RF[2]<<endl;			
			double* RF = new double[3];
			RF[0] = temp_result_RF[0];
			RF[1] = temp_result_RF[1];
			RF[2] = temp_result_RF[2];

			double* LH = new double[3]; 
			LH[0] = RF[0];
			LH[1] = RF[1];
			LH[2] = -RF[2];
					
			double* RH = new double[3]; 
			RH[0] = LF[0];
			RH[1] = LF[1];
			RH[2] = -LF[2] + 2*PI;
						
			double* angle_corrected;
			angle_corrected = angle_temp_correction_LF(LF[0], LF[1], LF[2]);
			LF[0] = angle_corrected[0];
			LF[1] = angle_corrected[1];
			LF[2] = angle_corrected[2];
					
			angle_corrected = angle_temp_correction_RF(RF[0], RF[1], RF[2]);
			RF[0] = angle_corrected[0];
			RF[1] = angle_corrected[1];
			RF[2] = angle_corrected[2];
					
			angle_corrected = angle_temp_correction_RF(LH[0], LH[1], LH[2]);
			LH[0] = angle_corrected[0];
			LH[1] = angle_corrected[1];
			LH[2] = angle_corrected[2];
					
			angle_corrected = angle_temp_correction_LF(RH[0], RH[1], RH[2]);	
			RH[0] = angle_corrected[0];
			RH[1] = angle_corrected[1];
			RH[2] = angle_corrected[2];					
					
			//Convert Angle to PWM
			int* PWM;
			int LF_PWM[3];
			int RF_PWM[3];
			int LH_PWM[3];
			int RH_PWM[3];

			PWM = angle2PWM(LF[0], LF[1], LF[2]);
			LF_PWM[0] = PWM[0];
			LF_PWM[1] = PWM[1];
			LF_PWM[2] = PWM[2];
										
			PWM = angle2PWM(RF[0], RF[1], RF[2]);
			RF_PWM[0] = PWM[0];
			RF_PWM[1] = PWM[1];
			RF_PWM[2] = PWM[2];
					
			PWM = angle2PWM(LH[0], LH[1], LH[2]);
			LH_PWM[0] = PWM[0];
			LH_PWM[1] = PWM[1];
			LH_PWM[2] = PWM[2];
				
			PWM = angle2PWM(RH[0], RH[1], RH[2]);
			RH_PWM[0] = PWM[0];
			RH_PWM[1] = PWM[1];
			RH_PWM[2] = PWM[2];				
					
			cout<<"LF_PWM Knee: "<<LF_PWM[0]<<"  LF_PWM Hip: "<<LF_PWM[1]<<"  LF_PWM Shoulder: "<<LF_PWM[2]<<endl;
			cout<<"RF_PWM Knee: "<<RF_PWM[0]<<"  RF_PWM Hip: "<<RF_PWM[1]<<"  RF_PWM Shoulder: "<<RF_PWM[2]<<endl;
			//cout<<"LH_PWM Knee: "<<LH_PWM[0]<<"  LH_PWM Hip: "<<LH_PWM[1]<<"  LH_PWM Shoulder: "<<LH_PWM[2]<<endl;
			//cout<<"RH_PWM Knee: "<<RH_PWM[0]<<"  RH_PWM Hip: "<<RH_PWM[1]<<"  RH_PWM Shoulder: "<<RH_PWM[2]<<endl;		
					
			//Write to Servo Motors			
			pos[0] = LF_PWM[0];
			pos[1] = LF_PWM[1];
			pos[2] = LF_PWM[2];
			pos[3] = RF_PWM[0];
			pos[4] = RF_PWM[1];
			pos[5] = RF_PWM[2];
			pos[6] = LH_PWM[0];
			pos[7] = LH_PWM[1];
			pos[8] = LH_PWM[2];
			pos[9] = RH_PWM[0];
			pos[10] = RH_PWM[1];
			pos[11] = RH_PWM[2];
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
			angle_shoulder = 0;
					
			temp_result_RF = angle_temp_correction_RF(angle_knee*(PI/180), angle_hip*(PI/180), angle_shoulder*(PI/180));
			result = angle2PWM(temp_result_RF[0], temp_result_RF[1], temp_result_RF[2]);
			pos[0] = result[0];
			pos[1] = result[1]; //Port 1 connects to B servo 
			pos[2] = result[2]; //Port 2 connects to A servo
			I2C_ctrl(&pwm, pos);
		}		
		
		//Option5: IMU Test
		if(option == 5)
		{
			//IMU
			result_imu = readMeasurement_imu(channel_imu);
			float roll, pitch, yah;
			roll = result_imu[0]; 
			pitch = result_imu[1]; 
			yah = result_imu[2]; 
			printf("Roll: %f Pitch: %f Yah: %f \n", roll, pitch, yah);
			
			//Servo Control
			double setpoint = 0;
			double inc_pitch = 0;
			double inc_roll = 0;
			inc_pitch = PID_pitch.calc(setpoint, pitch);
			inc_roll = PID_roll.calc(setpoint, roll);
			cout<<"inc_pitch: "<<inc_pitch<<endl;
			cout<<"inc_roll: "<<inc_roll<<endl;

			angle_front = -pitch + inc_pitch;//pitch + inc_pitch;
			angle_back = -pitch + inc_pitch;//pitch + inc_pitch;
			angle_left = roll - inc_roll;//roll + inc_roll;
			angle_right = roll + inc_roll;//roll - inc_roll;
			cout<<"front: "<<angle_front<<endl;
			cout<<"back: "<<angle_back<<endl;
			
			angle_LF = -angle_back + angle_left;
			angle_RF = angle_front + angle_right;
			angle_LH = -angle_front + angle_left;
			angle_RH = angle_back + angle_right;
			
			//angle_LF = 0;
			//angle_RF = 0;
			//angle_LH = 0;
			//angle_RH = 0;
			
			result = angle2PWM_IMU(angle_LF , angle_RF, angle_LH, angle_RH );
			//cout<<"LF_PWM: "<<result[0]<<"  RF_PWM: "<<result[1]<<"  LH_PWM: "<<result[2]<<"  RH_PWM: "<<result[3]<<endl;
			pos[0] = result[0]; //Left Front
			pos[1] = result[1]; //Right Front
			pos[2] = result[2]; 
			pos[3] = result[3];
			I2C_ctrl(&pwm, pos);
			option = 5;
		}
		
		//Option6: Trot
		if(option == 6)
		{
			double* x_range = new double[2];
			x_range[0] = -5;
			x_range[1] = 5;
			
			double* y_range = new double[2];
			y_range[0] = -11;
			y_range[1] = -11;

			double* z_range = new double[2];
			z_range[0] = -15;
			z_range[1] = -8.5;	
				
			int resolution = 50;	
			double **input_RF;
			double **input_LF;

			input_RF = trot_RF( x_range, y_range, z_range, resolution);	//For Right Fore Leg
			input_LF = trot_LF( x_range, y_range, z_range, resolution); //For Left Fore Leg
			
			int repeat = 15;
			for(int j = 0; j < repeat; ++j)
			{
				for(int ii = 0; ii < 4*resolution; ++ii)
				{					
					temp_result_LF = inverse_kinematics( input_LF[0][ii], input_LF[1][ii], input_LF[2][ii]);
					double* LF = new double[3];
					LF[0] = PI - temp_result_LF[0];
					LF[1] = PI - temp_result_LF[1];
					LF[2] = PI - temp_result_LF[2];

					temp_result_RF = inverse_kinematics( input_RF[0][ii], input_RF[1][ii], input_RF[2][ii]);					
					double* RF = new double[3];
					RF[0] = temp_result_RF[0];
					RF[1] = temp_result_RF[1];
					RF[2] = temp_result_RF[2];

					double* LH = new double[3]; 
					LH[0] = RF[0];
					LH[1] = RF[1];
					LH[2] = -RF[2];
					
					double* RH = new double[3]; 
					RH[0] = LF[0];
					RH[1] = LF[1];
					RH[2] = -LF[2] + 2*PI;

					
					double* angle_corrected;
					angle_corrected = angle_temp_correction_LF(LF[0], LF[1], LF[2]);
					LF[0] = angle_corrected[0];
					LF[1] = angle_corrected[1];
					LF[2] = angle_corrected[2];
					
					angle_corrected = angle_temp_correction_RF(RF[0], RF[1], RF[2]);
					RF[0] = angle_corrected[0];
					RF[1] = angle_corrected[1];
					RF[2] = angle_corrected[2];
					
					angle_corrected = angle_temp_correction_RF(LH[0], LH[1], LH[2]);
					LH[0] = angle_corrected[0];
					LH[1] = angle_corrected[1];
					LH[2] = angle_corrected[2];
					
					angle_corrected = angle_temp_correction_LF(RH[0], RH[1], RH[2]);	
					RH[0] = angle_corrected[0];
					RH[1] = angle_corrected[1];
					RH[2] = angle_corrected[2];					
					
					//Convert Angle to PWM
					int* PWM;
					int LF_PWM[3];
					int RF_PWM[3];
					int LH_PWM[3];
					int RH_PWM[3];

					PWM = angle2PWM(LF[0], LF[1], LF[2]);
					LF_PWM[0] = PWM[0];
					LF_PWM[1] = PWM[1];
					LF_PWM[2] = PWM[2];
										
					PWM = angle2PWM(RF[0], RF[1], RF[2]);
					RF_PWM[0] = PWM[0];
					RF_PWM[1] = PWM[1];
					RF_PWM[2] = PWM[2];
					
					PWM = angle2PWM(LH[0], LH[1], LH[2]);
					LH_PWM[0] = PWM[0];
					LH_PWM[1] = PWM[1];
					LH_PWM[2] = PWM[2];
						
					PWM = angle2PWM(RH[0], RH[1], RH[2]);
					RH_PWM[0] = PWM[0];
					RH_PWM[1] = PWM[1];
					RH_PWM[2] = PWM[2];				
					
					//Write to Servo Motors			
					pos[0] = LF_PWM[0];
					pos[1] = LF_PWM[1];
					pos[2] = LF_PWM[2];
					pos[3] = RF_PWM[0];
					pos[4] = RF_PWM[1];
					pos[5] = RF_PWM[2];
					pos[6] = LH_PWM[0];
					pos[7] = LH_PWM[1];
					pos[8] = LH_PWM[2];
					pos[9] = RH_PWM[0];
					pos[10] = RH_PWM[1];
					pos[11] = RH_PWM[2];
					I2C_ctrl(&pwm, pos);				
				}
			}
			
			//Memory Cleaning
			for(int i = 0; i < 3; ++i)
			{	
				delete[] input_RF[i];
				delete[] input_LF[i];
				
			}
			delete[] input_RF;
			delete[] input_LF;
		}	
		
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

//cout commands======================================================================================================================================================
//For Inv kinematics Debugging
//cout << "q5: " << temp_result[0] << " alpha: " << temp_result[1] << " beta: "<<temp_result[2] << " gamma: "<<temp_result[3] << " q1: "<<temp_result[4] << endl;
//cout << "psi: " << temp_result[5] << " xi: " << temp_result[6] << " a: "<<temp_result[7] << " b: "<<temp_result[8] << " delta1: "<<temp_result[9] << endl;
//cout << "delta2: " << temp_result[10] << " q2: " << temp_result[11] << " phi0: "<<temp_result[12] << " q4: "<<temp_result[13] << " q3: "<<temp_result[14] << endl;

//double* forward = forward_kinematics(temp_result[0], temp_result[1]);
//double* forward = forward_kinematics((PI/180)*155.2, (PI/180)*27);
//cout << "P5x: " << forward[0] << " P5y: " << forward[1] << " q3: " << forward[2]<<" q4: " << forward[3] << " q5: " << forward[4]<<endl;
				
//From Trot------------------------------------------------------------------------------------------------	
//cout<<"input_RF[0]: "<<input_RF[0][ii]<<"  input_RF[1]: "<<input_RF[1][ii]<<"  input_RF[2]: "<<input_RF[2][ii]<<endl;
//cout<<"input_LF[0]: "<<input_LF[0][ii]<<"  input_LF[1]: "<<input_LF[1][ii]<<"  input_LF[2]: "<<input_LF[2][ii]<<endl;	
		
//cout<<"LF Knee: "<<(180/PI)*LF[0]<<"  LF Hip: "<<(180/PI)*LF[1]<<"  LF Shoulder: "<<(180/PI)*LF[2]<<endl;
//cout<<"RF Knee: "<<(180/PI)*RF[0]<<"  RF Hip: "<<(180/PI)*RF[1]<<"  RF Shoulder: "<<(180/PI)*RF[2]<<endl;
//cout<<"LH Knee: "<<(180/PI)*LH[0]<<"  LH Hip: "<<(180/PI)*LH[1]<<"  LH Shoulder: "<<(180/PI)*LH[2]<<endl;
//cout<<"RH Knee: "<<(180/PI)*RH[0]<<"  RH Hip: "<<(180/PI)*RH[1]<<"  RH Shoulder: "<<(180/PI)*RH[2]<<endl;				

//cout<<"LF Knee: "<<LF[0]<<"  LF Hip: "<<LF[1]<<"  LF Shoulder: "<<LF[2]<<endl;
//cout<<"RF Knee: "<<RF[0]<<"  RF Hip: "<<RF[1]<<"  RF Shoulder: "<<RF[2]<<endl;
//cout<<"LH Knee: "<<LH[0]<<"  LH Hip: "<<LH[1]<<"  LH Shoulder: "<<LH[2]<<endl;
//cout<<"RH Knee: "<<RH[0]<<"  RH Hip: "<<RH[1]<<"  RH Shoulder: "<<RH[2]<<endl;

//cout<<"LF_PWM Knee: "<<LF_PWM[0]<<"  LF_PWM Hip: "<<LF_PWM[1]<<"  LF_PWM Shoulder: "<<LF_PWM[2]<<endl;
//cout<<"RF_PWM Knee: "<<RF_PWM[0]<<"  RF_PWM Hip: "<<RF_PWM[1]<<"  RF_PWM Shoulder: "<<RF_PWM[2]<<endl;
//cout<<"LH_PWM Knee: "<<LH_PWM[0]<<"  LH_PWM Hip: "<<LH_PWM[1]<<"  LH_PWM Shoulder: "<<LH_PWM[2]<<endl;
//cout<<"RH_PWM Knee: "<<RH_PWM[0]<<"  RH_PWM Hip: "<<RH_PWM[1]<<"  RH_PWM Shoulder: "<<RH_PWM[2]<<endl;
					
