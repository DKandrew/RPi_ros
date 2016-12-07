#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include "pca9685.h"
#include "ctrl_algorithm.h"
#include "wiringPi.h"
#include "pid.h"

#define PI 3.14159265

using namespace std; 

// This function calls I2C control to servos
void I2C_ctrl(pca9685 *pwm, vector<int> & pos){
	pwm->set_pwm(pos);
}

int main(int argc, char**argv){
	//Initialize ROS system
	ros::init(argc, argv, "sevo_node_isc");
	ros::NodeHandle nh;
	//Frequency Control 
	int sampleRate = 100;
	ros::Rate rate(sampleRate);
	//Setup I2C
	int addr = 0x40;
	pca9685 pwm = pca9685(addr);
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
	
	//Main
	double x,y,z;
	int up = 0;	
	double init_x = 0;
	double init_y = -11;	
	double init_z = -15;	
	x = init_x;
	y = init_y;	
	z = init_z;	
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
			/*
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
			*/
			;
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
