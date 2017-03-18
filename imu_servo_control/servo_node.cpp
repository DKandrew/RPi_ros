#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <termios.h>
#include <ctime>
#include "pca9685.h"
#include "lidar.h"
#include "ctrl_algorithm.h"
#include "wiringPi.h"
#include "pid.h"
#include "velocity.h"
#include "imu_servo_control/imu_signal.h"
#include "std_msgs/String.h"

#define PI 3.14159265

using namespace std;

/*////////////////////////////////////////////////////////////////////////
 * This .cpp file contains functions required for controlling servo motors
/*////////////////////////////////////////////////////////////////////////

//<Variables Section>==============================================================
// Global data to store data received from imu_signal 
float roll, pitch, yaw, w_x, w_y, w_z, acc_x, acc_y, acc_z;

//Rate of Communication with LIDAR and IMU 
int sampleRate = 100;

//Servo position vector
int servo_num = 16;
vector<int> pos(servo_num);

//PWM
int* PWM;
int LF_PWM[3];
int RF_PWM[3];
int LH_PWM[3];
int RH_PWM[3];

//Angles for Joints right before converted to PWM
double result[4][3];

//Temporary Joint Angles
double angles[4][3];		//Joints angles before correction
double* temp_result; 		//Temporary variable to receive pointer(before angle correction)
double* angle_corrected;	//Temporary variable to receive pointer(after angle correction)

//General
int option = -1;	//Option for Menu selection
int off = 0;		//Condition for getting out of a menu's while loop

//Reference Points for assembling the robot
double angle_knee;
double angle_hip;
double angle_shoulder;

//pid control initialization
double dt = 1.0 / (double)sampleRate;
double pid_max = 10;
double pid_min = -10;
double Kp = 0.005;
double Kd = 0.01;
double Ki = 0.0;
double setpoint = 0;
double inc_pitch = 0;
double inc_roll = 0;

//Glide
double default_F = 30.0;	//Default offset for fore legs
double default_H = 30.0;	//Default offset for hind legs
double angle_fore = 0;		//To be added for fore  legs
double angle_hind = 0;		//To be added for hind  legs
double angle_left = 0;		//To be added for left  legs
double angle_right = 0;		//To be added for right legs
double angle_LF = 0;		//Left fore leg dihedral angle
double angle_RF = 0;		//Right fore leg dihedral angle
double angle_LH = 0;		//Left  hind leg dihedral angle
double angle_RH = 0;		//Right hind leg dihedral angle
double shoulder_F = 0;		//Shoulder angle of fore legs
double shoulder_H = 20;		//Shoulder angle of hind legs
double q5_LF;				//q5 angle of left  fore leg
double q5_RF;				//q5 angle of right fore leg
double q5_LH;				//q5 angle of left  hind leg
double q5_RH;				//q5 angle of right hind leg
double perchingAngle = 30.0;//Preset Perching Angle
bool init_glide = false;	//Boolean to initialize gliding
int status = 0;				//Variable to log current status of gliding
float inc_glide_keyboard = 0.5; //Fixed dihedral angle increment
float glideAngleLimit = 40; //Bounding dihedral angle
int glideMode = 0;

//Gait
double** input_LF;			//Double array pointer to record Left  fore leg data points
double** input_RF;			//Double array pointer to record Right fore leg data points
double** input_LH;			//Double array pointer to record Left  hind leg data points
double** input_RH;			//Double array pointer to record Right hind leg data points
double x_gait = 0;			//Center x point of ellipse foot trajectory
double y_gait = -9.8182;//-11;		//Center y point of ellipse foot trajectory
double z_gait = -8;//-10.75;		//Center z point of ellipse foot trajectory
int idx_LF = 0;				//Index of left  fore foot trajectory
int idx_RF = 0;				//Index of right fore foot trajectory
int idx_LH = 0;				//Index of left  hind foot trajectory
int idx_RH = 0;				//Index of right hind foot trajectory
double w_L = 3.0;			//Left  stride length
double w_R = 3.0;			//Right stride length
double h_L = 2.0;			//Left  stride height
double h_R = 2.0;			//Right stride height
double angle_gait = 0.0;	//Angle of foot trajectory (tilts ellipsoid)
int res_gait = 50;			//Resolution of foot trajectory (controls speed)
double ratio_gait = 0.5; 	//Ratio of stance and swing phase ( = stance/(stance + swing) )
int walk = 0;				//Select mode (0: Stationary 1: Gait)
int gear = 0;				//Select direction (1: forward 0: N -1: R)
int count_right = 0;		//Count number of right arrow keys pressed (Negative: left)

//Landing
bool landTrigger = false;	//Indicates landing Mode
double landTH = 0.7;		//Threshold(distance) for landing 
int count_rise = 0;			//Counting of arising motion due to landing preparation
int countTH = 20;			//Threshold for count_rise
bool launched = false;		//Indicates launching
double launchTH = 1;		//Threshold(distance) for launching
double launchAccTH = 10;	//Threshold(acceleration) for launching
double close2landTH = 0.5;	//Threshold(distance) for second phase of landing
double landedTH = 0.05;		//Threshold(distance) for indicating complete landing

//LIDAR
int addr_lidar = 0x62;		//Address of LIDAR
int dist;					//Distance output from LIDAR [cm]
double dist_m;				//Distance in [m]
double dist_m_prev;			//Precious distance in [m]

//Singular
double singular_ON_knee = 145*PI/180;	//Knee     angle required to be close to singular position
double singular_ON_hip = -10*PI/180;	//Hip      angle required to be close to singular position
double singular_ON_shoulder = 0*PI/180;	//shoulder angle required to be close to singular position

double singular_OFF_knee = 145*PI/180;	//Knee     angle required to be close to singular position
double singular_OFF_hip = 90*PI/180;	//Hip      angle required to be close to singular position
double singular_OFF_shoulder = 0*PI/180;//shoulder angle required to be close to singular position

//Setup I2C
int addr = 0x40;				//pca9685 address
pca9685 pwm = pca9685(addr);	//Initialize pca9685

//IMU Velocity Conversion
float diff = 1.0/sampleRate;
float offX = -38.98;
float offY = 3.52;
float offZ = 2.03;
float* v;

//Log File
ofstream data;			//Initialize ofstream 
int w = 12;				//Width for each column in data
bool logging = false;	//Boolean to turn ON and OFF logging
//End of Variable Section======================================================================


//<Functions Section>=============================================================
// This function calls I2C control to servos
void I2C_ctrl(pca9685 *pwm, vector<int> & pos) 
{
	pwm->set_pwm(pos);
}

//This is the callback function created for subscriber. 
//Every time subscriber receives imu_siganl, it will call this callback funcion to update the global data
void callback_function(const imu_servo_control::imu_signal & msg)
{
	//ROS_INFO_STREAM("Receiving IMU Signal: " << " x=" << msg.x << " y=" << msg.y << " z=" << msg.z);
	roll = msg.angle_x + 90.0;
	pitch = -msg.angle_y;
	yaw = msg.angle_z;
	w_x = msg.w_x;
	w_y = msg.w_y;
	w_z = msg.w_z;
	acc_x = msg.acc_x + 0.14;
	acc_y = msg.acc_z - 0.38;
	acc_z = msg.acc_y + 9.710297782608693;
	//cout << "Output from callback func: x=" << angle_x << " y=" << angle_y << " z=" << angle_z << endl;
}

//Convert Angles to PWM values
void angle2PWM()
{
	PWM = angle2PWM_LF(result[0][0], result[0][1], result[0][2]);
	LF_PWM[0] = PWM[0];
	LF_PWM[1] = PWM[1];
	LF_PWM[2] = PWM[2];

	PWM = angle2PWM_RF(result[1][0], result[1][1], result[1][2]);
	RF_PWM[0] = PWM[0];
	RF_PWM[1] = PWM[1];
	RF_PWM[2] = PWM[2];
				
	PWM = angle2PWM_LH(result[2][0], result[2][1], result[2][2]);
	LH_PWM[0] = PWM[0];
	LH_PWM[1] = PWM[1];
	LH_PWM[2] = PWM[2];

	PWM = angle2PWM_RH(result[3][0], result[3][1], result[3][2]);
	RH_PWM[0] = PWM[0];
	RH_PWM[1] = PWM[1];
	RH_PWM[2] = PWM[2];
	
	return;
}

//Print out PWM values to the console and "data.txt"
void printPWM()
{
	cout<<"LF_PWM Knee: "<<LF_PWM[0]<<"  LF_PWM Hip: "<<LF_PWM[1]<<"  LF_PWM Shoulder: "<<LF_PWM[2]<<endl;
	cout<<"RF_PWM Knee: "<<RF_PWM[0]<<"  RF_PWM Hip: "<<RF_PWM[1]<<"  RF_PWM Shoulder: "<<RF_PWM[2]<<endl;
	cout<<"LH_PWM Knee: "<<LH_PWM[0]<<"  LH_PWM Hip: "<<LH_PWM[1]<<"  LH_PWM Shoulder: "<<LH_PWM[2]<<endl;
	cout<<"RH_PWM Knee: "<<RH_PWM[0]<<"  RH_PWM Hip: "<<RH_PWM[1]<<"  RH_PWM Shoulder: "<<RH_PWM[2]<<endl;
	
	data<<"LF_PWM Knee: "<<LF_PWM[0]<<"  LF_PWM Hip: "<<LF_PWM[1]<<"  LF_PWM Shoulder: "<<LF_PWM[2]<<endl;
	data<<"RF_PWM Knee: "<<RF_PWM[0]<<"  RF_PWM Hip: "<<RF_PWM[1]<<"  RF_PWM Shoulder: "<<RF_PWM[2]<<endl;
	data<<"LH_PWM Knee: "<<LH_PWM[0]<<"  LH_PWM Hip: "<<LH_PWM[1]<<"  LH_PWM Shoulder: "<<LH_PWM[2]<<endl;
	data<<"RH_PWM Knee: "<<RH_PWM[0]<<"  RH_PWM Hip: "<<RH_PWM[1]<<"  RH_PWM Shoulder: "<<RH_PWM[2]<<endl;
	
	return;
}

//Print out angles to the console and "data.txt"
void printAngles()
{	
	cout<<"LF Knee: "<<angles[0][0]<<"  LF Hip: "<<angles[0][1]<<"  LF Shoulder: "<<angles[0][2]<<endl;
	cout<<"RF Knee: "<<angles[1][0]<<"  RF Hip: "<<angles[1][1]<<"  RF Shoulder: "<<angles[1][2]<<endl;
	cout<<"LH Knee: "<<angles[2][0]<<"  LH Hip: "<<angles[2][1]<<"  LH Shoulder: "<<angles[2][2]<<endl;
	cout<<"RH Knee: "<<angles[3][0]<<"  RH Hip: "<<angles[3][1]<<"  RH Shoulder: "<<angles[3][2]<<endl;
	
	data<<"LF Knee: "<<angles[0][0]<<"  LF Hip: "<<angles[0][1]<<"  LF Shoulder: "<<angles[0][2]<<endl;
	data<<"RF Knee: "<<angles[1][0]<<"  RF Hip: "<<angles[1][1]<<"  RF Shoulder: "<<angles[1][2]<<endl;
	data<<"LH Knee: "<<angles[2][0]<<"  LH Hip: "<<angles[2][1]<<"  LH Shoulder: "<<angles[2][2]<<endl;
	data<<"RH Knee: "<<angles[3][0]<<"  RH Hip: "<<angles[3][1]<<"  RH Shoulder: "<<angles[3][2]<<endl;

	return;
}

//Write PWM values to servo motors
void write2Motors()
{
	//Write to Servo Motors			
	pos[7] = LH_PWM[0];
	pos[6] = LH_PWM[1];
	pos[4] = LH_PWM[2];
	pos[3] = RH_PWM[0];
	pos[2] = RH_PWM[1];
	pos[0] = RH_PWM[2];
	pos[15] = RF_PWM[0];
	pos[13] = RF_PWM[1];
	pos[12] = RF_PWM[2];
	pos[11] = LF_PWM[0];
	pos[10] = LF_PWM[1];
	pos[8] = LF_PWM[2];
	I2C_ctrl(&pwm, pos);
	
	return;
}

//Covert joint angles to motor angles
void assignAngles(int option)
{
	
	for(int i = 0; i < 4; ++i)
	{
		if(option == 0)
		{
			if(i == 0 || i == 3)
				angle_corrected = angle_temp_correction_LF(angles[i][0], angles[i][1], angles[i][2]);
			if(i == 1 || i == 2)
				angle_corrected = angle_temp_correction_RF(angles[i][0], angles[i][1], angles[i][2]);
		}
		if(option == 1)
		{
			temp_result = inverse_kinematics(angles[i][0], angles[i][1], angles[i][2]);			
		}
		if(option == 2)
		{
			temp_result = glide(angles[i][0], angles[i][1], angles[i][2]);
		}
		if(option != 0 && (i == 0 || i == 3))
			angle_corrected = angle_temp_correction_LF(temp_result[0], temp_result[1], temp_result[2]);
		if(option != 0 && (i == 1 || i == 2))
			angle_corrected = angle_temp_correction_RF(temp_result[0], temp_result[1], temp_result[2]);
		result[i][0] = angle_corrected[0];
		result[i][1] = angle_corrected[1];
		result[i][2] = angle_corrected[2];			
	}

	return;
}

//Drive motors to put the legs very close to singular configuration
void singularON()
{
	double* angle_corrected;
	angle_corrected = angle_temp_correction_LF(singular_ON_knee, singular_ON_hip, singular_ON_shoulder);
	result[0][0] = angle_corrected[0];
	result[0][1] = angle_corrected[1];
	result[0][2] = angle_corrected[2];	
	result[3][0] = angle_corrected[0];
	result[3][1] = angle_corrected[1];
	result[3][2] = angle_corrected[2];	
			
	angle_corrected = angle_temp_correction_RF(singular_ON_knee, singular_ON_hip, singular_ON_shoulder);
	result[1][0] = angle_corrected[0];
	result[1][1] = angle_corrected[1];
	result[1][2] = angle_corrected[2];	
	result[2][0] = angle_corrected[0];
	result[2][1] = angle_corrected[1];
	result[2][2] = angle_corrected[2];	
	
	angle2PWM();
	write2Motors();
	
	return;	
}

//Drive motors to put the legs out of close to singular configuration
void singularOFF()
{	
	angle_corrected = angle_temp_correction_LF(singular_OFF_knee, singular_OFF_hip, singular_OFF_shoulder);
	result[0][0] = angle_corrected[0];
	result[0][1] = angle_corrected[1];
	result[0][2] = angle_corrected[2];	
	result[3][0] = angle_corrected[0];
	result[3][1] = angle_corrected[1];
	result[3][2] = angle_corrected[2];
			
	angle_corrected = angle_temp_correction_RF(singular_OFF_knee, singular_OFF_hip, singular_OFF_shoulder);
	result[1][0] = angle_corrected[0];
	result[1][1] = angle_corrected[1];
	result[1][2] = angle_corrected[2];	
	result[2][0] = angle_corrected[0];
	result[2][1] = angle_corrected[1];
	result[2][2] = angle_corrected[2];	

	angle2PWM();
	write2Motors();
	
	return;	
}

//Calculate and assign q5 values
void getq5()
{
	q5_LF = 155.0 + (angle_LF + 40.0) / 8.0;
	q5_RF = 155.0 + (angle_RF + 40.0) / 8.0;
	q5_LH = 155.0 + (angle_LH + 40.0) / 8.0;
	q5_RH = 155.0 + (angle_RH + 40.0) / 8.0;
	
	return;
}

//Real-time Keyboard Input (Enter is required)
int _getch()
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

//Real-time Keyboard Input
char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = { 0 };
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if (rv == -1)
		ROS_ERROR("select");
	else if (rv == 0)
	{
		//ROS_INFO("no_key_pressed");
	}
	else
		read(filedesc, &buff, len);

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR("tcsetattr ~ICANON");
	return (buff);
}

//Print out current status of gliding on console
void getStatus_glide()
{
	printf("-----------------------------------------------------\n");
	if(status == 0)
		printf("<Standby Mode>\n");
	if(status == 1)
		printf("<Gliding Mode>\n");
	if(status == 2)
		printf("<Preparing Landing>\n");
	if(status == 3)
		printf("<Shoulder Tension Decreased>\n");
	if(status == 4)
		printf("<Close to Land>\n");
	if(status == 5)
		printf("<Landed>\n");
		
	printf("Roll: %.2f Pitch: %.2f Yaw: %.2f Acc_x: %.2f Acc_y: %.2f Acc_z: %.2f\n",roll, pitch, yaw, acc_x, acc_y, acc_z);
	printf("Distance: %.2fm\n", dist_m);
	printf("PID Increment:: roll: %.2f  pitch: %.2f\n", inc_roll, inc_pitch);
	printf("angle_fore: %.2f  angle_hind: %.2f\n", angle_fore, angle_hind);
	printf("angle_left: %.2f  angle_right: %.2f\n", angle_left, angle_right);
	printf("angle_LF: %.2f  angle_RF: %.2f  angle_LH: %.2f  angle_RH: %.2f\n", angle_LF, angle_RF, angle_LH, angle_RH);
	if(logging)
		printf("Logging Data ON\n\n");
	else
		printf("Logging Data OFF\n\n");

	return;
}

//Print out name of column at "data.txt" for data logging
void logGlideInit()
{
	data<<fixed<<setfill(' ')<<setw(w)<<"Roll";
	data<<fixed<<setfill(' ')<<setw(w)<<"Pitch";
	data<<fixed<<setfill(' ')<<setw(w)<<"Yaw";
	data<<fixed<<setfill(' ')<<setw(w)<<"w_x";
	data<<fixed<<setfill(' ')<<setw(w)<<"w_y";
	data<<fixed<<setfill(' ')<<setw(w)<<"w_z";
	data<<fixed<<setfill(' ')<<setw(w)<<"Vel_X";
	data<<fixed<<setfill(' ')<<setw(w)<<"Vel_Y";
	data<<fixed<<setfill(' ')<<setw(w)<<"Vel_Z";
	data<<fixed<<setfill(' ')<<setw(w)<<"Acc_X";
	data<<fixed<<setfill(' ')<<setw(w)<<"Acc_Y";
	data<<fixed<<setfill(' ')<<setw(w)<<"Acc_Z";
	data<<fixed<<setfill(' ')<<setw(w)<<"Dist[m]";
	data<<fixed<<setfill(' ')<<setw(w)<<"Inc Roll";
	data<<fixed<<setfill(' ')<<setw(w)<<"Inc Pitch";
	data<<fixed<<setfill(' ')<<setw(w)<<"Fore";
	data<<fixed<<setfill(' ')<<setw(w)<<"Hind";
	data<<fixed<<setfill(' ')<<setw(w)<<"Left";
	data<<fixed<<setfill(' ')<<setw(w)<<"Right";
	data<<fixed<<setfill(' ')<<setw(w)<<"LF";
	data<<fixed<<setfill(' ')<<setw(w)<<"RF";
	data<<fixed<<setfill(' ')<<setw(w)<<"LH";
	data<<fixed<<setfill(' ')<<setw(w)<<"RH"<<endl;
	
	return;
}

//Log data to "data.txt"
void logGlide()
{	
	data<<fixed<<setfill(' ')<<setw(w)<<roll;
	data<<fixed<<setfill(' ')<<setw(w)<<pitch;
	data<<fixed<<setfill(' ')<<setw(w)<<yaw;
	data<<fixed<<setfill(' ')<<setw(w)<<w_x;
	data<<fixed<<setfill(' ')<<setw(w)<<w_y;
	data<<fixed<<setfill(' ')<<setw(w)<<w_z;
	data<<fixed<<setfill(' ')<<setw(w)<<v[0];
	data<<fixed<<setfill(' ')<<setw(w)<<v[1];
	data<<fixed<<setfill(' ')<<setw(w)<<v[2];
	data<<fixed<<setfill(' ')<<setw(w)<<acc_x;
	data<<fixed<<setfill(' ')<<setw(w)<<acc_y;
	data<<fixed<<setfill(' ')<<setw(w)<<acc_z;
	data<<fixed<<setfill(' ')<<setw(w)<<dist_m;
	data<<fixed<<setfill(' ')<<setw(w)<<inc_roll;
	data<<fixed<<setfill(' ')<<setw(w)<<inc_pitch;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_fore;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_hind;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_left;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_right;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_LF;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_RF;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_LH;
	data<<fixed<<setfill(' ')<<setw(w)<<angle_RH<<endl;
	
	return;	
}

//Handle real-time keyboard input for gliding
void handleKey_glide(char c)
{
	//Terminate while loop
	if(c == '/')
		off = 1;
		
	//Print Current Status
	if(c == '.')
		getStatus_glide();
		
	//Log Start
	if(c == 'l')
	{
		logging = !logging;
		if(!logging)
		{
			cout<<"Log Mode OFF"<<endl;
			data<<"Log Mode OFF"<<endl;						
		}
		if(!init_glide && logging)
		{
			cout<<"Log Mode ON"<<endl;
			data<<"Log Mode ON"<<endl;	
			logGlideInit();
		}
	}
	
	//Individual Leg Control
	//LF Dihedral
	if(c == 'q')
	{
		angle_LF -= inc_glide_keyboard;
		printf(" <= Pressed  LF Dihedral Decreased => Current angle_LF: %.1f Degree\n", angle_LF);
	}
	if(c == 'w')
	{
		angle_LF += inc_glide_keyboard;
		printf(" <= Pressed  LF Dihedral Increased => Current angle_LF: %.1f Degree\n", angle_LF);
	}	
	//RF Dihedral
	if(c == 'e')
	{
		angle_RF -= inc_glide_keyboard;
		printf(" <= Pressed  RF Dihedral Decreased => Current angle_LF: %.1f Degree\n", angle_RF);
	}
	if(c == 'r')
	{
		angle_RF += inc_glide_keyboard;
		printf(" <= Pressed  RF Dihedral Increased => Current angle_LF: %.1f Degree\n", angle_RF);
	}	
	//LH Dihedral
	if(c == 'a')
	{
		angle_LH -= inc_glide_keyboard;
		printf(" <= Pressed  LH Dihedral Decreased => Current angle_LF: %.1f Degree\n", angle_LH);
	}
	if(c == 's')
	{
		angle_LH += inc_glide_keyboard;
		printf(" <= Pressed  LH Dihedral Increased => Current angle_LF: %.1f Degree\n", angle_LH);

	}
	//RH Dihedral
	if(c == 'd')
	{
		angle_RH -= inc_glide_keyboard;
		printf(" <= Pressed  RH Dihedral Decreased => Current angle_LF: %.1f Degree\n", angle_RH);
	}
	if(c == 'f')
	{
		angle_RH += inc_glide_keyboard;
		printf(" <= Pressed  RH Dihedral Increased => Current angle_LF: %.1f Degree\n", angle_RH);

	}
	
	//Fore and Hind Pair Control
	//Fore Pair
	if(c == 't')
	{
		angle_LF -= inc_glide_keyboard;
		angle_RF -= inc_glide_keyboard;
		printf(" <= Pressed  Fore Dihedral Decreased => Current angle_LF: %.1f Degree  angle_RF: %.1f Degree\n", angle_LF, angle_RF);
	}
	if(c == 'y')
	{
		angle_LF += inc_glide_keyboard;
		angle_RF += inc_glide_keyboard;
		printf(" <= Pressed  Fore Dihedral Increased => Current angle_LF: %.1f Degree  angle_RF: %.1f Degree\n", angle_LF, angle_RF);
	}
	//Hind Pair
	if(c == 'g')
	{
		angle_LH -= inc_glide_keyboard;
		angle_RH -= inc_glide_keyboard;
		printf(" <= Pressed  Hind Dihedral Decreased => Current angle_LH: %.1f Degree  angle_RH: %.1f Degree\n", angle_LH, angle_RH);
	}
	if(c == 'h')
	{
		angle_LH += inc_glide_keyboard;
		angle_RH += inc_glide_keyboard;
		printf(" <= Pressed  Hind Dihedral Increased => Current angle_LH: %.1f Degree  angle_RH: %.1f Degree\n", angle_LH, angle_RH);
	}
	
	//Fore and Hind Pair Tension Control
	//Fore Pair
	if(c == 'u')
	{
		shoulder_F -= inc_glide_keyboard;
		printf(" <= Pressed  Fore Tension Decreased => Current shoulder_F: %.1f Degree \n", shoulder_F);
	}
	if(c == 'i')
	{
		shoulder_F += inc_glide_keyboard;
		printf(" <= Pressed  Fore Tension Increased => Current shoulder_F: %.1f Degree \n", shoulder_F);
	}
	//Hind Pair
	if(c == 'j')
	{
		shoulder_H -= inc_glide_keyboard;
		printf(" <= Pressed  Hind Tension Decreased => Current shoulder_H: %.1f Degree \n", shoulder_H);
	}
	if(c == 'k')
	{
		shoulder_H += inc_glide_keyboard;
		printf(" <= Pressed  Hind Tension Decreased => Current shoulder_H: %.1f Degree \n", shoulder_H);
	}
	
	//Preset Angles
	//Home Position
	if(c == 'z')
	{
		angle_LF = default_F;
		angle_RF = default_F;
		angle_LH = default_H;
		angle_RH = default_H;
		printf(" <= Pressed  Home Positioning");
	}
	//Manual Mode
	if(c == 'x')
	{
		glideMode = 0;
		printf("Manual Mode\n");
		data<<"Manual Mode"<<endl;
	}
	//PID Horizontal Mode
	if(c == 'c')
	{
		glideMode = 1;
		printf("PID Mode\n");
		data<<"PID Mode"<<endl;
	}
	//Perching
	if(c == 'v')
	{
		glideMode = 2;		
		printf(" <= Pressed  Perching Mode");
	}
	
	
}

//Handle real-time keyboard input for gait
void handleKey_gait(char c)
{
	//Gait Width Control
	if (c == 'q')
	{
		w_L += 0.5;
		w_R += 0.5;
		printf(" <= Pressed  Gait Width Increased => Current w_L: %.1f cm   w_R: %.1f cm\n", w_L, w_R);
	}
	if (c == 'a')
	{
		w_L -= 0.5;
		w_R -= 0.5;
		printf(" <= Pressed  Gait Width Decreased => Current w_L: %.1f cm   w_R: %.1f cm\n", w_L, w_R);
	}

	//Gait Height Control
	if (c == 'w')
	{
		h_L += 0.5;
		h_R += 0.5;
		printf(" <= Pressed  Gait Height Increased => Current h_L: %.1f cm   h_R: %.1f cm\n", h_L, h_R);
	}
	if (c == 's')
	{
		h_L -= 0.5;
		h_R -= 0.5;
		printf(" <= Pressed  Gait Height Decreased => Current h_L: %.1f cm   h_R: %.1f cm\n", h_L, h_R);
	}

	//Gait Angle Contorl
	if (c == 'e')
	{
		angle_gait += 5.0;
		printf(" <= Pressed  Angle Increased => Current Angle: %.2f Degree\n", angle_gait);
	}
	if (c == 'd')
	{
		angle_gait -= 5.0;
		printf(" <= Pressed  Angle Decreased => Current Angle: %.2f Degree\n", angle_gait);
	}

	//Gait Speed Control
	if (c == 'r')
	{
		res_gait -= 2;
		idx_LF = (double)res_gait*(double)idx_LF/(double)(res_gait + 2);
		idx_RF = (double)res_gait*(double)idx_RF/(double)(res_gait + 2);
		idx_LH = (double)res_gait*(double)idx_LH/(double)(res_gait + 2);
		idx_RH = (double)res_gait*(double)idx_RH/(double)(res_gait + 2);
		printf(" <= Pressed  Speed Increased => Current Resolution: %i\n", res_gait);
	}
	if (c == 'f')
	{
		res_gait += 2;
		idx_LF = (double)res_gait*(double)idx_LF/(double)(res_gait - 2);
		idx_RF = (double)res_gait*(double)idx_RF/(double)(res_gait - 2);
		idx_LH = (double)res_gait*(double)idx_LH/(double)(res_gait - 2);
		idx_RH = (double)res_gait*(double)idx_RH/(double)(res_gait - 2);
		printf(" <= Pressed  Speed Decreased => Current Resolution: %i\n", res_gait);
	}

	//Gait Swing and Stance Ratio Control
	if (c == 't')
	{
		ratio_gait += 0.05;
		printf(" <= Pressed  Gait Ratio Increased => Stance: %i%, Swing: %i%\n", (int)(100.0*ratio_gait), (int)(100.0*(1.0 - ratio_gait)));
	}
	if (c == 'g')
	{
		ratio_gait -= 0.05;
		printf(" <= Pressed  Gait Ratio Decreased => Stance: %i%, Swing: %i%\n", (int)(100.0*ratio_gait), (int)(100.0*(1 - ratio_gait)) );
	}

	//Phase of Each Leg Control
	//LF
	if (c == 'y')
	{
		++idx_LF;
		printf(" <= Pressed  LF Leg Phase Increased\n");
	}
	if (c == 'h')
	{
		--idx_LF;
		printf(" <= Pressed  LF Leg Phase Decreased\n");
	}
	//RF
	if (c == 'u')
	{
		++idx_RF;
		printf(" <= Pressed  RF Leg Phase Increased\n");
	}
	if (c == 'j')
	{
		--idx_RF;
		printf(" <= Pressed  RF Leg Phase Decreased\n");
	}
	//LH
	if (c == 'i')
	{
		++idx_LH;
		printf(" <= Pressed  LH Leg Phase Increased\n");
	}
	if (c == 'k')
	{
		--idx_LH;
		printf(" <= Pressed  LH Leg Phase Decreased\n");
	}
	//RH
	if (c == 'o')
	{
		++idx_RH;
		printf("<= Pressed  RH Leg Phase Increased\n");
	}
	if (c == 'l')
	{
		--idx_RH;
		printf("<= Pressed  RH Leg Phase Decreased\n");
	}

	//Trot
	if (c == 'z')
	{
		ratio_gait = 0.6;
		res_gait = 70;
		idx_RF = idx_LF + res_gait/2.0;
		idx_LH = idx_LF + res_gait/2.0;
		idx_RH = idx_LF;
		printf(" <= Pressed\n => Trot Mode\n");
	}			
	//Lateral Sequence Walk
	if (c == 'x')
	{
		ratio_gait = 0.7;
		res_gait = 100;
		idx_RF = idx_LF + res_gait*0.5;
		idx_LH = idx_LF + res_gait*0.25;
		idx_RH = idx_LF + res_gait*0.75;		
		printf(" <= Pressed\n => Lateral Sequence Mode\n");
	}
	//Diagonal Sequence Walk
	if (c == 'c')
	{
		ratio_gait = 0.7;
		res_gait = 100;
		idx_RF = idx_LF + res_gait*0.5;
		idx_LH = idx_LF + res_gait*0.75;
		idx_RH = idx_LF + res_gait*0.25;				
		printf(" <= Pressed\n => Diagonal Sequence Mode\n");
	}
	if(c == 'v')
	{
		++walk;
		if(walk >= 2)
			walk = 0;
		if(walk == 0)
			printf(" <= Pressed\n => Walking Mode\n");
		else
		{
			idx_LF = 0;
			idx_RF = 0;
			idx_LH = 0;
			idx_RH = 0;
			ratio_gait = 0.5;
			printf(" <= Pressed\n => Stationary Mode\n");
		}
	}
				
	//Arrow Key Keyboard Input
	if (c == '\033')
	{
		getch();
		char arrow = getch();
		if (arrow == 'A')
		{
			++gear;
			if(gear >= 1)
				gear = 1;
			printf(" Up Arrow Key Pressed\n");
			if(gear == 0)
				printf(" Gear: %i => Idle Mode\n", gear);
			if(gear == 1)
				printf(" Gear: %i => Move Forward\n", gear);
		}
		if (arrow == 'B')
		{
			--gear;
			if(gear < -1)
				gear = -1;
			printf(" Down Arrow Key Pressed\n");
			if(gear == 0)
				printf(" Gear: %i => Idle Mode\n", gear);
			if(gear == -1)
				printf(" Gear: %i => Move Backward\n", gear);			
		}
		if (arrow == 'C')
		{
			w_L += 0.5;
			w_R -= 0.5;
			++count_right;
			printf("Right Arrow Key Pressed\n => Current w_L = %0.1f  w_R = %0.1f\n", w_L, w_R);
		}
		if (arrow == 'D')
		{
			w_L -= 0.5;
			w_R += 0.5;
			--count_right;
			printf("Left Arrow Key Pressed\n => Current w_L = %0.1f  w_R = %0.1f\n", w_L, w_R);
		}
	}
	
	//Get Current Status
	if(c == '.')
	{
		printf("Current Status----------------------------------\n");
		if(walk == 0)
			printf("Mode: Gait Mode\n");
		else
			printf("Mode: Swag Mode\n");
		if(gear == 1)
			printf("Gear: 1\n");
		if(gear == 0)
			printf("Gear: N\n");
		if(gear == -1)
			printf("Gear: R\n");
		if(count_right == 0)
			printf("Direction: Moving Strait\n");
		if(count_right > 0)
			printf("Direction: Right %i\n", count_right);
		if(count_right < 0)
			printf("Direction: Left %i\n", -count_right);	
		
		printf("Left Width: %0.1f  Right Width: %0.1f\n", w_L, w_R);
		printf("Left Height: %0.1f  Right Height: %0.1f\n", h_L, h_R);
		printf("Resolution: %i\n", res_gait);
		printf("Stance Phase: %i%\n", (int)(100*ratio_gait));
		printf("Angle: %i\n", (int)angle_gait);
		printf("------------------------------------------------\n");
	}
	
	//Terminate while loop
	if(c == '/')
		off = 1;
	return;
}
//End of function section====================================================================

//Main Function Starts Here===================================================================
int main(int argc, char**argv) 
{
	//Initialize ROS system
	ros::init(argc, argv, "servo_node_isc");
	ros::NodeHandle nh;
	
	//Frequency Control 
	ros::Rate rate(sampleRate);
	
	//Create a Subscriber
	ros::Subscriber sub = nh.subscribe("isc/imu_signal", 100, callback_function);
		
	//Set PWM frequency
	pwm.set_pwm_freq(100);

	//Setup GPIO
	wiringPiSetup();
	int pinNumber = 29;
	pinMode(pinNumber, OUTPUT);
	int LED_ON = 0;
	
	//PID Initialization
	pid PID_pitch = pid(dt, pid_max, pid_min, Kp, Kd, Ki);
	pid PID_roll = pid(dt, pid_max, pid_min, Kp, Kd, Ki);
	
	//LIDAR Initialization
	lidar _lidar = lidar(addr_lidar);
	
	//Velocity Initialization
	Velocity vel = Velocity(diff, offX, offY, offZ);	
	
	//Logging Data Initialization
	data.open("data.txt", ofstream::out | ofstream::app);
	time_t t = time(0);
	struct tm *now = localtime(&t);
	data<<"\n==============================<Da Real One(DR1) Data Logging Started>===========================\n";
	data<<setfill('0')<<"Date: "<<(now->tm_year + 1900)<<'/'<<setw(2)<<(now->tm_mon)+1<<'/'<<setw(2)<<now->tm_mday;
	data<<setfill('0')<<"   Time: "<<setw(2)<<now->tm_hour<<':'<<setw(2)<<now->tm_min<<':'<<setw(2)<<now->tm_sec<<endl;
	data<<endl;
	
	option = -1;

	
	//Main ROS While Loop Starts
	while (ros::ok())
	{
		//Invoke Subscriber
		ros::spinOnce();  //Execute callback_funcion to update IMU information 
						 
		//Real-time Keyboard Input
		char c = getch();
						  
		//LIDAR reading
		//dist = cos(pitch)*(_lidar.i2cRead() - 13);
		//dist_m = (double)dist/100.0;
		
		//While loop for choosing a menu
		if(option == -1)
		{
			while(option != -1 || option != 0 || option != 1 || option != 2 || option != 3 || option != 4 || option != 5 || option != 6 || option != 7)
			{
				cout<<endl;
				cout<<"< Da Real One(DR1) Operational >"<<endl;
				cout<<"=============================================="<<endl;
				cout<<"Choose an Option"<<endl;
				cout<<"0: Debugging mode"<<endl;
				cout<<"1: Gliding Angle Control"<<endl;
				cout<<"2: Gliding Mode"<<endl;
				cout<<"3: Gait Mode"<<endl;
				cout<<"4: Singular ON"<<endl;
				cout<<"5: Singular OFF"<<endl;
				cout<<"6: Manual Gliding Mode"<<endl;
				cout<<"7: PWM Frequency Control"<<endl;
				cout<<endl;
				cout<<"(To Terminate, press ctrl+c => press 0 => Enter)"<<endl;
				cout<<"=============================================="<<endl;
				cout<<"Option: "; cin >> option;
				cout<<endl;
				option = (int)option;
				cout<<"Choosen Option: "<<option<<endl;
				if( option == 0 || option == 1 || option == 2 || option == 3 || option == 4 || option == 5 || option == 6 || option == 7)
					break;
				else
					cout<<"Please Select from 0, 1, 2, 3, 4, 5, 6 and 7"<<endl;
			}		
		}				  

		off = 0;	//set off = 0 to make sure each option's while loop works

		//Option 0: <Debugging Mode> -----------------------------------
		//This mode drives servo motors to have
		//LF & RH: Knee     = 0  degree  RF & LH: knee     = 180 degree
		//LF & RH: Hip      = 90 degree  RF & LH: Hip      = 90  degree
		//LF & RH: Shoulder = 0  degree  RF & LH: Shoulder = 0   degree
		//This configuration helps assembling the robot
		//--------------------------------------------------------------
		if (option == 0)
		{
			cout<<"<Debugging Mode Selected>"<<endl;
			data<<"<Debugging Mode Selected>"<<endl;
			angle_knee = 180*(PI / 180);
			angle_hip = 90*(PI / 180);
			angle_shoulder = 0;
			
			angles[0][0] = angle_knee; angles[0][1] = angle_hip; angles[0][2] = angle_shoulder;
			angles[1][0] = angle_knee; angles[1][1] = angle_hip; angles[1][2] = angle_shoulder;
			angles[2][0] = angle_knee; angles[2][1] = angle_hip; angles[2][2] = angle_shoulder;
			angles[3][0] = angle_knee; angles[3][1] = angle_hip; angles[3][2] = angle_shoulder;

			assignAngles(0);
			
			angle2PWM();
			printPWM();
			write2Motors();
			
			dist_m_prev = dist_m;
			dist = cos(pitch)*(_lidar.i2cRead() - 13);
			dist_m = (double)dist/100.0;
			printf("Lidar Reading: %.2f m\n", dist_m);
			data<<"Lidar Reading: "<<dist_m<<endl;
			option = -1;
			
			cout<<"<Debugging Mode Complete>\n"<<endl;
			data<<"<Debugging Mode Complete>\n"<<endl;
		}		
		
		//Option 1: <Gliding Angle Control>--------------------------------------------
		//This mode requires user to enter dihedral & shoulder angles for fore and hind
		//This mode has two different functions
		//Function 1: Check tension of membrane
		//Function 2: Check if the robot can have correct dihedral angles
		//-----------------------------------------------------------------------------
		
		if (option == 1)
		{
			/*
			double dihedral_fore;
			double dihedral_hind;
			
			cout << "<Angle Control Mode Selected>" << endl;
			data << "<Angle Control Mode Selected>" << endl;
			
			cout << "Enter Desired Fore Dihedral Angle: "; cin >> dihedral_fore;
			dihedral_fore = (double)dihedral_fore;
			data << "Enter Desired Fore Dihedral Angle: "<<dihedral_fore<<endl;
			
			cout << "Enter Desired Hind Dihedral Angle: "; cin >> dihedral_hind;
			dihedral_hind = (double)dihedral_hind;
			data << "Enter Desired Hind Dihedral Angle: "<<dihedral_hind<<endl;
			
			cout << "Enter Desired Fore Shoulder Angle: "; cin >> shoulder_F;
			shoulder_F = (double)shoulder_F;
			data << "Enter Desired Fore Shoulder Angle: "<<shoulder_F<<endl;			
			
			cout << "Enter Desired Hind Shoulder Angle: "; cin >> shoulder_H;
			shoulder_H = (double)shoulder_H;
			data << "Enter Desired Hind Shoulder Angle: "<<shoulder_H<<endl;
			
			getq5();
			
			angles[0][0] = dihedral_fore; angles[0][1] = shoulder_F; angles[0][2] = q5_LF;
			angles[1][0] = dihedral_fore; angles[1][1] = shoulder_F; angles[1][2] = q5_RF;
			angles[2][0] = dihedral_hind; angles[2][1] = shoulder_H; angles[2][2] = q5_LH;
			angles[3][0] = dihedral_hind; angles[3][1] = shoulder_H; angles[3][2] = q5_RH;
			*/
			cout << "<Angle Control Mode Selected>" << endl;
			data << "<Angle Control Mode Selected>" << endl;
			
			cout << "Enter Desired LF Dihedral Angle: "; cin >> angle_LF;
			angle_LF = (double)angle_LF;
			
			cout << "Enter Desired RF Dihedral Angle: "; cin >> angle_RF;
			angle_RF = (double)angle_RF;
			
			cout << "Enter Desired LH Dihedral Angle: "; cin >> angle_LH;
			angle_LH = (double)angle_LH;
			
			cout << "Enter Desired RH Dihedral Angle: "; cin >> angle_RH;
			angle_RH = (double)angle_RH;
			
			cout << "Enter Desired Fore Shoulder Angle: "; cin >> shoulder_F;
			shoulder_F = (double)shoulder_F;			
			
			cout << "Enter Desired Hind Shoulder Angle: "; cin >> shoulder_H;
			shoulder_H = (double)shoulder_H;
			
			getq5();
			
			angles[0][0] = angle_LF; angles[0][1] = shoulder_F; angles[0][2] = q5_LF;
			angles[1][0] = angle_RF; angles[1][1] = shoulder_F; angles[1][2] = q5_RF;
			angles[2][0] = angle_LH; angles[2][1] = shoulder_H; angles[2][2] = q5_LH;
			angles[3][0] = angle_RH; angles[3][1] = shoulder_H; angles[3][2] = q5_RH;
			assignAngles(2);

			angle2PWM();
			printPWM();
			write2Motors();
			option = -1;
			
			cout << "<Angle Control Mode Complete>\n" << endl;
			data << "<Angle Control Mode Complete>\n" << endl;

		}

		//Option 2: <Gliding Mode>---------------------------------------------------
		//This mode continuously reveives data from IMU and LIDAR
		//Based on the data, it controls the wing to glide safely
		//When the robot is about to land(determined by distance reading from LIDAR),
		//The robot will prepare for landing
		//---------------------------------------------------------------------------
		if (option == 2)
		{	
			//Print out to console and "data.txt"
			printf("====================<Gliding Mode Started>====================\n");
			data<<"=====================<Gliding Mode Started>===================="<<endl;
			printf("To get Current Status, press '.' Key\n");
			printf("To turn ON or OFF Log Mode, press 'l' Key (Initially OFF)\n");			
			printf("To terminate Gliding Mode, press '/' key\n\n");
			
			//Initialize variables	
			init_glide = true;
			launched = false;
			shoulder_F = 0;
			shoulder_H = 20;
			option = -1;
			
			//While '/' key is pressed
			while(off == 0)
			{	
				//Read Keyboard Input
				c = getch();
				
				//Handle keyboard input
				handleKey_glide(c);
					
				//IMU Update
				ros::spinOnce();
				vel.updateVel(acc_x, acc_y, acc_z);
				vel.transform(w_x, w_y, w_z);
				v = vel.getV();
				
				//Lidar Update
				dist_m_prev = dist_m;
				dist = cos(pitch)*(_lidar.i2cRead() - 13);
				dist_m = (double)dist/100.0;	
				
				//Determine if the robot is launched
				if(!launched && acc_x >= launchAccTH && dist_m >= launchTH)
				{
					launched = true;
					status = 1;
					printf("Launched!\n");
					data<<"Launched!"<<endl;
				}										
				
				//PID Control
				inc_pitch = PID_pitch.calc(setpoint, pitch);
				inc_roll = PID_roll.calc(setpoint, roll);			

				//Calculate fore, hind, left & right using PID increment
				angle_fore = -pitch - inc_pitch;
				angle_hind = pitch + inc_pitch;
				angle_left = -roll - inc_roll;
				angle_right = roll + inc_roll;
				
				//Combine fore hind left right to calculate LF RF LH RH angles
				angle_LF = default_F + angle_fore;// + angle_left;
				angle_RF = default_F + angle_fore;// + angle_right;
				angle_LH = default_H + angle_hind;// + angle_left;
				angle_RH = default_H + angle_hind;// + angle_right;	
				
				//Bounding angles			
				if(angle_LF >= glideAngleLimit) 
					angle_LF = glideAngleLimit;
				if(angle_RF >= glideAngleLimit) 
					angle_RF = glideAngleLimit;
				if(angle_LH >= glideAngleLimit) 
					angle_LH = glideAngleLimit;
				if(angle_RH >= glideAngleLimit) 
					angle_RH = glideAngleLimit;
				if(angle_LF <= -glideAngleLimit) 
					angle_LF = -glideAngleLimit;
				if(angle_RF <= -glideAngleLimit) 
					angle_RF = -glideAngleLimit;
				if(angle_LH <= -glideAngleLimit) 
					angle_LH = -glideAngleLimit;
				if(angle_RH <= -glideAngleLimit) 
					angle_RH = -glideAngleLimit;
				
				
				//< Landing Algorithm >
				// If it is launched && It is the first time to reach threshold height, activate Landing Mode
				if(launched && dist_m <= landTH && !landTrigger)
				{
					landTrigger = true;
					status = 2;
					data<<"Landing Mode ON!"<<endl;
					printf("Landing Mode ON!\n");
				}
				
				// If it is Landing Mode, change dihedral angles to make the robot nose up		
				if(landTrigger)
				{
					angle_LF = 28;					
					angle_RF = 28;
					angle_LH = -28;
					angle_RH = -28;
				}
				
				// If Landing Mode is activated & height is increasing, increase count
				if(landTrigger && dist_m_prev <= dist_m)
					++count_rise;
				
				// If Landing Mode is activated & current height is less than threshold
				// && count_rise is greater than threshold(robot was rising for a while),
				// Then decrease shoulder angle to increase lift force 
				if(landTrigger && dist_m <= 0.5 && count_rise >= countTH)
				{
					shoulder_H = 0;
					status = 3;
					data<<"Rise Detected, Changing Shoulder Angle to '0'"<<endl;
					printf("Rise Detected, Changing Shoulder Angle to '0'\n");
				}
				
				//If it is Landing Mode & close to land, raise legs to protect the legs
				if(landTrigger && dist_m <= close2landTH)
				{
					angle_LF = 28;					
					angle_RF = 28;
					angle_LH = 28;
					angle_RH = 28;
					status = 4;
				}
				
				//If it is Landing Mode & landed
				if(landTrigger && dist_m <= landedTH)
				{
					status = 5;
					data<<"Landed"<<endl;
					printf("Landed\n");
					data<<"Preparing for Gait..."<<endl;
					printf("Preparing for Gait...\n");
					ros::Duration(2).sleep();
					data<<"Singular OFF..."<<endl;
					printf("Singular OFF...\n");
					singularOFF();
					ros::Duration(1).sleep();
					data<<"Entering to Gait Mode..."<<endl;
					printf("Entering to Gait Mode...\n");
					option = 3;
					break;
				}
				
				//If it is the first time in the loop, prepare for gliding				
				if(init_glide)
				{
					singularON();
					data<<"Initializing to Singular Configuration..."<<endl;
					printf("Initializing to Singular Configuration...\n");
					init_glide = false;
					ros::Duration(1).sleep(); //To make sure singular is ON
					data<<"Initializing to Singular Configuration Complete!"<<endl;
					printf("Initializing to Singular Configuration Complete!\n\n");
				}
				//Else(Not the first time in the loop),
				//Calculate motor angles with given dihedral and shoulder angles
				else
				{
					getq5();
					angles[0][0] = angle_LF; angles[0][1] = shoulder_F; angles[0][2] = q5_LF;
					angles[1][0] = angle_RF; angles[1][1] = shoulder_F; angles[1][2] = q5_RF;
					angles[2][0] = angle_LH; angles[2][1] = shoulder_H; angles[2][2] = q5_LH;
					angles[3][0] = angle_RH; angles[3][1] = shoulder_H; angles[3][2] = q5_RH;
					assignAngles(2);
				}			
				
				angle2PWM();				
				write2Motors();				

				if(logging)
					logGlide();				

			}//End of While
			printf("====================<Gliding Mode Complete>====================\n\n");
			data<<"====================<Gliding Mode Complete>====================\n"<<endl;
		}//End of Gliding Mode

		//Option 3: <Gait Mode>-----------------------------------------------------
		//This mode calculate foot trajectories
		//The trajectories will be converted to motor angles and saved in 2-D arrays
		//Each leg will have its own index to negivate in the 2-D array
		//Increasing or decreasing index by 1 for each loop,
		//The robot will follow the trajectories
		//The robot will be controlled by a keyboard
		//The keyboard interface will be printed when user enters this mode
		//-------------------------------------------------------------------------- 
		if (option == 3)
		{
			printf("====================<Gait Mode Started>====================\n");
			data<<"====================<Gait Mode Started>===================="<<endl;
			printf("v: Change Mode (Swag Mode <=> Gait Mode)\n");
			printf("z: Trot Mode\n");
			printf("x: Lateral Sequence Walk Mode\n");
			printf("c: Diagonal Sequence Walk Mode\n\n");
			printf("r: Speed Up\n");
			printf("f: Speed Down\n\n");
			printf("q: Increase Stride\n");
			printf("a: Decrease Stride\n\n");
			printf("w: Increase Height\n");
			printf("s: Decrease Height\n\n");
			printf("e: Increase Angle\n");
			printf("d: Decrease Angle\n\n");
			printf("t: Increase Stance Phase\n");
			printf("g: Decrease Stance Phase\n\n");
			printf("Arrow Up:    Increase Gear\n");
			printf("Arrow Down:  Decrease Gear\n");
			printf("Arrow Right: Turn Right\n");
			printf("Arrow Left:  Turn Left\n\n");
			printf(".: Print Current Status\n\n");
			printf("To terminate Gait Mode, press '/' key\n");
			
			//Initialize Variables
			walk = 0;
			idx_LF = 0; idx_RF = 0; idx_LH = 0; idx_RH = 0;
			w_L = 3; w_R = 3; h_L = 2; h_R = 2;
			res_gait = 50;
			ratio_gait = 0.5;
			angle_gait = 0;
			option = -1;
			
			//While '/' is pressed
			while(off == 0)
			{
				c = getch();
				handleKey_gait(c);
					
				//If gear == 1, move forward
				if(gear == 1)
				{
					++idx_LF;
					++idx_RF;
					++idx_LH;
					++idx_RH;				
				}
				//If gear == -1, move backward
				if(gear == -1)
				{
					--idx_LF;
					--idx_RF;
					--idx_LH;
					--idx_RH;
				}
				
				//Resolution Bound
				if(res_gait <= 0)
				{
					res_gait = 2;
					printf("Resolution cannot be less than or equal to 0\n");
				}				
				
				//Lower Bound for idx
				if(idx_LF < 0)
					idx_LF += res_gait;
				if(idx_RF < 0)
					idx_RF += res_gait;
				if(idx_LH < 0)
					idx_LH += res_gait;
				if(idx_RH < 0)
					idx_RH += res_gait;
					
				//Upper Bound for idx
				if(idx_LF >= res_gait)
					idx_LF -= res_gait;
				if(idx_RF >= res_gait)
					idx_RF -= res_gait;
				if(idx_LH >= res_gait)
					idx_LH -= res_gait;
				if(idx_RH >= res_gait)
					idx_RH -= res_gait;
				
				//Gait Mode
				if(walk == 0)
				{
					input_LF = gait(x_gait, y_gait, z_gait, w_L, h_L, angle_gait, res_gait, ratio_gait, 0);
					input_RF = gait(x_gait, y_gait, z_gait, w_R, h_R, angle_gait, res_gait, ratio_gait, 0);
					input_LH = gait(x_gait, y_gait, z_gait, w_L, h_L, angle_gait, res_gait, ratio_gait, 1);
					input_RH = gait(x_gait, y_gait, z_gait, w_R, h_R, angle_gait, res_gait, ratio_gait, 1);
				}
				//Stationary Mode
				if (walk == 1)
				{
					input_LF = fixedHeight(x_gait, y_gait + h_L, z_gait, w_L, -h_L, angle_gait, res_gait, ratio_gait, 0);
					input_RF = fixedHeight(x_gait, y_gait, z_gait, w_R, h_R, angle_gait, res_gait, ratio_gait, 0);
					input_LH = fixedHeight(x_gait, y_gait + h_L, z_gait, w_L, -h_L, angle_gait, res_gait, ratio_gait, 1);
					input_RH = fixedHeight(x_gait, y_gait, z_gait, w_R, h_R, angle_gait, res_gait, ratio_gait, 1);
				}
				
				//Assign angles
				angles[0][0] = input_LF[0][idx_LF]; angles[0][1] = input_LF[1][idx_LF]; angles[0][2] = input_LF[2][idx_LF];
				angles[1][0] = input_RF[0][idx_RF]; angles[1][1] = input_RF[1][idx_RF]; angles[1][2] = input_RF[2][idx_RF];
				angles[2][0] = input_LH[0][idx_LH]; angles[2][1] = input_LH[1][idx_LH]; angles[2][2] = input_LH[2][idx_LH];
				angles[3][0] = input_RH[0][idx_RH]; angles[3][1] = input_RH[1][idx_RH]; angles[3][2] = input_RH[2][idx_RH];
				assignAngles(1);

				angle2PWM();
				//printPWM();
				write2Motors();

				//Memory Cleaning
				for (int i = 0; i < 3; ++i)
				{
					delete[] input_LF[i];
					delete[] input_RF[i];
					delete[] input_LH[i];
					delete[] input_RH[i];
				}
				delete[] input_LF;
				delete[] input_RF;
				delete[] input_LH;
				delete[] input_RH;
			}//End of while off		
				
			printf("====================<Gait Mode Complete>====================\n\n");
			data<<"====================<Gait Mode Complete>====================\n"<<endl;
			
		}// End of Option6: Gait
		
		//Option 4: <Singular ON>-----------------------------------------
		//This mode place the legs very close to singular configuration
		//This is required for gliding mode
		//If the legs are not placed very close to singular configuration,
		//The robot will not have desired dihedral angles
		//----------------------------------------------------------------
		if(option == 4)
		{
			printf("<Singular ON Mode Selected>\n");
			data<<"<Singular ON Mode Selected>"<<endl;
			singularON();
			printPWM();
			option = -1;
			printf("<Singular ON Mode Complete>\n\n");
			data<<"<Singular ON Mode Complete>\n"<<endl;
		}
		
		//Option 5: <Singular OFF>----------------------------------------
		//This mode put the legs off of very close to singular configuration
		//This is required for smooth transition from gliding to gait
		//----------------------------------------------------------------
		if(option == 5)
		{
			printf("<Singular OFF Mode Selected>\n");
			data<<"<Singular OFF Mode Selected>"<<endl;
			singularOFF();
			printPWM();
			option = -1;
			printf("<Singular OFF Mode Complete>\n\n");
			data<<"<Singular OFF Mode Complete>\n"<<endl;
		}
		
		//Option 6: <Manual Gliding Mode>--------------------------------------------
		//This mode continuously reveives data from IMU and LIDAR
		//Based on the data, it controls the wing to glide safely
		//When the robot is about to land(determined by distance reading from LIDAR),
		//The robot will prepare for landing
		//---------------------------------------------------------------------------
		if (option == 6)
		{	
			//Print out to console and "data.txt"
			printf("====================<Gliding Mode Started>====================\n");
			data<<"=====================<Gliding Mode Started>===================="<<endl;
			printf("To get Current Status, press '.' Key\n");
			printf("To turn ON or OFF Log Mode, press 'l' Key (Initially OFF)\n");			
			printf("To terminate Gliding Mode, press '/' key\n\n");
			
			//Initialize variables	
			init_glide = true;
			launched = false;
			angle_LF = default_F;
			angle_RF = default_F;
			angle_LH = default_H;
			angle_RH = default_H;
			shoulder_F = 0;
			shoulder_H = 20;
			glideMode = 0;
			option = -1;
			
			//While '/' key is pressed
			while(off == 0)
			{	
				//Read Keyboard Input
				c = getch();
				
				//Handle keyboard input
				handleKey_glide(c);
					
				//IMU Update
				ros::spinOnce();
				vel.updateVel(acc_x, acc_y, acc_z);
				vel.transform(w_x, w_y, w_z);
				v = vel.getV();				
				
				//Lidar Update
				dist_m_prev = dist_m;
				dist = cos(pitch)*(_lidar.i2cRead() - 13);
				dist_m = (double)dist/100.0;				
								
				//Determine if the robot is launched
				if(!launched && acc_x >= launchAccTH && dist_m >= launchTH)
				{
					launched = true;
					status = 1;
					printf("Launched!\n");
					data<<"Launched!"<<endl;
				}
				
				//Manual Control
				if(glideMode == 0)
				{
					//Handle with handleKey_glide(char c)					
				}
				
				//PID Horizontal Control
				if(glideMode == 1)
				{					
					//PID Control
					inc_pitch = PID_pitch.calc(setpoint, pitch);
					inc_roll = PID_roll.calc(setpoint, roll);			

					//Calculate fore, hind, left & right using PID increment
					angle_fore = -pitch - inc_pitch;
					angle_hind = pitch + inc_pitch;
					angle_left = -roll - inc_roll;
					angle_right = roll + inc_roll;
					
					//Combine fore hind left right to calculate LF RF LH RH angles
					angle_LF = default_F + angle_fore;// + angle_left;
					angle_RF = default_F + angle_fore;// + angle_right;
					angle_LH = default_H + angle_hind;// + angle_left;
					angle_RH = default_H + angle_hind;// + angle_right;					
				}
				if(glideMode == 2)
				{
					printf("Perching Mode!\n");
					data<<"Perching Mode!"<<endl;
					angle_LF = perchingAngle;
					angle_RF = perchingAngle;
					angle_LH = -40;
					angle_RH = -40;					
				}							
				
				//Bounding angles			
				if(angle_LF >= glideAngleLimit) 
					angle_LF = glideAngleLimit;
				if(angle_RF >= glideAngleLimit) 
					angle_RF = glideAngleLimit;
				if(angle_LH >= glideAngleLimit) 
					angle_LH = glideAngleLimit;
				if(angle_RH >= glideAngleLimit) 
					angle_RH = glideAngleLimit;
				if(angle_LF <= -glideAngleLimit) 
					angle_LF = -glideAngleLimit;
				if(angle_RF <= -glideAngleLimit) 
					angle_RF = -glideAngleLimit;
				if(angle_LH <= -glideAngleLimit) 
					angle_LH = -glideAngleLimit;
				if(angle_RH <= -glideAngleLimit) 
					angle_RH = -glideAngleLimit;
							
				//< Landing Algorithm >
				// If it is launched && It is the first time to reach threshold height, activate Landing Mode
				if(launched && dist_m <= landTH && !landTrigger)
				{
					landTrigger = true;
					status = 2;
					angle_LF = perchingAngle;
					angle_RF = perchingAngle;
					angle_LH = -40;
					angle_RH = -40;
					data<<"Landing Mode ON!"<<endl;
					printf("Landing Mode ON!\n");
				}
				
				// If Landing Mode is activated & height is increasing, increase count
				if(landTrigger && dist_m_prev <= dist_m)
					++count_rise;
				
				// If Landing Mode is activated & current height is less than threshold
				// && count_rise is greater than threshold(robot was rising for a while),
				// Then decrease shoulder angle to increase lift force 
				if(landTrigger && dist_m <= 0.5 && count_rise >= countTH)
				{
					status = 3;
					data<<"Rise Detected, Changing Shoulder Angle to '0'"<<endl;
					printf("Rise Detected, Changing Shoulder Angle to '0'\n");
				}
				
				//If it is Landing Mode & close to land, raise legs to protect the legs
				if(landTrigger && dist_m <= close2landTH)
				{
					status = 4;
					angle_LF = 30;
					angle_RF = 30;
					angle_LH = 30;
					angle_RH = 30;
				}
				
				//If it is Landing Mode & landed
				if(landTrigger && dist_m <= landedTH)
				{
					status = 5;
					data<<"Landed"<<endl;
					printf("Landed\n");
					data<<"Preparing for Gait..."<<endl;
					printf("Preparing for Gait...\n");
					ros::Duration(2).sleep();
					data<<"Singular OFF..."<<endl;
					printf("Singular OFF...\n");
					singularOFF();
					ros::Duration(1).sleep();
					data<<"Entering to Gait Mode..."<<endl;
					printf("Entering to Gait Mode...\n");
					option = 3;
					break;
				}
				
				//If it is the first time in the loop, prepare for gliding				
				if(init_glide)
				{
					singularON();
					data<<"Initializing to Singular Configuration..."<<endl;
					printf("Initializing to Singular Configuration...\n");
					init_glide = false;
					ros::Duration(0.5).sleep(); //To make sure singular is ON
					data<<"Initializing to Singular Configuration Complete!"<<endl;
					printf("Initializing to Singular Configuration Complete!\n\n");
				}
				//Else(Not the first time in the loop),
				//Calculate motor angles with given dihedral and shoulder angles
				else
				{
					getq5();
					angles[0][0] = angle_LF; angles[0][1] = shoulder_F; angles[0][2] = q5_LF;
					angles[1][0] = angle_RF; angles[1][1] = shoulder_F; angles[1][2] = q5_RF;
					angles[2][0] = angle_LH; angles[2][1] = shoulder_H; angles[2][2] = q5_LH;
					angles[3][0] = angle_RH; angles[3][1] = shoulder_H; angles[3][2] = q5_RH;
					assignAngles(2);
				}			
				
				angle2PWM();				
				write2Motors();				

				if(logging)
					logGlide();				

			}//End of While
			printf("====================<Gliding Mode Complete>====================\n\n");
			data<<"====================<Gliding Mode Complete>====================\n"<<endl;
		}//End of Gliding Mode
		
		//PWM Frequency control
		//This mode changes PWM frequency sent to the servo motors
		if(option == 7)
		{
			cout<<"<PWM Frequency control Mode Selected>"<<endl;
			data<<"<PWM Frequency control Mode Selected>"<<endl;
			
			int pwmFreq = 0;
			cout<<"Frequency(Hz): "; cin >> pwmFreq;
			cout<<endl;
			pwmFreq = (int)pwmFreq;
			cout<<"Choosen Option: "<<pwmFreq<<endl;
			pwm.set_pwm_freq(pwmFreq);
			option = -1;
			
			cout<<"<PWM Frequency control Mode Complete>\n"<<endl;
			data<<"<PWM Frequency control Mode Complete>\n"<<endl;
		}
		
		// GPIO control
		if (LED_ON == 0) 
			LED_ON = 1;
		else
			LED_ON = 0;
		digitalWrite(pinNumber, LED_ON);
		
		//Wait
		rate.sleep();
	}
}
