#include <ros/ros.h>
#include "servo_controller.h"
#include <wiringPiSPI.h>

int modeFlag = 0; 			// if modeFlag == 0, it is in Wheel mode; if modeFlag == 1 it is in Joint mode
int DEBUG = 1;				// Debug flag
int READ_DELAY_TIME = 200;	// By default, the servo delay 0.5 msec to send the status packet after it receives inst packet. 
							// But the acutal delay time can be modified if you changed the Return Delay Time register
							// 100 is picked by experiment. 50 will fail.
int currPos = 0;
int isFail = 0;
int expect = 0;

void printData(char *data, int len){
	for(int i=0; i<len; i++){
		printf("%x ", data[i]);
	}
	printf("\n");
}

/*
 * Check if the data is a valid. Return -1 if the data is invalid like checksum error or no 0xff, or the length is 
 * Else, return the position where the data starts.
 */
int checkDataIntegrity(char *data, int len){
	// Find 0xFF 0xFF
	
	// Find the length of the data
	
	// Check checksum
	return -1;
}

/*	This function will write data into the dynamicxel servo. 
 * 	There are some delay after the swtich is turned on and when write() is called.
 * 	The time is manually picked. The delay time after write() cannot be zero otherwise we fail to read status packet from servo.
 */
int writeData(int fd, char *data, int dataLen){
	// Enable wirte
	digitalWrite(SWITCH, SWITCH_ON);
	delayMicroseconds(50);			//Delay
	ssize_t result = -1;
	while(result == -1){
		result = write(fd, data, dataLen);	//Write
	}
	
	// For 1M Baud rate, sending 1 byte is about 0.01ms. But considering the digitalWrite may run faster than write(), we set delay = 10*datalen
	delayMicroseconds(100);			// The delay timing is picked by experiment... 		
	// Disable wirte
	digitalWrite(SWITCH, SWITCH_OFF);
	
	return 0;
}

/* This function will read the data from servo into buffer. Because it uses the first 0xff in the status packet, 
 * The returned buffer will only the rest of the information (i.e. ignored the first 0xff)
 * buffer is pre-allocated. User should use delete [] after calling this function. len is the total length of the buffer. 
 */
int readData(int fd, char* buffer, int len){

	int reading_len = len - 1; 		// The last byte of buffer is reserved for '\0'
	int test;
	int result = read(fd, (void*)buffer, reading_len);
	buffer[len - 1] = '\0';
	
	return result;
	/*
	for(int i = 0; i < reading_len; i++){
		printf("%x ", buffer[i]);
	}
	cout << endl;
	cin >> test;
	//if(test == 1) return;
	while(buffer[0] != 0xFF || buffer[1] != 0xFF){
		//cout << "PASS" << endl;
		
		cout << read(fd, (void*)buffer, reading_len) << endl;
		for(int i = 0; i < reading_len; i++){
			printf("%x ", buffer[i]);
		}
		cout << endl;
		cin >> test;
	}
	if(DEBUG == 1){
		printf("0xff, oxff detected, next: ");
		int i = 0;
		while(i < reading_len){
			printf("%x ", buffer[i]);
			i++;
		}
		printf("\n");
	}
	// Add '\0' at the last byte of buffer
	buffer[len - 1] = '\0';*/

/*
	// Read Status Packet 
	int fa=0; 				//Flags for detecting FF signal
	char temp[2];
	
	// Read the first two bytes and check if it is 0xFF
	read(fd, (void*)temp, 2);
	if(temp[0] == 0xFF && temp[1] == 0xFF){
		fa = 1;
	}
	
	if(fa){
		buffer[0] = temp[0];
		buffer[1] = temp[1];
		
		int i = 0;
		int dataLen = len - 3;	//Why len - 3 ??
		read(fd, (void*)(buffer + 2), dataLen);
		
		//Debug
		if(DEBUG == 1){
			printf("0xff, oxff detected, next: ");
			i = 0;
			while(i<dataLen){
				printf("%x ", buffer[i+2]);
				i++;
			}
			printf("\n");
		}
		
		// Add '\0' at the last byte of buffer
		buffer[len - 1] = '\0';
	}
*/
}

int readRegister(int fd, int id, int inst){
	int rpl = 0;	// Read Parameter Length
	switch (inst){
		case AX_MODEL_NUMBER_L: rpl = 2; break;
		case AX_VERSION: 		rpl = 1; break;
		case AX_ID: 			rpl = 1; break;
		case AX_BAUD_RATE: 		rpl = 1; break;
		case AX_RETURN_DELAY_TIME: 	rpl = 1; break;
		case AX_CW_ANGLE_LIMIT_L: 	rpl = 2; break;
		case AX_CCW_ANGLE_LIMIT_L: 	rpl = 2; break;
		case AX_HIGHEST_LIMIT_TEMPERATURE: 	rpl = 1; break;
		case AX_LOWEST_LIMIT_VOLTAGE: 		rpl = 1; break;
		case AX_HIGHEST_LIMIT_VOLTAGE: 		rpl = 1; break;
		case AX_MAX_TORQUE_L: 	rpl = 2; break;
		case AX_RETURN_LEVEL: 	rpl = 1; break;
		case AX_ALARM_LED: 		rpl = 1; break;
		case AX_ALARM_SHUTDOWN: rpl = 1; break;
		case AX_TORQUE_ENABLE: 	rpl = 1; break;
		case AX_LED_ENABLE: 	rpl = 1; break;
		case AX_CW_COMPLIANCE_MARGIN: 	rpl = 1; break;
		case AX_CCW_COMPLIANCE_MARGIN: 	rpl = 1; break;
		case AX_CW_COMPLIANCE_SLOPE: 	rpl = 1; break;
		case AX_CCW_COMPLIANCE_SLOPE: 	rpl = 1; break;
		case AX_GOAL_POSITION_L: 	rpl = 2; break;
		case AX_MOVE_SPEED_L: 		rpl = 2; break;
		case AX_TORQUE_LIMIT_L: 	rpl = 2; break;
		case AX_PRESENT_POSITION_L: rpl = 2; break;
		case AX_PRESENT_SPEED_L: 	rpl = 2; break;
		case AX_PRESENT_LOAD_L: 	rpl = 2; break;
		case AX_PRESENT_VOLTAGE: 	rpl = 1; break;
		case AX_PRESENT_TEMPERATURE: 	rpl = 1; break;
		case AX_REGISTERED_INSTRUCTION: rpl = 1; break;
		case AX_MOVING: 	rpl = 1; break;
		case AX_LOCK: 		rpl = 1; break;
		case AX_PUNCH_L: 	rpl = 2; break;
		default: break;	
	}
	
	// Write data 
	int len = 4;
	int checksum = ~(id + len + AX_READ_DATA + inst + rpl) &  0xff;
	int dataLen = len + 4;
	int readLen = 7 + rpl;		// Status packet will return "FF FF ID LEN ERROR [rpl] CHECKSUM \0" (\0 means the end of a string), so readlen should be 6 bytes (everything except parameters) + rpl
	char* buffer = new char[readLen];
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_READ_DATA, inst, rpl, checksum};

	int suc = 0;				// Keep track of how many data we successfully read in one read(). It is used because read() sometime fails to read the packet at one time. It needs read() twice. 
	while(suc != readLen-1 || buffer[0]!=0xff || buffer[1]!=0xff){		//Check if the readData return the correct length.

		writeData(fd, data, dataLen);
		delay(100);				// When finish writeData() we need delay because the computer's speed is way faster than the servo. If there is no delay, we may just read a bunch of 0s. The delay time 100 here is picked by experiment.
		suc = readData(fd, buffer, readLen);	
	}

	// Find the result in the buffer
	int result = 0;						
	if(*buffer == 0xff){
		int index = 5; 			// Skip id, len, error in status packet which is 4 bytes in total.
		if (rpl > 1){
			int lowbyte = buffer[index];
			int highbyte = buffer[index + 1]; 
			result = highbyte << 8 | lowbyte;
		}else {
			result = buffer[index];
		}
		isFail = 0;
	}else{
		isFail = 1;				//printf("Fail to read register.\n");
	}
	
	// Delete buffer
	delete [] buffer; 
	
	// Return result
	return result;
}

/*
 * Read the status packet after setting functions. The status packet is always 0xff 0xff 0x1 0x2 ERROR Checksum
 * Return the ERROR information
 * Note:	We think you have to read the status packet every time you send a instruction packet. 
 * 			Otherwise, the status packet will remain on the bus and corrupte the future reading.
 */
int readStatusPacket(int fd){
	return 1;
	/*
	int len = 7;
	char *response = new char[len];
	int read_len = readData(fd, response, len);	// Because readData() may not read a full length of data at once. For example 
												// we should expect status packet 0xff 0xff 0x1 0x2 Error Checksum. But the first readData() only read the first 4 bytes 0xff 0xff 0x1 0x2. 
												// We find that the remaining 2 bytes is being read in the next readData(). Therefore, we do readData() twice here in order to read a full length of data. 
												// Failing to read a full length of data will corrupt all the future readData() because it will contain data from previous status packet. 
	int count = 0;
	while(read_len < len-1){
		read_len = read_len + readData(fd, response+read_len, len-read_len);
		//cout << "here" << endl;
		count++;
		if(count == 2) break;
	}
	

	int retval = 0;
	retval = response[4];
	// Delete response
	delete [] response;
	
	return retval;*/
}

//-----------------------------------Setting Function-----------------------------------
// Enable the torque. In the wheel mode however, the torque enable register will be automatically set to 1.
void torqueEnable(int fd, int id, int enable){
	int len = 4;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_TORQUE_ENABLE + enable) & 0xff;
	int dataLen = len + 4;
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_TORQUE_ENABLE, enable, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);
	
}

// Wheel Mode is the mode where servo can have torque control
void setToWheelMode(int fd, int id){
	int len = 7;					// Data length is 7
	int checksum = ~(id + len + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L) & 0xff;  // Set both cwLimit and ccwLimit to zero
	int dataLen = len + 4;			// 4 represnets the length of FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, 0, 0, 0, 0, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);

	modeFlag = 0;		// Update modeFlag
}


/* Joint Mode is the mode where servo can have position control.
   cwLimit is the min value of Goal position. ccwLimit is the max value of Goal position.
*/
void setToJointMode(int fd, int id, int cwLimit, int ccwLimit){
	if (cwLimit < 0) cwLimit = 0;		// min value of cwLimit is 0
	if (ccwLimit > 1023) ccwLimit = ccwLimit % 1024; // max value of ccwLimit is 1023
	
	// Set the lower & upper bytes of cwLimit and ccwLimit
	int cwLimit_l = cwLimit & 0xff;
	int cwLimit_h = cwLimit >> 8;
	int ccwLimit_l = ccwLimit & 0xff;
	int ccwLimit_h = ccwLimit >> 8;
	
	int len = 7;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + cwLimit_l + cwLimit_h + ccwLimit_l + ccwLimit_h) & 0xff; 
	int dataLen = len + 4;			// 4 represnets the length of FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, cwLimit_l, cwLimit_h, ccwLimit_l, ccwLimit_h, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);
	modeFlag = 1;		// Update modeFlag
	return ;
}

void setPosition(int fd, int id, int position){
	//position = position % 1024;		// The range of AX servo position is from 0 (0x0) to 1023(0x3ff). 
	int posi_l = position & 0xff;	// Lower 8 bits of position 
	int posi_h = position >> 8;		// Upper 8 bits of position 
	int len = 5;					// Data length is 5
	int checksum = ~(id + len + AX_WRITE_DATA + AX_GOAL_POSITION_L + posi_l + posi_h) & 0xff; // First NOT the sum, then truncate it to 1 bytes, i.e. AND 0xFF
	int dataLen = 9; 				// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_GOAL_POSITION_L, posi_l, posi_h, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);

}

// This function will set the torque servo produces, i.e. Torque control
void setTorqueLimit(int fd, int id, int torque){
	//torque = torque % 1024;					// The range of torque is from 0 (0x0) to 1023(0x3ff). 
	int torque_limit_l = torque & 0xff;		// Lower 8 bits of torque limit
	int torque_limit_h = torque >> 8;		// Upper 8 bits of torque limit
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_TORQUE_LIMIT_L + torque_limit_l + torque_limit_h) & 0xff;
	int dataLen = 9; 						// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_TORQUE_LIMIT_L, torque_limit_l, torque_limit_h, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);
}

/*	This function will set the moving speed of the servo. Speed in Wheel mode and Joint mode are different
 * 	In the Joint mode, the range of speed is from 0~1023(0x3ff). The unit is about 0.111 rpm. At 1023, the (max) speed is 114 rpm
 * 	In the Wheel mode, the range of speed is from 0~2047(0x7ff). Range 0~1023 is rotating in CCW directions; Range 1024~2047 is rotating in CW directions; 
 */
void setMovingSpeed(int fd, int id, int speed){
	if(modeFlag == 0) speed %= 2048;		// If in wheel mode, speed limit is 2047.
	if(modeFlag == 1) speed %= 1024;		// If in joint mode, speed limit is 1023.
	
	int moving_speed_l = speed & 0xff; 		// Lower 8 bits of moving speed
	int moving_speed_h = speed >> 8;		// Upper 8 bits of moving speed
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_MOVE_SPEED_L + moving_speed_l + moving_speed_h) & 0xff;
	int dataLen = 9; 						// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_MOVE_SPEED_L, moving_speed_l, moving_speed_h, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);
}

void setMaxTorque(int fd, int id, int torque){
	int torque_limit_l = torque & 0xff;		// Lower 8 bits of torque limit
	int torque_limit_h = torque >> 8;		// Upper 8 bits of torque limit
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_MAX_TORQUE_L + torque_limit_l + torque_limit_h) & 0xff;
	int dataLen = len + 4; 		
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_MAX_TORQUE_L, torque_limit_l, torque_limit_h, checksum};
	writeData(fd, data, dataLen); 
	// Read Status Packet
	int return_error = readStatusPacket(fd);
}

void setReturnDelayTime(int fd, int id, int dalay_time){
	int len = 4;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + dalay_time) & 0xff;
	int dataLen = len + 4;
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_RETURN_DELAY_TIME, dalay_time, checksum};
	writeData(fd, data, dataLen);
	// Read Status Packet
	int return_error = readStatusPacket(fd);
}

//-----------------------------------Reading Function-----------------------------------
int readPresentPosition(int fd, int id){
	return readRegister(fd, id, AX_PRESENT_POSITION_L);
}

//-----------------------------------Encoder Function-----------------------------------
int readEncoder(int channel){
	int dataLen = 2;
	unsigned char data[dataLen] = {0xff, 0xff};
	wiringPiSPIDataRW(channel, data, dataLen);
	int EncReading = (((unsigned int) (data[0] & 0x7f)) << 6) + (((unsigned int) (data[1] & 0xf8)) >> 2);
	return EncReading;
}

//-----------------------------------Main-----------------------------------
// To run: 
int main(int argc, char **argv){
	//Initialize ROS system
	ros::init(argc, argv, "servo_node");
	ros::NodeHandle nh;
	//Ros Rate
	int sampleRate;
	cout << "Please enter the period (in ms): ";
	cin >> sampleRate;
	double frequency = 1000/sampleRate;
	ros::Rate rate(frequency);
	
	//UART
	int baud = 1000000;				//wiringPi does not support 1M baud rate originally. You have to add this into the source code "wiringSerial.h" and re-build the wiringPi.
	int fd = serialOpen("/dev/ttyS0",baud);		
	if(( fd < 0 )){
		std::cout << "Open file failed " << fd << std::endl;
		return 0;
	}
	
	//Initialize GPIO switch
	wiringPiSetup();
	pinMode(SWITCH, OUTPUT);
	digitalWrite(SWITCH, SWITCH_ON);
	
	//Setup SPI for Encoder
	int channel = 0; // Pi has 2 channels: 0 and 1
	int spi_freq = 1000000;
	int spi_mode = 3; // IMU needs mode3. // Encoder works for all the modes. (in mode 0, it reads different value.)
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}

	//---Variable---
	int posi = 0;
	int id = 1;

	double step_len;
	int speed = 1023;
	int low_limit, hi_limit;
	int do_exit = 0;
	int do_convert = 0;
	
	double singleAng = 1024/300;			// convert from degree to the servo unit.

	// Set the servo return delay time	
	setReturnDelayTime(fd, id, 100);
	
	// When low_limit < hi_limit, the servo will rotate in the opposite direction 
	cout << "Please input the lowest limit of the servo(from 0 to 300 degree): ";
	cin >> low_limit;
	cout << "Please input the highest limit of the servo(from 0 to 300 degree): ";
	cin >> hi_limit;
	
	low_limit = (int)(low_limit * singleAng);
	hi_limit = (int)(hi_limit * singleAng);
	cout << "low_limit is " << low_limit << ", hi_limit is " << hi_limit << endl;
	
	if(low_limit > hi_limit) do_convert = 1;	// Find the direction the motor should move;
	else do_convert = 0;
	
	//Initializing
	cout << "Initialize to the low limit.........." << endl;
	if(do_convert){
		setToJointMode(fd, id, hi_limit, low_limit);
	}
	else{
		setToJointMode(fd, id, low_limit, hi_limit);
	}
	setPosition(fd, id, low_limit);
	setMovingSpeed(fd, id, 100);
	// zero the encoder and synchonize
	int encoderBase = readEncoder(channel);
	
	cout << "Finish initialize." << endl;
	
	// Get step length and speed
	/*		The smallest step length that will work is 0.5 degree. When the step length is 0.1 degree, it won't work when the period is less or equal to 50ms.		*/
	currPos = low_limit;
	cout << "Please input the step length you want(in degree): ";
	cin >> step_len;
	step_len *= singleAng;
	
	int target = 0;		// The goal position
	
	while(ros::ok()){
		if(!do_convert){
			target = currPos + step_len;
			if(target > hi_limit) target = hi_limit;
		}
		else{
			target = currPos - step_len;
			if(target < hi_limit) target = hi_limit;
		}
		//Encoder: Read current position 
		int EncChannel = 0;
		int EncReading = readEncoder(EncChannel);
		float EncAngle =((float)(EncReading - encoderBase )/8192)*(360);  
		cout <<"Encoder angle: "<< EncAngle << endl;		//debugging
		
		setPosition(fd, id, target);
		currPos = target;
		if(!do_convert && currPos >= hi_limit) break;
		else if(do_convert && currPos <= hi_limit) break;
		
		//Wait
		rate.sleep();
	}
	
	serialClose(fd);
	return 0;
}
