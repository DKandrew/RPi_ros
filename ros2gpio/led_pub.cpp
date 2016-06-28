#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <ros2gpio/led.h>  //include led.msg file 

using namespace std;

int main(int argc, char **argv) {
	//Initialize ROS system
	ros::init(argc, argv, "led_pub");
	ros::NodeHandle nh;
	//Create a publisher object
	ros::Publisher pub = nh.advertise<ros2gpio::led>("led/control", 1000);
	//Frequency Control
	ros::Rate rate(8);
	//Ask user for frequency
	int freq1, freq2, freq3, freq4;
	cout << "how fast do you want the first led to blink? " << endl;
	cin >> freq1;
	cout << "how fast do you want the second led to blink? " << endl;
	cin >> freq2;
	cout << "how fast do you want the third led to blink? " << endl;
	cin >> freq3;
	cout << "how fast do you want the forth led to blink? " << endl;
	cin >> freq4;
	int cntr1, cntr2, cntr3, cntr4;
	cntr1=0;cntr2=0;cntr3=0;cntr4=0;
	//Signal
	int high=1; int low=0;
	//Main
	while(ros::ok()){
		//Create led control message
		ros2gpio::led msg;
		if(cntr1%freq1==0){
			msg.LED1 = low;
			cntr1=0; 		//Clear control counter
		}else{
			msg.LED1 = high;
		}
		if(cntr2%freq2==0){
			msg.LED2 = low;
			cntr2=0; 		//Clear control counter
		}else{
			msg.LED2 = high;
		}
		if(cntr3%freq3==0){
			msg.LED3 = low;
			cntr3=0; 		//Clear control counter
		}else{
			msg.LED3 = high;
		}
		if(cntr4%freq4==0){
			msg.LED4 = low;
			cntr4=0; 		//Clear control counter
		}else{
			msg.LED4 = high;
		}
		//Update control counter
		cntr1++;
		cntr2++;
		cntr3++;
		cntr4++;
		//Publish the message.
		pub.publish(msg);
		//Send a message to rosout with details
		ROS_INFO_STREAM("Sending message of LED blinking command");
		//Wait
		rate.sleep(); 
	}
}	
	
	//Ask for flashing rate
	/*cout << "How fast you want LED to blink?" << endl;
	int freq;
	cin >> freq;

	int i = 1;
	while(ros::ok()){
		//Create message and fill this out
		geometry_msgs::Twist msg;
		if(i == freq){
			msg.linear.x = double(0);
		}else{
			msg.linear.x = double(1);
		}
		//Publish the message.
		pub.publish(msg);
		//Send a message to rosout with details
		ROS_INFO_STREAM("Sending message of LED blinking command");
		//Update i
		i = i % freq + 1; 
		//Wait
		rate.sleep(); 
	}
	*/


//Trash
/*
 * 
 * char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}
 * 
 *
 *
 */ 
