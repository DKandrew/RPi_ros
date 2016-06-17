#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

using namespace std;

int main(int argc, char **argv) {
	//Initialize ROS system
	ros::init(argc, argv, "led_pub");
	ros::NodeHandle nh;
	//Create a publisher object
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("led/cmd_vel", 1000);
	//Frequency Control
	ros::Rate rate(2);
	//Ask for flashing rate
	cout << "How fast you want LED to blink?" << endl;
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
	
	
}

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
