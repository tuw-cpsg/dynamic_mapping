#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEY_SPACE 0x20
#define KEY_SPEED_INCREASE 97	//a
#define KEY_SPEED_DECREASE 115	//s

using geometry_msgs::Twist;
using namespace std;


ros::Publisher chatter_pub;
ros::Time t1;
Twist vel;
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	chatter_pub = n.advertise<Twist>("/cmd_vel", 1);  
	signal(SIGINT,quit);
	char c;
	int speed = 1;
	char state = 0;
	bool dirty=false;
	t1=ros::Time::now();

	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	while (ros::ok())
	{
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		switch(c)
		{
		case KEYCODE_L:
			ROS_DEBUG("LEFT");
			puts("TURN LEFT");
			vel.angular.z  = 0.1 * speed;
			dirty = true;
			break;

		case KEYCODE_R:
			ROS_DEBUG("RIGHT");
			puts("TURN RIGHT");
			vel.angular.z  = -0.1 * speed;
			dirty = true;
			break;

		case KEYCODE_U:
			ROS_DEBUG("UP");
			puts("FORWARD");
			vel.linear.x = 0.1 * speed;
			vel.angular.z  = 0;
			dirty = true;
			break;

		case KEYCODE_D:
			ROS_DEBUG("DOWN");
			puts("BACKWARD");
			vel.linear.x = -0.1 * speed;
			vel.angular.z  = 0;
			dirty = true;
			break;

		case KEY_SPACE:
			ROS_DEBUG("STOP");
			puts("SPACE");
			vel.linear.x = 0.0;
			vel.angular.z = 0.0;
			dirty = true;
			break;

		case KEY_SPEED_INCREASE:   
			ROS_DEBUG("F");      	
			if(speed < 10)	{
				puts("FASTER");
				speed++;
			}
			else	{
				puts("MAX SPEED");
			}
			break;

		case KEY_SPEED_DECREASE:
			ROS_DEBUG("F");
			if(speed > 2)	{
				puts("SLOWER");
				speed--;
			}
			else	{
				puts("MIN SPEED");
			}
			break;
		}
		vel.linear.y=0;
		vel.linear.z=0;
		vel.angular.x = 0;
		vel.angular.y = 0;
		if(dirty==true)  {
			chatter_pub.publish(vel);
			dirty=false;
		}
		ros::spinOnce();
	}//while

	return(0);
}
