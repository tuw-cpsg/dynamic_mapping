#include "ros/ros.h"
#include "std_msgs/String.h"

#include <signal.h>
#include <termios.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"
#include "ROSARIA/BumperState.h"

#define NUMBER_OF_BUMPERS 5
#define NUMBER_OF_SONAR 8

#define LASER_CHECK_FROM 160
#define LASER_CHECK_UNTIL 400
#define LASER_ALERT_RANGE 0.5
#define LASER_SLOW_RANGE 1.0
#define LASER_DELTA 5
#define SLOWDOWN_FACTOR 0.3

using geometry_msgs::Twist;
using namespace std;

Twist vel;
int stopBackward, stopForward, stopForwardSonar, stopForwardLaser, reduceSpeed = 0;

void controlCallback(const geometry_msgs::TwistConstPtr &velocity)
{
	printf("Linear: %f\n", (double) velocity->linear.x);
	printf("Angular: %f\n", (double) velocity->angular.z);
	printf("------------\n");
	vel.linear.x = velocity->linear.x;
	vel.linear.y = velocity->linear.y;
	vel.linear.z = velocity->linear.z;
	vel.angular.x = velocity->angular.x;
	vel.angular.y = velocity->angular.y;
	vel.angular.z = velocity->angular.z;
}

void bumperCallback(const ROSARIA::BumperStateConstPtr &bstate)
{
	int count = 0;
	for(int i = 0; i < NUMBER_OF_BUMPERS; i++)	{
		if((bool) bstate->front_bumpers[i] == 1)	{
			printf("Collision Front Bumper[%d]\n",i);
			stopForward = 1;
		}
		else	{
			count++;
		}
	}
	if(count == NUMBER_OF_BUMPERS)	{
		stopForward = 0;
	}

	count = 0;
	for(int i = 0; i < NUMBER_OF_BUMPERS; i++)	{
		if((bool) bstate->rear_bumpers[i] == 1)	{
			printf("Collision Rear Bumper[%d]\n",i);
			stopBackward = 1;
		}
		else	{
			count++;
		}
	}
	if(count == NUMBER_OF_BUMPERS)	{
		stopBackward = 0;
	}

}

void sonarCallback(const sensor_msgs::PointCloudConstPtr &pcloud)
{
	int count = 0;
	for(int i = 0; i < NUMBER_OF_SONAR; i++)	{
		if((bool) pcloud->points[i].x < 0.05)	{
			printf("Collision Front Avoided [%d]\n",i);
			stopForwardSonar = 1;
		}
		else	{
			count++;
		}
	}
	if(count == NUMBER_OF_SONAR)	{
		stopForwardSonar = 0;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &lscan)
{
	int count = 0;
	int count_slow = 0;
	for(int i = LASER_CHECK_FROM; i < LASER_CHECK_UNTIL; i++)	{
		if((bool) (lscan->ranges[i] > LASER_ALERT_RANGE))	{
			count++;
		}
		if((bool) (lscan->ranges[i] > LASER_SLOW_RANGE))	{
			count_slow++;
		}
	}
	if(count < ((LASER_CHECK_UNTIL - LASER_CHECK_FROM) - LASER_DELTA))	{
		stopForwardLaser = 1;
		printf("Collision Front by Laser Avoided [%d]\n",count);
	}
	else	{
		stopForwardLaser = 0;
	}

	if(count_slow < ((LASER_CHECK_UNTIL - LASER_CHECK_FROM) - LASER_DELTA))	{
		reduceSpeed = 1;
		printf("Slow down because of possible collision [%d]\n",count);
	}
	else	{
		reduceSpeed = 0;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety_navigation");
	int use_laser;

	ros::NodeHandle n;
	ros::NodeHandle m;
	ros::NodeHandle nh("~");
	// command line parameter to use laser or not
	nh.param("use_laser", use_laser, 0);

	// keyboard control message
	//ros::Subscriber sub_control = n.subscribe("p3at_control", 1, controlCallback);
	ros::Subscriber sub_control = n.subscribe("cmd_vel", 1, controlCallback);	// 4.7. for navigation stack

	// bumper message
	// in MobileSim not available!)
	ros::Subscriber sub_bumper = m.subscribe("/RosAria/bumper_state", 1, bumperCallback);

	// sonar message
	// in MobileSim not available!)
	//ros::Subscriber sub_sonar = m.subscribe("/RosAria/sonar", 1, sonarCallback);

	// laser message
	ros::Subscriber sub_laser;
	if(use_laser)	{
		sub_laser = m.subscribe("/scan", 1, laserCallback);
	}

	ros::NodeHandle o;
	ros::Publisher chatter_pub = o.advertise<Twist>("/RosAria/cmd_vel", 1); 

	while (ros::ok()) {
		if((stopBackward && vel.linear.x < 0.01) || ((stopForward || stopForwardSonar || stopForwardLaser) && vel.linear.x > -0.01))	{
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.linear.z = 0;
			vel.angular.x = 0;
			vel.angular.y = 0;
			vel.angular.z = 0;
		}
		else if(reduceSpeed && vel.linear.x > 0.1)	{
			vel.linear.x = 0.1;
		}
		chatter_pub.publish(vel);
		ros::spinOnce();
	}

	return 0;
}
