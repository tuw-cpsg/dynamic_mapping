#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

#define PI 3.1415


tf::Transform transformMaps;
tf::Transform transformDynMaps;
tf::Transform transformUpdateArea;

void initPoseRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose)	{
	  transformMaps.setOrigin( tf::Vector3(init_pose->pose.pose.position.x, init_pose->pose.pose.position.y, init_pose->pose.pose.position.z) );
  	  transformMaps.setRotation( tf::Quaternion(init_pose->pose.pose.orientation.x, init_pose->pose.pose.orientation.y, init_pose->pose.pose.orientation.z, init_pose->pose.pose.orientation.w) ); 
	  printf("Initial position was set");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  ros::Rate r(30);	// set frequency to 30hz, without that we would have 800hz, which too much

  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformBroadcaster tf_broadcaster_dyn;
  tf::TransformBroadcaster tf_broadcaster_update;

  // subscribe to initial position topic
  ros::Subscriber init_pose = node.subscribe("initialpose", 1, initPoseRobotCallback);
  transformMaps.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transformMaps.setRotation( tf::Quaternion(0.0, 0.0, 0.0) ); 
  transformDynMaps.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transformDynMaps.setRotation( tf::Quaternion(0.0, 0.0, 0.0) ); 
  //transformUpdateArea.setOrigin( tf::Vector3(0.1, -0.1, 0.0) ); // determined by experiments
	transformUpdateArea.setOrigin( tf::Vector3(0.1, 0.1, 0.0) ); // determined by experiments
  transformUpdateArea.setRotation( tf::Quaternion( 0.0, PI, - PI / 2) );
	//transformUpdateArea.setRotation( tf::Quaternion( 0.0, 0, - PI / 2) );

  printf("Transform broadcaster STARTED\n");
  while (node.ok()){
	  tf_broadcaster_dyn.sendTransform(tf::StampedTransform(transformDynMaps, ros::Time::now(), "dyn_map", "map"));
	  tf_broadcaster.sendTransform(tf::StampedTransform(transformMaps, ros::Time::now(), "map", "slam_map"));
	  tf_broadcaster_update.sendTransform(tf::StampedTransform(transformUpdateArea, ros::Time::now(), "base_link", "update_area"));
	  ros::spinOnce();
	  r.sleep();
  }
  return 0;
};
