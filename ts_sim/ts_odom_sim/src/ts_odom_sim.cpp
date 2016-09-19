#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#define trans_speed 1
#define rot_speed 1

bool first = true;
double yaw = 0;
ros::Time last_time;
nav_msgs::Odometry odom;
nav_msgs::Path path;

void vel_Callback(const geometry_msgs::TwistConstPtr& vel){
	ros::Duration d_time = odom.header.stamp - last_time;
	double d_trans = vel->linear.x * (d_time.toSec()) * trans_speed;
	double d_angul = vel->angular.z * (d_time.toSec()) * rot_speed;
	yaw += d_angul;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	odom.pose.pose.position.x += d_trans * cos(yaw);
	odom.pose.pose.position.y += d_trans * sin(yaw);
	odom.twist.twist.linear.x = vel->linear.x;
	odom.twist.twist.angular.z = vel->angular.z;
	geometry_msgs::PoseStamped ps;
	ps.pose = odom.pose.pose;
	path.poses.push_back(ps);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ts_odom_sim");
	ros::NodeHandle n;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path_odometry", 10);
	ros::Subscriber vel_sub = n.subscribe("/cmd_vels",1,vel_Callback);
	
	last_time = ros::Time::now();

	tf::TransformBroadcaster odom_tb;
	path.header.frame_id = odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	ros::Rate loop_rate(40);

	last_time = ros::Time::now();
	first = false;
	geometry_msgs::TransformStamped odom_trans;
	odom.header.stamp = ros::Time::now();
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	odom.pose.pose.position.x = 0;
	odom.pose.pose.position.y = 0;
	odom.pose.pose.position.z = 0;
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.angular.z = 0;
	odom_trans.header.stamp = odom.header.stamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = odom.child_frame_id;
	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
	odom_trans.transform.rotation = odom_quat;
	odom_tb.sendTransform(odom_trans);

	while(ros::ok())
	{
		last_time = odom.header.stamp;
		odom.header.stamp = ros::Time::now();
		ros::spinOnce();
		odom_pub.publish(odom);
		path_pub.publish(path);
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odom.header.frame_id ;
		odom_trans.child_frame_id = odom.child_frame_id;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_tb.sendTransform(odom_trans);
		odom.header.seq++;
		loop_rate.sleep();
	}
	return 0;
}
