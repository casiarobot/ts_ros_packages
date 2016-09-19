//This node Plots the AMCL path on Rviz using Marker.
//frame_id is "map".
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include "visualization_msgs/Marker.h"


//visualization_msgs::Marker points,line_strip;
nav_msgs::Path path;

void callback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose){
//    ROS_INFO("OK");
//    geometry_msgs::Point p;
//    p.x = amcl_pose.pose.pose.position.x;
//    p.y = amcl_pose.pose.pose.position.y;
//    p.z = amcl_pose.pose.pose.position.z;
    geometry_msgs::PoseStamped ps;
    ps.pose = amcl_pose.pose.pose;
    path.poses.push_back(ps);
//    line_strip.points.push_back(p);
}

int main(int argc,char **argv){
    ros::init(argc ,argv ,"amcl_path_plotter");
    ros::NodeHandle n;
    ros::Subscriber pose_sub = n.subscribe("amcl_pose",1,callback);
//    ros::Publisher path_marker_pub = n.advertise<visualization_msgs::Marker>("amcl_path",1);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("amcl_path",1);
//    points.header.frame_id = line_strip.header.frame_id = "map";
//    points.ns = line_strip.ns = "amcl_path_publisher";
//    points.action = line_strip.action = visualization_msgs::Marker::ADD;
//    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

//    points.id = 0;
//    line_strip.id = 1;

//    points.type = visualization_msgs::Marker::POINTS;
//    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

//    points.scale.x=1;
//    points.scale.y=1;

////    line_strip.scale.x=0.2;
//    line_strip.scale.y=0.2;
//    line_strip.scale.z=0.2;

//    line_strip.color.b=1;
//    line_strip.color.a=1;
    path.header.frame_id = "map";
    ros::Rate r(20);

    while(ros::ok()){
//        points.header.stamp = line_strip.header.stamp = ros::Time::now();
//		path_marker_pub.publish(points);
//        path_marker_pub.publish(line_strip);
	path_pub.publish(path);
        r.sleep();
        ros::spinOnce();

    }
}
