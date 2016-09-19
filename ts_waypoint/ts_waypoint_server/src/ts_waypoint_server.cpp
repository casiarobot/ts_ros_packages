#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include "jsk_rviz_plugins/OverlayText.h"

std::string wp;
std::vector<std::vector<double> > waypoints;
interactive_markers::InteractiveMarkerServer *server;
//visualization_msgs::InteractiveMarker *int_marker;
visualization_msgs::InteractiveMarker change_marker;
int wp_no = 0;
int gn;

ros::Publisher waypoint_pub;

class waypoint_server
{
	public:
	ros::NodeHandle n;
	ros::Publisher init_pub,wp_no_pub,jsk_text_pub;
	ros::Subscriber result_sub;

	waypoint_server(){
		init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
		waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
		wp_no_pub = n.advertise<std_msgs::Int32>("/waypoint_num", 1);
		jsk_text_pub = n.advertise<jsk_rviz_plugins::OverlayText>("/text",1);
		result_sub = n.subscribe("/move_base/result", 1, &waypoint_server::result_callback, this);

		sleep(6);
		geometry_msgs::PoseWithCovarianceStamped initial_pose;
		initial_pose.header.frame_id = "/map";
		initial_pose.header.stamp = ros::Time::now();
		initial_pose.pose.pose.position.x = waypoints[wp_no][0];
		initial_pose.pose.pose.position.y = waypoints[wp_no][1];
		initial_pose.pose.pose.position.z = waypoints[wp_no][2];
		initial_pose.pose.pose.orientation.x = waypoints[wp_no][3];
		initial_pose.pose.pose.orientation.y = waypoints[wp_no][4];
		initial_pose.pose.pose.orientation.z = waypoints[wp_no][5];
		initial_pose.pose.pose.orientation.w = waypoints[wp_no][6];
		initial_pose.pose.covariance[0] = 0.25;
		initial_pose.pose.covariance[7] = 0.25;
		initial_pose.pose.covariance[35] = 0.06853891945200942;

		init_pub.publish(initial_pose);
		wp_no++;
		sleep(6);
		publish_wp();
		ROS_INFO("Published first Waypoint");
	}

	void publish_wp(){
		geometry_msgs::PoseStamped waypoint;
		waypoint.header.frame_id = "/map";
		waypoint.header.stamp = ros::Time::now(); 
		waypoint.pose.position.x = waypoints[wp_no][0];
		waypoint.pose.position.y = waypoints[wp_no][1];
		waypoint.pose.position.z = waypoints[wp_no][2];
		waypoint.pose.orientation.x = waypoints[wp_no][3];
		waypoint.pose.orientation.y = waypoints[wp_no][4];
		waypoint.pose.orientation.z = waypoints[wp_no][5];
		waypoint.pose.orientation.w = waypoints[wp_no][6];
		waypoint_pub.publish(waypoint);
		publish_text(wp_no-1);
		wp_no++;
	}

	void publish_text(int wn){
		jsk_rviz_plugins::OverlayText text;
		text.width = 140;
		text.height = 60;
		text.left = 20;
		text.top = 20;
		text.text_size = 20;
		text.line_width = 2;
		text.font = "DejaVu Sans Mono";
		std::stringstream ss;
		ss << "Waypoint " << wn << "/" << gn;
		text.text = ss.str();
		text.fg_color.r = 25/255.0;
		text.fg_color.g = 1.0;
		text.fg_color.b = 240/255.0;
		text.fg_color.a = 1.0;
		text.bg_color.a = 0.5;
		jsk_text_pub.publish(text);
	}

	void result_callback(const move_base_msgs::MoveBaseActionResult & result){

		if(result.status.status==3 ){
			if(wp_no!=gn){
				std_msgs::Int32 wp_num;
				geometry_msgs::PoseStamped waypoint;
				waypoint.header.frame_id = "/map";
				waypoint.header.stamp = ros::Time::now(); 
				waypoint.pose.position.x = waypoints[wp_no][0];
				waypoint.pose.position.y = waypoints[wp_no][1];
				waypoint.pose.position.z = waypoints[wp_no][2];
				waypoint.pose.orientation.x = waypoints[wp_no][3];
				waypoint.pose.orientation.y = waypoints[wp_no][4];
				waypoint.pose.orientation.z = waypoints[wp_no][5];
				waypoint.pose.orientation.w = waypoints[wp_no][6];
				waypoint_pub.publish(waypoint);
				wp_num.data = wp_no;
				wp_no_pub.publish(wp_num);
				std::stringstream ss;
				ss << "Waypoint No." << wp_no-1;
				server->erase(ss.str());
				server->applyChanges();
				ROS_INFO("Update Waypoint No.%d",wp_no);
				publish_text(wp_no-1);
				wp_no++;
			}
			else{
				ROS_INFO("GOAL!");
				jsk_rviz_plugins::OverlayText text;
				text.width = 140;
				text.height = 60;
				text.left = 20;
				text.top = 20;
				text.text_size = 20;
				text.line_width = 2;
				text.font = "DejaVu Sans Mono";
				text.text = "GOAL!";
				text.fg_color.r = 240/255.0;
				text.fg_color.g = 1.0;
				text.fg_color.b = 25/255.0;
				text.fg_color.a = 1.0;
				text.bg_color.a = 0.5;
			}
		}
		return;
	}

};

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
	int mn;
	sscanf(feedback->marker_name.c_str(),"Waypoint No.%d",&mn);
	for(int i = 0;i < mn;i++){
		std::stringstream ss;
		ss << "Waypoint No." << i;
		server->erase(ss.str());
		server->applyChanges();
	}
	ROS_INFO("Skipped Waypoint %d -> %d",wp_no,mn);
	wp_no = mn;
	geometry_msgs::PoseStamped waypoint;
	waypoint.header.frame_id = "/map";
	waypoint.header.stamp = ros::Time::now(); 
	waypoint.pose.position.x = waypoints[wp_no][0];
	waypoint.pose.position.y = waypoints[wp_no][1];
	waypoint.pose.position.z = waypoints[wp_no][2];
	waypoint.pose.orientation.x = waypoints[wp_no][3];
	waypoint.pose.orientation.y = waypoints[wp_no][4];
	waypoint.pose.orientation.z = waypoints[wp_no][5];
	waypoint.pose.orientation.w = waypoints[wp_no][6];
	waypoint_pub.publish(waypoint);
	wp_no++;
}

void new_wp(double px,double py,double pz,double qx,double qy,double qz,double qw,int wp){
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.pose.position.x = px;
	int_marker.pose.position.y = py;
	int_marker.pose.position.z = pz;
	int_marker.pose.orientation.w = qw;
	int_marker.pose.orientation.x = qx;
	int_marker.pose.orientation.y = qy;
	int_marker.pose.orientation.z = qz;
	std::stringstream ss;
	ss << "Waypoint No." << wp;
	int_marker.name = ss.str().c_str();
//	int_marker.description = ss.str().c_str();
	int_marker.header.frame_id = "map";

	visualization_msgs::Marker circle_marker;
	circle_marker.type = visualization_msgs::Marker::CYLINDER;
	circle_marker.scale.x = 1.0;
	circle_marker.scale.y = 1.0;
	circle_marker.scale.z = 0.01;
	circle_marker.color.r = 1.0;
	circle_marker.color.g = 0.0;
	circle_marker.color.b = 0.0;
	circle_marker.color.a = 1.0;

	visualization_msgs::InteractiveMarkerControl control;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	control.always_visible = true;
	control.markers.push_back(circle_marker);
	int_marker.controls.push_back(control);
	change_marker = int_marker;
	server->insert(int_marker, &processFeedback);	
	server->applyChanges();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ts_waypoint_server");
	if(argc == 2){
		wp = argv[1];
		if(wp.find(".wp")==std::string::npos){
			ROS_ERROR("Use .wp file");
			return 0;
		}
	}
	else{
		ROS_ERROR("Ah~ this is shit!");
		return 0;
	}

	ROS_INFO("waypoints file : %s",wp.c_str());

	server = new interactive_markers::InteractiveMarkerServer("WayPoint");
	if(FILE* fp = fopen(wp.c_str(), "r")){
		double px,py,pz,qx,qy,qz,qw;
		fscanf(fp,"Number Of Goals :%d\n",&gn);
		waypoints = std::vector<std::vector<double> >(gn, std::vector<double>(7, 0));
		for(int i = 0;i < gn;i++){
			fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&px,&py,&pz,&qx,&qy,&qz,&qw);
			waypoints[i][0] = px;
			waypoints[i][1] = py;
			waypoints[i][2] = pz;
			waypoints[i][3] = qx;
			waypoints[i][4] = qy;
			waypoints[i][5] = qz;
			waypoints[i][6] = qw;
			if(i>0){
				new_wp(px,py,pz,qx,qy,qz,qw,i);
			}
		}
		fclose(fp);
		ROS_INFO("Loaded %d Waypoints",gn);
	}
	else{
		ROS_ERROR("Can't open waypoint file.");
		return 0;
	}

	waypoint_server ws;
	ros::spin();

	return 0;
}

