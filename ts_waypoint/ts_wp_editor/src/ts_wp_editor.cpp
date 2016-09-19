#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PointStamped.h>
//#include <tf/transform_listener.h>

interactive_markers::InteractiveMarkerServer *server;
std::vector<std::vector<double> > waypoints;
std::string wp,savename;
int marker_no = 0;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
	int mn;
	sscanf(feedback->marker_name.c_str(),"Waypoint No.%d",&mn);
	waypoints[mn][0] = feedback->pose.position.x;
	waypoints[mn][1] = feedback->pose.position.y;
	waypoints[mn][2] = feedback->pose.position.z;
	waypoints[mn][3] = feedback->pose.orientation.x;
	waypoints[mn][4] = feedback->pose.orientation.y;
	waypoints[mn][5] = feedback->pose.orientation.z;
	waypoints[mn][6] = feedback->pose.orientation.w;
	ROS_INFO("Changed WayPoint No.%d Pose",mn);
	server->applyChanges();
}

void new_marker(double px,double py,double pz,double qx,double qy,double qz,double qw){
	visualization_msgs::InteractiveMarker int_marker;

	int_marker.pose.position.x = px;
	int_marker.pose.position.y = py;
	int_marker.pose.position.z = pz;
	int_marker.pose.orientation.w = qw;
	int_marker.pose.orientation.x = qx;
	int_marker.pose.orientation.y = qy;
	int_marker.pose.orientation.z = qz;
	std::stringstream ss;
	ss << "Waypoint No." << marker_no;
	int_marker.name = ss.str().c_str();
	int_marker.description = ss.str().c_str();
	marker_no++;
	int_marker.header.frame_id = "map";

	visualization_msgs::InteractiveMarkerControl plane_control;
	plane_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	plane_control.orientation.w = 1;
	plane_control.orientation.x = 0;
	plane_control.orientation.y = 1;
	plane_control.orientation.z = 0;
	plane_control.always_visible = true;

	visualization_msgs::InteractiveMarkerControl rotate_control;
	rotate_control.name = "Rotate Yaw";
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	rotate_control.orientation.w = 1;
	rotate_control.orientation.x = 0;
	rotate_control.orientation.y = 1;
	rotate_control.orientation.z = 0;
	rotate_control.always_visible = true;

	visualization_msgs::Marker arr_marker;
	arr_marker.type = visualization_msgs::Marker::ARROW;
	arr_marker.scale.x = 0.65;
	arr_marker.scale.y = 0.1;
	arr_marker.scale.z = 0.1;
	arr_marker.color.r = 1.0;
	arr_marker.color.g = 0.0;
	arr_marker.color.b = 0.0;
	arr_marker.color.a = 1.0;
	arr_marker.pose.position.z = 0.1;
	rotate_control.markers.push_back(arr_marker);

	visualization_msgs::Marker cyl_marker;
	cyl_marker.type = visualization_msgs::Marker::CYLINDER;
	cyl_marker.scale.x = 0.6;
	cyl_marker.scale.y = 0.6;
	cyl_marker.scale.z = 0.2;
	cyl_marker.color.r = 0.0;
	cyl_marker.color.g = 1.0;
	cyl_marker.color.b = 0.0;
	cyl_marker.color.a = 1.0;
	plane_control.markers.push_back(cyl_marker);
	cyl_marker.scale.x = 1.0;
	cyl_marker.scale.y = 1.0;
	cyl_marker.scale.z = 0.1;
	cyl_marker.color.r = 0.0;
	cyl_marker.color.g = 0.0;
	cyl_marker.color.b = 1.0;
	cyl_marker.color.a = 1.0;
	rotate_control.markers.push_back(cyl_marker);

	int_marker.controls.push_back(plane_control);
	int_marker.controls.push_back(rotate_control);
	server->insert(int_marker, &processFeedback);

	server->applyChanges();
}

void callback(const geometry_msgs::PointStamped::ConstPtr point){
	new_marker(point->point.x,point->point.y,point->point.z,0,0,0,1);
	std::vector<double> create_wp(7,0);
	create_wp[0] = point->point.x;
	create_wp[1] = point->point.y;
	create_wp[2] = point->point.z;
	create_wp[6] = 1;
	waypoints.push_back(create_wp);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ts_wp_editor");

	if(argc == 2){
		wp = savename = argv[1];
		if(wp.find(".wp")==std::string::npos){
			ROS_ERROR("Use .wp file");
			return 0;
		}
		savename.insert(wp.find(".wp"),"_changed");
	}
	else{
		ROS_INFO("Ah~ this is shit!");
	}
	ros::NodeHandle n;
	ros::Subscriber point_sub = n.subscribe("/clicked_point",1,callback);
	server = new interactive_markers::InteractiveMarkerServer("WayPoint");
	int s;
	if(FILE* fp = fopen(wp.c_str(), "r")){
		double px,py,pz,qx,qy,qz,qw;
		fscanf(fp,"Number Of Goals :%d\n",&s);
		waypoints = std::vector<std::vector<double> >(s, std::vector<double>(7, 0));
		for(int i = 0;i < s;i++){
			fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",&px,&py,&pz,&qx,&qy,&qz,&qw);
			waypoints[i][0] = px;
			waypoints[i][1] = py;
			waypoints[i][2] = pz;
			waypoints[i][3] = qx;
			waypoints[i][4] = qy;
			waypoints[i][5] = qz;
			waypoints[i][6] = qw;
			new_marker(px,py,pz,qx,qy,qz,qw);
		}
		fclose(fp);
		ROS_INFO("Loaded %d Waypoints",s);
	}
	else{
		ROS_ERROR("Can't open waypoint file.");
		return 0;
	}
	ros::spin();


	if(FILE* fp = fopen(savename.c_str(), "w")){
		printf("Writing WayPoints");
		fprintf(fp,"Number Of Goals :%04d\n",marker_no);
		for(int i = 0;i < marker_no;i++){
			fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",waypoints[i][0],waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4],waypoints[i][5],waypoints[i][6]);
		}
		fclose(fp);
		printf("Done");
	}
	else{
		ROS_ERROR("Can't write waypoint file.");
		return 0;
	}

	return 0;

}
