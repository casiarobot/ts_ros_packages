#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/exceptions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/geometry.h>

const double PI = 3.14159265;


//void callback(const nav_msgs::Odometry& odometry){
//	robot_pose = odometry.pose.pose;
//}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ts_laser_sim");

	ros::NodeHandle n;
//	ros::Subscriber pose_sub = n.subscribe("/odom",1,callback);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
	ros::ServiceClient map_service_client = n.serviceClient<nav_msgs::GetMap>("static_map");
	nav_msgs::GetMap srv_map;
	nav_msgs::OccupancyGrid grid;
	ros::Rate loop_rate(40);
	geometry_msgs::Pose laser_pose;
	tf::TransformListener listener;

	sleep(4);

	if (map_service_client.call(srv_map)){
		ROS_INFO("Map service called successfully");
		grid = srv_map.response.map;
	}
	else{
		ROS_ERROR("Failed to call map service");
		return 0;
	}

	while(ros::ok()){
		ros::spinOnce();
		sensor_msgs::LaserScan scan_info;
		scan_info.angle_min = -PI*3/4;
		scan_info.angle_max = PI*3/4;
		scan_info.angle_increment = (PI*3/2)/1080;
		scan_info.range_max = 25.0;
		scan_info.header.frame_id = "laser";
		tf::StampedTransform transform;
		scan_info.header.stamp = ros::Time::now();
		try{
			listener.lookupTransform("/odom","/laser",ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		laser_pose.orientation.x = transform.getRotation().getX();
		laser_pose.orientation.y = transform.getRotation().getY();
		laser_pose.orientation.z = transform.getRotation().getZ();
		laser_pose.orientation.w = transform.getRotation().getW();
		laser_pose.position.x = transform.getOrigin().getX();
		laser_pose.position.y = transform.getOrigin().getY();
		laser_pose.position.z = transform.getOrigin().getZ();
		sensor_msgs::LaserScan::ConstPtr scan = occupancy_grid_utils::simulateRangeScan(grid, laser_pose, scan_info,0);
		scan_pub.publish(scan);
		loop_rate.sleep();
	}

	return 0;
}

