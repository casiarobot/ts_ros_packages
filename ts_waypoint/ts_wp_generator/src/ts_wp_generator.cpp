#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

bool first = true;
int marker_id = 0;
int wp_no = 0;

int main(int argc, char** argv){
  ros::init(argc, argv, "ts_wp_generator");

  ros::NodeHandle node;
  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("/gmapping_pose",1);
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>("/gmapping_path",1);
  ros::Publisher wp_pub = node.advertise<visualization_msgs::MarkerArray>("/waypoints", 1);

  tf::TransformListener listener;
  geometry_msgs::PoseStamped ps;
  geometry_msgs::PoseStamped current_wp;
  nav_msgs::Path path;
  visualization_msgs::MarkerArray marker_array;
  uint32_t shape = visualization_msgs::Marker::ARROW;

  std::ofstream ofs("output.wp");

  ros::Rate rate(2.0);
  while (node.ok()){
    visualization_msgs::Marker marker;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    ps.header.stamp = path.header.stamp = marker.header.stamp = ros::Time::now();
    ps.header.frame_id = path.header.frame_id = marker.header.frame_id = "/map";
    marker.ns = "way_points";
    if (first){
      marker.id = marker_id;
      marker.type = shape;
      current_wp.pose.position.x = marker.pose.position.x = 0;
      current_wp.pose.position.y = marker.pose.position.y = 0;
      current_wp.pose.position.z = marker.pose.position.z = 0;
      current_wp.pose.orientation.x = marker.pose.orientation.x = 0.0;
      current_wp.pose.orientation.y = marker.pose.orientation.y = 0.0;
      current_wp.pose.orientation.z = marker.pose.orientation.z = 0.0;
      current_wp.pose.orientation.w = marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.8;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      ofs << "Number Of Goals :0000\n";
      ofs << "0.0000\t0.0000\t0.0000\t0.0000\t0.0000\t0.0000\t1.0000\n";
      marker_array.markers.push_back(marker);
      wp_pub.publish(marker_array);
      first = false;
      marker_id++;
      wp_no++;
    }
    ps.pose.position.x = transform.getOrigin().getX();
    ps.pose.position.y = transform.getOrigin().getY();
    ps.pose.position.z = transform.getOrigin().getZ();
    ps.pose.orientation.x = transform.getRotation().getX();
    ps.pose.orientation.y = transform.getRotation().getY();
    ps.pose.orientation.z = transform.getRotation().getZ();
    ps.pose.orientation.w = transform.getRotation().getW();

    if(sqrt(pow(ps.pose.position.x - current_wp.pose.position.x,2.0)+pow(ps.pose.position.y - current_wp.pose.position.y,2.0)) >= 2.5){
      marker.id = marker_id;
      marker.type = shape;
      current_wp.pose.position.x = marker.pose.position.x = ps.pose.position.x;
      current_wp.pose.position.y = marker.pose.position.y = ps.pose.position.y;
      current_wp.pose.position.z = marker.pose.position.z = ps.pose.position.z;
      current_wp.pose.orientation.x = marker.pose.orientation.x = ps.pose.orientation.x;
      current_wp.pose.orientation.y = marker.pose.orientation.y = ps.pose.orientation.y;
      current_wp.pose.orientation.z = marker.pose.orientation.z = ps.pose.orientation.z;
      current_wp.pose.orientation.w = marker.pose.orientation.w = ps.pose.orientation.w;
      marker.scale.x = 0.8;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      ofs << ps.pose.position.x << "\t" << ps.pose.position.y << "\t" << ps.pose.position.z << "\t" << ps.pose.orientation.x << "\t" << ps.pose.orientation.y << "\t" << ps.pose.orientation.z << "\t" << ps.pose.orientation.w << "\n";

      marker_array.markers.push_back(marker);
      wp_pub.publish(marker_array);
      marker_id++;
      wp_no++;
    }

    path.poses.push_back(ps);

    path_pub.publish(path);
    pose_pub.publish(ps);

    rate.sleep();
  }

  ofs.seekp(0);
  ofs << "Number Of Goals :" << std::setw(4) << std::setfill('0') << wp_no << "\n";
  return 0;
};
