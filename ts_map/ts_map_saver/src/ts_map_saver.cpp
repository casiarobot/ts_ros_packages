#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

class OG2pgm
{
	public:
	std::string mapname;
	ros::Subscriber map_sub;
	bool saved;

	OG2pgm(const std::string& mapname) : mapname(mapname), saved(false){
		ros::NodeHandle n;
		ROS_INFO("Waiting for the map");
		map_sub = n.subscribe("map", 1, &OG2pgm::mapCallback, this);
	}

	void mapCallback(const nav_msgs::OccupancyGrid& map){
		ROS_INFO("Received a %d X %d map @ %.3f m/pix",map.info.width,map.info.height,map.info.resolution);
		std::string mapdatafile = mapname + ".pgm";
		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		FILE* out = fopen(mapdatafile.c_str(), "w");
		if(out == NULL){
		ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
		return;
		}
		fprintf(out,"P2\n");
		fprintf(out,"#origin.x : %.4lf\n",map.info.origin.position.x);
		fprintf(out,"#origin.y : %.4lf\n",map.info.origin.position.y);
        	geometry_msgs::Quaternion orientation = map.info.origin.orientation;
        	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        	double yaw, pitch, roll;
        	mat.getEulerYPR(yaw, pitch, roll);
		fprintf(out,"#origin.yaw : %.4lf\n",yaw);
		fprintf(out,"#resolution : %.4lf\n",map.info.resolution);
		fprintf(out,"%d %d\n",map.info.width,map.info.height);
		fprintf(out,"%d\n",100);
		int pixel = 0;
		int index = 0;
		for(size_t row = 0;row < map.info.height;row++){
			for(size_t column = 0;column < map.info.width;column++){
				index = (map.info.height-row-1)*map.info.width+column;
				if((map.data[index] < 0 || map.data[index] == 50))pixel = 50;
				else if(map.data[index] >= 100)pixel = 0;
				else pixel = 100;
				fprintf(out,"%d ",pixel);
			}
			fprintf(out,"\n");
		}
      		fclose(out);
      		saved = true;
		ROS_INFO("The information in the previous yaml file, It is included in the pgm file");
		ROS_INFO("DONE");
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "ts_map_saver");
	std::string mapname = "map";

	for(int i=1; i<argc; i++){
		if(!strcmp(argv[i], "-h")){
			std::cout << USAGE;
			return 0;
		}
		else if(!strcmp(argv[i], "-f")){
			if(++i < argc){
				mapname = argv[i];
			}
			else{
				std::cout << USAGE;
				return 1;
			}
		}
		else{
			std::cout << USAGE;
			return 1;
		}
	}
  
	OG2pgm og2pgm(mapname);

	while(!og2pgm.saved && ros::ok()){
		ros::spinOnce();
	}

	return 0;
}
