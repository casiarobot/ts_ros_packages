#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

std::string mapname,savename;
double x,y,yaw;
float res;
int width,height,max;
int line_size = 1;
cv::Mat img,tmp_img;
cv::Point prev_pt;

cv::Scalar color = CV_RGB(0,0,0);

class MapEditor{

	public:
	MapEditor(){
		cv::namedWindow("ts_map_editor",CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
		cv::resizeWindow("ts_map_editor",1000,1000);
		cv::createButton("Save", save, NULL, CV_PUSH_BUTTON, 0);
		cv::createButton("Undo", undo, NULL, CV_PUSH_BUTTON, 0);
		cv::createTrackbar("Line Size","ts_map_editor",&line_size,10);
		cvCreateButton("White",choose_white,NULL,CV_RADIOBOX,0);
		cvCreateButton("Gray",choose_gray,NULL,CV_RADIOBOX,0);
		cvCreateButton("Black",choose_black,NULL,CV_RADIOBOX,1);
		cv::setMouseCallback("ts_map_editor", mouse_callback);
		cv::Mat src_img = cv::imread(mapname, 0);
		img = src_img;
		tmp_img = src_img;
//		while(1){
		cv::imshow("ts_map_editor",img);
		cv::waitKey(0);
//		}
		return;
	}
	~MapEditor(){
		cv::destroyWindow("test");
	}
	static void save(int state, void* userdata){
		img = (img/255)*100;
		ROS_INFO("Saving...");
		if(FILE* fp = fopen(savename.c_str(), "w")){
			fprintf(fp,"P2\n");
			fprintf(fp,"#origin.x : %.4lf\n",x);
			fprintf(fp,"#origin.y : %.4lf\n",y);
			fprintf(fp,"#origin.yaw : %.4lf\n",yaw);
			fprintf(fp,"#resolution : %.4lf\n",res);
			fprintf(fp,"%d %d\n",width,height);
			fprintf(fp,"%d\n",max);
			int pixel = 0;
			for(int y = 0 ; y < img.rows; y++){
				for(int x = 0 ; x < img.cols; x++){
					pixel = img.data[y * img.cols + x];
					fprintf(fp,"%d ",pixel);
				}
				fprintf(fp,"\n");
			}
			fclose(fp);			
		}
		else{
			ROS_ERROR("Failed to save pgm.");
			return;
		}
		ROS_INFO("DONE");
		std::exit(0);
	}
	static void undo(int state, void* userdata){
		img = tmp_img;
		cv::imshow("ts_map_editor",tmp_img);
		ROS_INFO("Undo");
		return;
	}
	static void choose_white(int state, void* userdata){
		color = CV_RGB(255,255,255);
		return;
	}
	static void choose_gray(int state, void* userdata){
		color = CV_RGB(127,127,127);
		return;
	}
	static void choose_black(int state, void* userdata){
		color = CV_RGB(0,0,0);
		return;
	}
	static void mouse_callback( int event, int x, int y, int flags, void* ) {
		if(event == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON) || !(flags & CV_EVENT_FLAG_CTRLKEY)){
        		prev_pt = cv::Point(-1,-1);
			tmp_img = img;
		}
		else if(event == CV_EVENT_LBUTTONDOWN && (flags & CV_EVENT_FLAG_CTRLKEY)){
			prev_pt = cv::Point(x,y);
		}
		else if(event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) && (flags & CV_EVENT_FLAG_CTRLKEY)){
			cv::Point pt = cvPoint(x,y);
			if( prev_pt.x < 0 ){
				tmp_img=img;
				prev_pt = pt;
			}
        		cv::line(img, prev_pt, pt, color, line_size);
        		prev_pt = pt;
			imshow("ts_map_editor",img);
			cv::waitKey(0);
		}
		return;
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "ts_map_editor");
	if(argc >=2 && 4 > argc){
		mapname = argv[1];
		if(mapname.find(".pgm")==std::string::npos){
			ROS_ERROR("Use pgm file");
			return 0;
		}
		if(argc==3){
			savename = argv[2];
				if(savename.find(".pgm")==std::string::npos){
					ROS_ERROR("Use pgm file");
				return 0;
			}
		}
		else{
			savename=argv[1];
			savename.insert(mapname.find(".pgm"),"_changed");;
		}
	}
	else{
		ROS_ERROR("Fuck");
		return 0;
	}
	if(FILE* fp = fopen(mapname.c_str(), "r")){
		fscanf(fp,"P2\n");
		fscanf(fp,"#origin.x : %lf\n",&x);
		fscanf(fp,"#origin.y : %lf\n",&y);
		fscanf(fp,"#origin.yaw : %lf\n",&yaw);
		fscanf(fp,"#resolution : %f\n",&res);
		fscanf(fp,"%d %d\n",&width,&height);
		fscanf(fp,"%d\n",&max);
		fclose(fp);
	}
	else{
		ROS_ERROR("Can't open %s",mapname.c_str());
		return 0;
	}
//	ROS_INFO("%.4lf %.4lf %.4lf %.4f %d %d %d\n",x,y,yaw,res,width,height,max);
	ROS_INFO("input:%s output:%s",mapname.c_str(),savename.c_str());
	MapEditor me;
	return 0;
}
