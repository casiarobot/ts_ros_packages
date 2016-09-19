#include <ros/ros.h>
#include "dynamic_reconfigure/server.h"
#include "amcl_laser_model_viz/lasermodelConfig.h"

double z_hit,z_short,z_max,z_rand,sigma_hit,lambda_short;
double max,true_value;
FILE* gp;

void reconfigureCB(amcl_laser_model_viz::lasermodelConfig &config, uint32_t level) {
	z_hit = config.laser_z_hit;
	z_short = config.laser_z_short;
	z_max = config.laser_z_max;
	z_rand = config.laser_z_rand;
	sigma_hit = config.laser_sigma_hit;
	lambda_short = config.laser_lambda_short;
	max = config.max;
	true_value = config.true_value;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "amcl_laser_model_viz");
	dynamic_reconfigure::Server<amcl_laser_model_viz::lasermodelConfig> server;
	dynamic_reconfigure::Server<amcl_laser_model_viz::lasermodelConfig>::CallbackType f = boost::bind(&reconfigureCB, _1, _2);
	server.setCallback(f);

	ros::NodeHandle p_nh;
	p_nh.param("laser_z_hit", z_hit, 0.95);
	p_nh.param("laser_z_short", z_short, 0.1);
	p_nh.param("laser_z_max", z_max, 0.05);
	p_nh.param("laser_z_rand", z_rand, 0.05);
	p_nh.param("laser_sigma_hit", sigma_hit, 0.2);
	p_nh.param("laser_lambda_short", lambda_short, 0.1);
	p_nh.param("max", max, 20.0);
	p_nh.param("true_value", true_value, 10.0);

	gp = popen("gnuplot","w");
	fprintf(gp,"set sample 10000\n");
	fprintf(gp,"set ylabel \'Likelihood\'\n");
	fprintf(gp,"set xlabel \'Distance\'\n");
	fprintf(gp, "set yrange [0:1.1]\n");
	fprintf(gp,"unset key\n");
	std::stringstream ss;
//	ss<<"z_hit(x) = "<<z_hit<<"*(exp(-((x-"<<true_value<<")**2)/(2*"<<sigma_hit<<"**2)))"<<"\n";
//	fprintf(gp,"%s",ss.str().c_str());
//	ss<<"z_short(x) = (x<"<<true_value<<")?"<<z_short*lambda_short<<"*(exp(-"<<lambda_short<<"*x)):0"<<"\n";
//	fprintf(gp,"%s",ss.str().c_str());
//	ss<<"z_max(x) = (x>"<<max-0.01<<")?"<<z_max<<":0"<<"\n";
//	fprintf(gp,"%s",ss.str().c_str());
//	ss<<"z_rand(x) = "<<z_rand<<"\n";
//	fprintf(gp,"%s",ss.str().c_str());
	while(ros::ok()){
	fprintf(gp, "set xrange [0:%lf]\n",max+1);
	fprintf(gp, "set xtics(\'z*\' %lf,\'z_{max}\' %lf)\n",true_value,max);
	ss<<"beam(x)=(x<"<<max<<")?(("<<z_hit<<"*(exp(-((x-"<<true_value<<")**2)/(2*"<<sigma_hit<<"**2))))+((x<"<<true_value<<")?"<<z_short*lambda_short<<"*(exp("<<-lambda_short<<"*x)):0)+((x>"<<max-0.01<<")?"<<z_max<<":0)+"<<z_rand<<"):1/0\n";
	fprintf(gp,"%s",ss.str().c_str());
	fprintf(gp,"set label 1 \'{/Symbol Z}_{hit}=0.6\' at 16,0.8\n");
	fprintf(gp,"set label 2 \'{/Symbol Z}_{short}=0.2\' at 16,0.7\n");
	fprintf(gp,"set label 3 \'{/Symbol Z}_{max}=0.05\' at 16,0.6\n");
	fprintf(gp,"set label 4 \'{/Symbol Z}_{rand}=0.15\' at 16,0.5\n");
	fprintf(gp,"set label 5 \'{/Symbol s}_{hit}=0.5\' at 16,0.4\n");
	fprintf(gp,"set label 6 \'{/Symbol l}_{short}=0.25\' at 16,0.3\n");
	fprintf(gp,"set label 7 \'z_{max}=20\' at 16,0.2\n");
	fprintf(gp,"set label 1 font \'Arial,25\'\n");
	fprintf(gp,"set label 2 font \'Arial,25\'\n");
	fprintf(gp,"set label 3 font \'Arial,25\'\n");
	fprintf(gp,"set label 4 font \'Arial,25\'\n");
	fprintf(gp,"set label 5 font \'Arial,25\'\n");
	fprintf(gp,"set label 6 font \'Arial,25\'\n");
	fprintf(gp,"set label 7 font \'Arial,25\'\n");
	fprintf(gp,"plot beam(x)\n");
	ros::spinOnce();
	}
//	fprintf(gp,"replot z_hit(x) title \"Hit\"\n");
//	fprintf(gp,"replot z_short(x) title \"Short\"\n");
//	fprintf(gp,"replot z_max(x) title \"Max\"\n");
//	fprintf(gp,"replot z_rand(x) title \"Rand\"\n");

	//↓PNGで画像出力するとき
//	fprintf(gp,"set terminal png\nset output \'test.png\'\nplot beam(x) title \"Beam\"\nset terminal x11\nset output\n");
	//↓epsで画像出力するとき
	fprintf(gp,"set terminal postscript eps enhanced\nset output \'test.eps\'\nplot beam(x) title \"Beam\"\nset terminal x11\nset output\n");

	pclose(gp);
	return 0;
}
