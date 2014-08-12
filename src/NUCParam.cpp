#include "NUCParam.h"

double NUCParam::area_rotation = 0;
double NUCParam::cx = 0;
double NUCParam::cy = 0;
double NUCParam::sensingTime = 2;
double NUCParam::min_footprint = 4.0;
std::string NUCParam::log_folder = std::string("");
bool NUCParam::logging = false;
double NUCParam::FOV = 3.14/2.0;
std::string NUCParam::interesting_label = "grass";
std::string NUCParam::training_set_dir = "";
int NUCParam::image_w = 640;
int NUCParam::image_h = 480;
bool NUCParam::simulation = true;
bool NUCParam::interesting_simulation = false;
bool NUCParam::visualization = true;
int NUCParam::bf_sqrt = 2;
double NUCParam::speed = 1.0;
double NUCParam::area_length = 16;
bool NUCParam::sim_running = true;


void NUCParam::GetParams(ros::NodeHandle &nh)
{
    nh.param<double>("FOV",FOV,1.57);
    nh.param<double>("area_rotation",area_rotation,0);
    nh.param<double>("sensing_time",sensingTime,2);
    nh.param<double>("min_footprint",min_footprint,2);
    nh.param("logging", logging, false);
    nh.param("log_folder", log_folder, std::string(""));
    nh.param("interesting_label", interesting_label, std::string(""));
    nh.param("training_set_dir", training_set_dir, std::string(""));
    nh.param<int>("image_w",image_w,640);
    nh.param<int>("image_h",image_h,480);

    nh.param<bool>("simulation", simulation, true);
    nh.param<bool>("interesting_simulation", interesting_simulation, true);
    nh.param<bool>("visualization", visualization, true);
    nh.param<int>("branching_sqrt",bf_sqrt,2);
    nh.param<double>("speed",speed,1.0);
    nh.param<double>("area_length",area_length,16);

}
