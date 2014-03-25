
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "PelicanCtrl/start_log.h"
#include "PelicanCtrl/periodic.h"

bool startPeriodic = false;

void usage()
{
  std::string text("usage: \n\n");
  text = "ctrl_test motors [0 | 1]\n";
  text += "ctrl_test ctrl [acc | vel | pos] x y z yaw\n";
  std::cout << text << std::endl;
}

bool start_log(PelicanCtrl::start_logRequest &req, PelicanCtrl::start_logResponse &res)
{
    ROS_INFO("Service Called ................");
    system("~/log.sh");
   // system("source /media/startlog.sh");
    return true;
}

bool periodic(PelicanCtrl::periodicRequest &req, PelicanCtrl::periodicResponse &res)
{
    startPeriodic = !startPeriodic;
    return true;
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "PelicanCtrl");
  ros::NodeHandle nh;

  ros::Publisher pub;
  ros::ServiceServer service = nh.advertiseService("start_log", start_log);
  ros::ServiceServer periodic_service = nh.advertiseService("periodic", periodic);



//  ros::Rate r(15);

//  while(ros::ok())
//  {
//	r.sleep();
//	ros::spinOnce();
//  }

//  if (argc == 1)
//  {
//    ROS_ERROR("Wrong number of arguments!!!");
//    usage();
//    return -1;
//  }

//  std::string command = std::string(argv[1]);

//  if (command == "motors")
//  {
//    if (argc != 3)
//    {
//      ROS_ERROR("Wrong number of arguments!!!");
//      usage();
//      return -1;
//    }

//    asctec_hl_comm::mav_ctrl_motors::Request req;
//    asctec_hl_comm::mav_ctrl_motors::Response res;
//    req.startMotors = atoi(argv[2]);
//    ros::service::call("fcu/motor_control", req, res);
//    std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
//  }
//  else if (command == "ctrl")
  {
//    if (argc != 7)
//    {
//      ROS_ERROR("Wrong number of arguments!");
//      usage();
//      return -1;
//    }
    asctec_hl_comm::mav_ctrl msg;
    msg.x = 0.01;
    msg.y = 0;
    msg.z = 0;
    msg.yaw = 0;
    msg.v_max_xy = -1; // use max velocity from config
    msg.v_max_z = -1;

    std::string type("vel");
    if (type == "acc")
      msg.type = asctec_hl_comm::mav_ctrl::acceleration;
    else if (type == "vel")
      msg.type = asctec_hl_comm::mav_ctrl::velocity;
    else if (type == "pos")
      msg.type = asctec_hl_comm::mav_ctrl::position;
    else
    {
      ROS_ERROR("Command type not recognized");
      usage();
      return -1;
    }

    pub = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1);
    ros::Rate r(15); // ~15 Hz

    while(ros::ok())
    {

        if(startPeriodic)
        {
            for (int i = 0; i < 15; i++)
            {
              pub.publish(msg);
              if (!ros::ok())
                return 0;
              r.sleep();
            }
        }
            // reset
        if (type != "pos")
        {
          msg.x = 0;
          msg.y = 0;
          msg.z = 0;
          msg.yaw = 0;
        }

        for(int i=0; i<5; i++){
          pub.publish(msg);
          r.sleep();
        }


        ros::spinOnce();
    }

  }

  return 0;
}
