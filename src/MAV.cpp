#include "MAV.h"
#include "GL/glut.h"
#include "pelican_ctrl/gotoPos.h"
#include "tf/tf.h"
//#include "HuskyInterface.h"
#include "NUC.h"
#include "NUCParam.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

using namespace TooN;

//double MAV::speed = 1;

MAV::MAV()
{
    pos = makeVector(0,0,10);
    //NUCParam::speed = 1;
    realpos = makeVector(0,0,0);
    yaw = 0;
}

void MAV::Init(ros::NodeHandle *nh_, bool simulation_)
{
    nh = nh_;
    simulation = simulation_;

    //if(!simulation)
    {
        gotoPosService = nh->serviceClient<pelican_ctrl::gotoPos>("/PelicanCtrl/gotoPos");
        atGoalSub = nh->subscribe("/PelicanCtrl/at_goal", 10, &MAV::atGoalCallback, this);
        gpsPose_sub = nh->subscribe("/PelicanCtrl/fixedPose", 10, &MAV::gpsPoseCallback, this);
        gps_sub = nh->subscribe("/fcu/gps", 10, &MAV::gpsCallback, this);
        //test();
    }

    if(simulation)
    {
        asctecPelican.Init(nh_);
    }
}

void MAV::test()
{
    SetGoal(makeVector(0,0,-5));
}

void MAV::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    realpos[0] = msg->pose.pose.position.x;
    realpos[1] = msg->pose.pose.position.y;
    realpos[2] = msg->pose.pose.position.z;
    //yaw = tf::getYaw(msg->pose.pose.orientation);


    tf::Matrix3x3 rm;
    tf::Quaternion qtr(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    rm.setRotation(qtr);
    double r1,r2,r3;
    rm.getEulerYPR(r1,r2,r3);
//    if(simulation)
     yaw = r1;

    //ROS_INFO_THROTTLE(1, "Yaw: %f", yaw);
    //ROS_INFO_THROTTLE(0.5, "Height: %f\t Goal height: %f", realpos[2], goal[2]);
    //ROS_INFO_THROTTLE(3, "POS: %f %f %f rot: %f %f %f", realpos[0], realpos[1], realpos[2], r1, r2, r3);

    NUC_LOG("POSE: T %f %f %f R %f %f %f\n", realpos[0], realpos[1], realpos[2], r1, r2, r3);

    //realpos[3] = 0;
}

void MAV::gpsCallback(const sensor_msgs::NavSatFixPtr &msg)
{
    //static bool sent=false;
    gpsLocation = (*msg);

//    if(!sent)
//    {
//        HuskyInterafce::Instance()->SendWaypoint(gpsLocation);
//        ROS_INFO("Sent GPS to husky.");
//        sent = true;
//    }

}

void MAV::atGoalCallback(const std_msgs::Bool::Ptr &msg)
{
    if(msg->data == true)
        atGoal = true;
}

void circle(float x, float y, float z, float r, int segments)
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(x, y, z);
    for( int n = 0; n <= segments; ++n )
    {
        float const t = 2*M_PI*(float)n/(float)segments;
        glVertex3f(x + sin(t)*r, y + cos(t)*r, z);
    }
    glEnd();
}

double r(double half_p=0.5)
{
    return (-half_p*10.0+rand()%((int)ceil(2.0*half_p*10.0)))/10.0;
}

void glVertex(Vector<3> p)
{
    glVertex3f(p[0],p[1],p[2]);
}

void MAV::glDraw()
{
    //ROS_INFO_THROTTLE(1, "AtGoal:%d", atGoal);
    Vector<3> p;
//    if(simulation)
//    {
//        p = pos;
//    }
//    else
    {
        p[0] = realpos[0];p[1] = realpos[1];p[2] = realpos[2];
    }

    glColor3f(0,0,1);
    glLineWidth(2);
    double l=0.25;
    glBegin(GL_LINES);
    Vector<3> p1,p2,p3,p4;
    Matrix<3> rot_m = Data(cos(-yaw), sin(-yaw), 0,
                          -sin(-yaw), cos(-yaw), 0,
                           0        , 0        , 0);

    //ROS_INFO_THROTTLE(1,"%f %f %f %f", rot_m[0][0], rot_m[0][1], rot_m[1][0], rot_m[1][1]);
    p1=p+rot_m*makeVector(-l+r()*0.01,+r()*0.01,0);
    p2=p+rot_m*makeVector(+l+r()*0.01,+r()*0.01,0);
    p3=p+rot_m*makeVector(+r()*0.01,-l+r()*0.01,0);
    p4=p+rot_m*makeVector(+r()*0.01,+l+r()*0.01,0);
    glVertex(p1);
    glVertex(p2);
    glVertex(p3);
    glVertex(p4);
    glEnd();

    glColor4f(0,0,0,0.5);
    circle(p1[0]+r()*0.01,p1[1]+r()*0.01,p1[2], 0.15, 50);
    //glColor4f(0,1,0,0.9);
    glColor4f(0,0,0,0.5);
    circle(p2[0]+r()*0.01,p2[1]+r()*0.01,p2[2], 0.15, 50);
    glColor4f(0,0,0,0.5);
    circle(p3[0]+r()*0.01,p3[1]+r()*0.01,p3[2], 0.15, 50);
    glColor4f(0,0,0,0.5);
    circle(p4[0]+r()*0.01,p4[1]+r()*0.01,p4[2], 0.15, 50);

//    glPointSize(10);
//    glColor3f(0,0,1);
//    glBegin(GL_POINTS);
//    glVertex3f(goal[0], goal[1], goal[2]);
//    glEnd();
}

void MAV::SetGoal(TooN::Vector<3> goalpos, bool set_orig)
{
    goal = goalpos;
    atGoal = false;

//    if(simulation)
//    {
//        toGoalNorm = (goal-pos);
//        normalize(toGoalNorm);
//    }
//    else
    {

        pelican_ctrl::gotoPos srv;
        srv.request.x = goal[0];
        srv.request.y = goal[1];
        srv.request.z = goal[2];
        srv.request.yaw = 0;
        srv.request.set_orig = set_orig;

        if(gotoPosService.call(srv))
        {
            //ROS_INFO("Waypoint is sent to Pelican Controller");
        }
        else
        {
            ROS_INFO("** Unable to call gotoPos service on Pelican Controller");
        }
    }
}

void MAV::Update(double dt)
{
    if(simulation)
    {
        asctecPelican.Update();


//        if(atGoal)
//            return;

//        double step = dt * NUCParam::speed;
//        double goalDistSqr = (goal - pos)*(goal-pos);
//        if(goalDistSqr < step*step || goalDistSqr < 0.2*0.2)
//        {
//            pos = goal;
//            atGoal = true;

//        }
//        else
//        {
//            pos += step*toGoalNorm;
//        }
    }
    else
    {
        //We don't need to check if MAV is at goal since
        //it will be set in upon receiving the AtGoal message
        //ROS_INFO_THROTTLE(1, "AtGoal:%d", atGoal);
    }
}


MAV::AsctecFCU::AsctecFCU()
{
    delay = 0;
    this->pose = makeVector(0,0,2,0);
    vel = makeVector(0,0,0,0);
}

void MAV::AsctecFCU::Init(ros::NodeHandle *nh_)
{
    fcuPose_pub = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/fcu/gps_pose", 10);
    fcuMag_pub = nh_->advertise<geometry_msgs::Vector3Stamped>("/fcu/mag", 50);
    fcuCtrl_sub = nh_->subscribe("/fcu/control", 20, &MAV::AsctecFCU::fcuCtrlCallback, this);
}

void MAV::AsctecFCU::Update()
{
    static ros::Time last_mag = ros::Time::now();
    static ros::Time last_pose = ros::Time::now();

    static unsigned int p_seq = 0;
    static unsigned int m_seq = 0;

    static Vector<4> p = makeVector(0,0,0,0);

    ros::Time t = ros::Time::now();

    if((t-last_pose).toSec() > 0.02) // = 5 hz
    {

        last_pose = t;


//        while(poseQ.size()>1  && fabs(t.toSec() - poseQ[poseQ.size()-2].first) < fabs(t.toSec() - poseQ[poseQ.size()-1].first) )
//            poseQ.pop_back();

//        if(poseQ.size() > 2)
        {
            p_seq++;
            geometry_msgs::PoseWithCovarianceStamped p_msg;
            p = poseQ.back().second;
            p_msg.pose.pose.position.x = p[0];
            p_msg.pose.pose.position.y = p[1];
            p_msg.pose.pose.position.z = p[2];
            p_msg.header.stamp = t;
            p_msg.header.seq = p_seq;

            fcuPose_pub.publish(p_msg);
        }
//        else
        {
            poseQ.clear();
        }
    }

    if((t-last_mag).toSec() > 1/50) // = 50 hz
    {
        double dt = (t-last_mag).toSec();
        last_mag = t;

        if(NUCParam::sim_running)
        {
            m_seq++;
            geometry_msgs::Vector3Stamped m_msg;

            m_msg.vector.x = cos(p[3]+1.57/2);
            m_msg.vector.y = sin(p[3]+1.57/2);

            m_msg.header.stamp = t;
            m_msg.header.seq = m_seq;

            fcuMag_pub.publish(m_msg);

            //ROS_INFO("vel: %f %f %f", vel[0], vel[1], vel[2]);
            pose += dt * NUCParam::speed * vel;
            poseQ.insert(poseQ.begin(), std::pair<double,Vector<4> >(t.toSec()+delay,pose));


        }
    }
}

void MAV::AsctecFCU::fcuCtrlCallback(const asctec_hl_comm::mav_ctrl::Ptr &msg)
{

    static ros::Time last_t = ros::Time::now();
    ros::Time t = ros::Time::now();

    double dt = (t-last_t).toSec();
    if(dt > 0.5)
    {
        ROS_INFO("Delay in publishing control commands. Ignoring /fcu/control ....");
        dt = 0;
    }

    last_t = t;

    Vector<2> bodyX = makeVector(-cos(pose[3]+1.57), -sin(pose[3]+1.57));
    Vector<2> bodyY = makeVector(cos(pose[3]+3.14), sin(pose[3]+3.14));

    Vector<2> p = msg->x*bodyX + msg->y*bodyY;

    static Vector<3> sigv = makeVector(0,0,0);
    //Vector<3> sig = 0.01 * makeVector(r(),r(),r());
    //sigv += sig;

    //Vector<4> dp;
    vel[0] = p[0] + sigv[0];
    vel[1] = p[1] + sigv[1];
    vel[2] = msg->z + sigv[2];
    vel[3] = 0;//msg->yaw;

}
