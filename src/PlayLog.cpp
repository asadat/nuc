
#include "PlayLog.h"
#include <string.h>
#include "GL/glut.h"
#include <map>

#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include "TooN/TooN.h"
#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include "tf/transform_datatypes.h"
#include "opencv2/opencv.hpp"

using namespace TooN;

PlayLog * PlayLog::instance = NULL;
bool visualize = true;

unsigned int updateMS = 33;
bool doUpdate = false;
bool ScanRunning = false;
timeval last_time;
bool firstFrame = true;

std::map<unsigned char, bool> Key;
bool Mouse_Left, Mouse_Right, Mouse_Middle;
TooN::Vector<2, int> Mouse_Position;
TooN::Vector<2, int> Mouse_Change;

// track camera position
TooN::Vector<3> Cam_Target;
TooN::Vector<3> Cam_Rotation;

void translateCamera(double x, double y, double z)
{
    doUpdate = true;
    Cam_Target[0] += cos(Cam_Rotation[0])*y;
    Cam_Target[1] += sin(Cam_Rotation[0])*y;
    Cam_Target[0] += cos(Cam_Rotation[0]+M_PI/2)*cos(Cam_Rotation[1])*x;
    Cam_Target[1] += sin(Cam_Rotation[0]+M_PI/2)*cos(Cam_Rotation[1])*x;
    Cam_Target[2] += sin(Cam_Rotation[1])*x + z;
}

void rotateCamera(double yaw, double pitch, double roll)
{
    doUpdate = true;
    Cam_Rotation[0] += yaw;
    Cam_Rotation[1] += pitch;
    Cam_Rotation[2] += roll;
}

void update_event(int ms)
{
   // static FeatureTracker featureTracker(makeVector(0,0,3));


    static bool firsttime =true;

    if(firsttime)
    {

        //featureTracker.ExecuteCoveragePlan(World::Instance()->GetWorldWidth(), World::Instance()->GetWorldLength(), 2, 0.5);
        firsttime = false;
    }


    doUpdate = false; // any change and we go again. If we don't change anything, we stop updating

    PlayLog::Instance()->hanldeKeyPressed(Key, doUpdate);
    // actions:
    // - W or Left_Mouse = Move Forward constant rate
    // - S or Middle_Mouse = Move Backward constant rate
    // - A and Right_Mouse = Strafe Left constant rate
    // - D and Right_Mouse = Strafe Right constant rate
    // - A = Rotate Left constant rate
    // - D = Rotate Right constant rate
    // - R = Ascend constant rate
    // - F = Descend constant rate
    // - Right Click = Yaw + Pitch camera variable rate

    double moveRate = (static_cast<double>(ms) / 1000.0) * 20.;
    double angleRate = (static_cast<double>(ms) / 1000.0) * 1.;
    if(Key['w'] || Mouse_Left) // move forward
        translateCamera(moveRate, 0, 0);

    if(Key['s'] || Mouse_Middle) // move backward
        translateCamera(-moveRate, 0, 0);

    if(Key['a'] && Mouse_Right) // strafe left
        translateCamera(0, -moveRate, 0);

    if(Key['d'] && Mouse_Right) // strafe right
        translateCamera(0, moveRate, 0);

    if(Key['a'] && !Mouse_Right) // yaw left
        rotateCamera(angleRate, 0, 0);

    if(Key['d'] && !Mouse_Right) // yaw right
        rotateCamera(-angleRate, 0, 0);

    if(Key['r']) // Ascend
        translateCamera(0, 0, moveRate);

    if(Key['f']) // Descend
        translateCamera(0, 0, -moveRate);





    if(Mouse_Right)
    {
        // yaw and pitch camera
        double rate = 0.01;
        rotateCamera(Mouse_Change[0] * rate, Mouse_Change[1] * rate, 0);
        Mouse_Change = TooN::makeVector(0,0);
    }

    glutPostRedisplay();

    if(doUpdate)
        glutTimerFunc(ms, update_event, ms);
}

void idle_event()
{
 PlayLog::Instance()->idle();

 if(PlayLog::Instance()->ortho)
    ROS_INFO_THROTTLE(1,"ortho...");

    //if(!ros::ok())
    // exit(0);
}



void render_event()
{
    //ROS_INFO_THROTTLE(0.5,"gl rendering ...");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // reset camera
    glLoadIdentity();

    double cosa = cos(Cam_Rotation[0]);
    double cosb = cos(Cam_Rotation[2]);
    double cosc = cos(Cam_Rotation[1]);
    double sina = sin(Cam_Rotation[0]);
    double sinb = sin(Cam_Rotation[2]);
    double sinc = sin(Cam_Rotation[1]);

    Matrix<3> rotMatrix = Data(
                cosa*cosb, cosa*sinb*sinc-sina*cosc, cosa*sinb*cosc+sina*sinc,
                sina*cosb, sina*sinb*sinc+cosa*cosc, sina*sinb*cosc-cosa*sinc,
                -sinb, cosb*sinc, cosb*cosc);

    Vector<3> Cam_Plane = rotMatrix * makeVector(0,1,0);
    Vector<3> Cam_Normal = rotMatrix * makeVector(0,0,1);

    Cam_Plane += Cam_Target;

    if(PlayLog::Instance()->ortho)
    {
        glOrtho(-50, 50, -50, 50, -1, 1000);
    }

    gluLookAt(Cam_Target[0], Cam_Target[1], Cam_Target[2],
            Cam_Plane[0], Cam_Plane[1], Cam_Plane[2],
            Cam_Normal[0], Cam_Normal[1], Cam_Normal[2]);

//    gluLookAt(Cam_Target[0], Cam_Target[1], Cam_Target[2],
//            0, 0, 0,
//            0, 0, 1);



   PlayLog::Instance()->glDraw();

   glutSwapBuffers();
}


void resize_event(int w, int h)
{
    // Prevent a divide by zero, when window is too short
    // (you cant make a window of zero width).
    if(h == 0)
        h = 1;
    float ratio = static_cast<float>(w) / static_cast<float>(h);

    // Use the Projection Matrix
    glMatrixMode(GL_PROJECTION);

    // Reset Matrix
    glLoadIdentity();

    // Set the viewport to be the entire window
    glViewport(0, 0, w, h);

    // Set the correct perspective.
    // clipping plane = 1 to 1000
    gluPerspective(60,ratio,0.1,1000);

    // Get Back to the Modelview
    glMatrixMode(GL_MODELVIEW);
}

void mouse_drag_event(int x, int y)
{
    static bool ignore = false;
    if(ignore)
    {
        ignore = false;
    }else if(Mouse_Right)
    {
        Mouse_Change += (Mouse_Position - makeVector(x, y));
        //Mouse_Position = makeVector(x, y);
        //glutWarpPointer(Mouse_Position[0], Mouse_Position[1]);
        if(!doUpdate)
            update_event(updateMS);
        ignore = true;
        glutWarpPointer(Mouse_Position[0], Mouse_Position[1]);
    }else
    {
        Mouse_Position = makeVector(x, y);
        //Mouse_Change = makeVector(0 ,0);
    }
}

void mouse_move_event(int x, int y)
{
    Mouse_Position = makeVector(x, y);
}

void mouse_event(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON)
    {
        Mouse_Left = (state == GLUT_DOWN);
    }else if(button == GLUT_RIGHT_BUTTON)
    {
        Mouse_Right = (state == GLUT_DOWN);
        if(Mouse_Right)
        {
            glutSetCursor(GLUT_CURSOR_NONE);
        }else
        {
            glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
        }
    }else if(button == GLUT_MIDDLE_BUTTON)
    {
        Mouse_Middle = (state == GLUT_DOWN);
    }
    if(!doUpdate)
        update_event(updateMS);
}

void keyboard_event(unsigned char key, int x, int y)
{
    Key[key] = true;
    if(!doUpdate)
        update_event(updateMS);

}

void keyboard_up_event(unsigned char key, int x, int y)
{
    Key[key] = false;
}

PlayLog::PlayLog(int argc, char **argv)
{
    ortho = false;
    drawImages = true;
    drawSensedImages = false;
    drawTrajectory = true;
    drawWaypoints = true;

    fixedYaw = 0;
    orig = makeVector(0,0,0,0);
    p_orig = makeVector(0,0,0);


    gpsPose_sub = nh.subscribe("/PelicanCtrl/fixedPose", 100, &PlayLog::gpsPoseCallback, this);
    imu_sub = nh.subscribe("/fcu/imu", 100, &PlayLog::imuCallback, this);

    fixedPose_sub = nh.subscribe("/msf_core/pose_after_update", 100, &PlayLog::fixedPoseCallback, this);


    position_pub = nh.advertise<geometry_msgs::PointStamped>("/msf_updates/position_input1", 10);

    if(argc > 1)
    {
        FILE * f = fopen(argv[1],"r");
        float p[3];
        int d[3];
        float r[4];
        while(true)
        {
            bool flag = (fscanf(f,"%f %f %f %d %d %d %f %f %f %f", &p[0], &p[1], &p[2], &d[0], &d[1], &d[2], &r[0], &r[1], &r[2], &r[3]) == EOF);
            Vector<3> wp = makeVector(p[0], p[1], p[2]);
            waypoints.push_back(wp);

            Vector<4> fp = makeVector(r[0], r[1], r[2], r[3]);
            footprints.push_back(fp);
            
            if(flag)
                break;
        }
        fclose(f);
    }

    if(argc > 2)
    {
        FILE * f = fopen(argv[2],"r");
        float pp[3];
        char tmp[2];
        float d[3];
        bool first = true;
        Vector<3> prevPos;
        double trajectoryLength = 0;
        int nn=0;

        while(true)
        {


            bool flag = (fscanf(f,"%c %f %f %f %c %f %f %f", &tmp[0], &pp[0], &pp[1], &pp[2], &tmp[1], &d[0], &d[1], &d[2]) == EOF);
            Vector<3> pos = makeVector(pp[0], pp[1], pp[2]);
            if(nn==0)
                positions.push_back(pos);

            nn++;
            if(nn%10 ==0)
            {
                positions.push_back(pos);
                if(!first)
                {
                    trajectoryLength += sqrt((prevPos-pos)*(prevPos-pos));
                }

                first = false;
                prevPos = pos;
            }

            if(flag)
            {
                ROS_INFO("*** TRAJECTORY LENGTH: %f", trajectoryLength);
                break;
            }
        }
        fclose(f);
        ROS_INFO("Poses #: %lu", positions.size());
    }

    if(argc > 3)
    {
        FILE * f = fopen(argv[3],"r");

        while(true)
        {
            char filename[256];
            bool flag = (fscanf(f,"%s", filename) == EOF);
            std::string fn(filename);

            rawTexFiles.push_back(fn);

            if(flag)
                break;
        }

        fclose(f);
    }

    if(argc > 4)
    {
        FILE * f = fopen(argv[4],"r");

        while(true)
        {
            char filename[256];
            bool flag = (fscanf(f,"%s", filename) == EOF);
            std::string fn(filename);

            sensedTexFiles.push_back(fn);

            if(flag)
                break;
        }

        fclose(f);
    }

    glutInit(&argc, argv);
}

PlayLog::~PlayLog()
{

}

void PlayLog::imuCallback(const sensor_msgs::Imu::Ptr &msg)
{
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 rot(q);
    double y=0,p=0,r=0;
    rot.getEulerYPR(y,p,r);
    ROS_INFO("YPR: %f %f %f", y, p, r);
}

void PlayLog::fixedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    static bool firstTime = true;
    geometry_msgs::PoseWithCovarianceStamped m;
    m.header.stamp = ros::Time::now();
    m.pose.pose.position = msg->pose.pose.position;
    m.pose.pose.orientation = msg->pose.pose.orientation;


    fixedYaw = tf::getYaw(m.pose.pose.orientation);
    m.pose.pose.orientation.y = fixedYaw;

    TooN::Vector<3> pos;
    pos[0] = m.pose.pose.position.x;
    pos[1] = m.pose.pose.position.y;
    pos[2] = m.pose.pose.position.z;

    if(firstTime)
    {
       p_orig = pos;
       firstTime = false;
    }

    positions.push_back(pos);
    
    Vector<4> att;
    att[0] = msg->pose.pose.orientation.x;
    att[1] = msg->pose.pose.orientation.y;
    att[2] = msg->pose.pose.orientation.z;
    att[3] = msg->pose.pose.orientation.w;
    
    curAtt = att;
}

double mean(std::vector<double> &v)
{
    if(v.empty())
        return 0;

    double sum=0;
    for(unsigned int i=0; i<v.size(); i++)
    {
        sum += v[i];
    }

    return sum/v.size();
}

double var(std::vector<double> &v)
{
    if(v.size() <= 1)
        return 0;

    double m=mean(v);
    double difsum;
    for(unsigned int i=0; i<v.size(); i++)
    {
        difsum = (m-v[i])*(m-v[i]);
    }

    return sqrt(difsum/(v.size()-1));
}

void PlayLog::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    static bool firstgpPose = false;
    Vector<3> p;
    p[0] = msg->pose.pose.position.x;
    p[1] = msg->pose.pose.position.y;
    p[2] = msg->pose.pose.position.z;

    geometry_msgs::PointStamped pointStamped;
    pointStamped.header = msg->header;
    pointStamped.header.frame_id = "/world";
    pointStamped.point.x = p[0];
    pointStamped.point.y = p[1];
    pointStamped.point.z = p[2];

    position_pub.publish(pointStamped);

    p_pos.push_back(p);

    Vector<4> att;
    att[0] = msg->pose.pose.orientation.x;
    att[1] = msg->pose.pose.orientation.y;
    att[2] = msg->pose.pose.orientation.z;
    att[3] = msg->pose.pose.orientation.w;


    p_att.push_back(att);

    double yaw = tf::getYaw(msg->pose.pose.orientation);


    Vector<4> pos = makeVector(p[0], p[1], p[2], yaw);

    if(firstgpPose)
    {
        orig = pos;
        firstgpPose = false;
    }

   // ROS_INFO("X: %f\t Y: %f\t Z: %f\t YAW: %f", pos[0], pos[1], pos[2], pos[3]*180/3.14);

}

void PlayLog::Clear()
{
    positions.clear();
    p_pos.clear();
    p_att.clear();
}

void PlayLog::glDraw()
{

    double zorder = (ortho)?-1.0:1.0; // dont know why there is a sign difference when changin to otho mode
     float w=30;
     if(!waypoints.empty())
     {
         //w = 2*waypoints[0][2];

         if(gluintsRaw.empty())
         {
             for(unsigned int i=0;i < rawTexFiles.size(); i++)
             {
                 cv::Mat img = cv::imread(rawTexFiles[i]);
                 GLuint mnFrameTex;
                 glEnable(GL_TEXTURE_2D);
                 glGenTextures(1, &mnFrameTex);
                 glBindTexture(GL_TEXTURE_2D, mnFrameTex);
                 glTexImage2D(GL_TEXTURE_2D,
                 0, GL_RGB,
                                 img.cols, img.rows,
                 0,GL_RGB,
                 GL_UNSIGNED_BYTE,
                 img.data);
                 glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , GL_NEAREST);
                 glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , GL_NEAREST);
                 gluintsRaw.push_back(mnFrameTex);
             }
         }

         if(gluintsSensed.empty())
         {
             for(unsigned int i=0;i < sensedTexFiles.size(); i++)
             {
                 cv::Mat img = cv::imread(sensedTexFiles[i]);
                 GLuint mnFrameTex;
                 glEnable(GL_TEXTURE_2D);
                 glGenTextures(1, &mnFrameTex);
                 glBindTexture(GL_TEXTURE_2D, mnFrameTex);
                 glTexImage2D(GL_TEXTURE_2D,
                 0, GL_RGB,
                                 img.cols, img.rows,
                 0,GL_RGB,
                 GL_UNSIGNED_BYTE,
                 img.data);
                 glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , GL_NEAREST);
                 glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , GL_NEAREST);
                 gluintsSensed.push_back(mnFrameTex);
             }
         }
     }

     glLineWidth(2);
     glColor4f(0,0,0,0.5);
     glBegin(GL_LINES);
     for(int i=0; i<=w; i++)
     {
     glVertex3f(-w/2, -w/2+i, 0);
     glVertex3f( w/2, -w/2+i, 0);
     }

     for(int i=0; i<=w; i++)
     {
     glVertex3f(-w/2+i, -w/2, 0);
     glVertex3f(-w/2+i, w/2, 0);
     }

     glEnd();


     glPointSize(5);
     glColor3f(1,0,0);
     glBegin(GL_POINTS);
     glVertex3f(0,0,0);
     glEnd();

     glPointSize(2);
     glBegin(GL_POINTS);
     for(unsigned int i=0; i<p_pos.size(); i++)
     {
         //float c = ((float)(i))/p_pos.size();
         glColor3f(0, 0, 0);
         glVertex3f(p_pos[i][0]-orig[0], p_pos[i][1]-orig[1], p_pos[i][2]-orig[2]);
     }
     glEnd();

     if(drawTrajectory)
     {
         glPointSize(3);
         glBegin(GL_POINTS);
         for(unsigned int i=0; i<positions.size(); i++)
         {
           // if(i%5 != 0)
           //     continue;

            // ROS_INFO_THROTTLE(1,"*** %d pose: %f %f %f orig: %f %f %f", i, positions[i][0],positions[i][1],positions[i][2],p_orig[0],p_orig[1],p_orig[2]);
             glColor3f(0.4, 0.5, 0.4);
             glVertex3f(positions[i][0]-p_orig[0], positions[i][1]-p_orig[1], zorder*(positions[i][2]-p_orig[2]));
         }
         glEnd();
     }

     glColor3f(0.4,0,1);
     
     glPointSize(drawImages?12:10);
     if(drawWaypoints)
     {
         glBegin(GL_POINTS);
         for(unsigned int i=0; i<waypoints.size();i++)
            glVertex3f(waypoints[i][0],waypoints[i][1],zorder*waypoints[i][2]);
         glEnd();

         glLineWidth(7);
         glBegin(GL_LINES);
         for(unsigned int i=0; i<waypoints.size()-1;i++)
         {
            TooN::Vector<3> c1 = GetColor(waypoints[i][2]);
            TooN::Vector<3> c2 = GetColor(waypoints[i+1][2]);

            glColor3f(c1[0],c1[1],c1[2]);
            glVertex3f(waypoints[i][0],waypoints[i][1],zorder*(waypoints[i][2]+0.01));
            glColor3f(c2[0],c2[1],c2[2]);
            glVertex3f(waypoints[i+1][0],waypoints[i+1][1],zorder*(waypoints[i+1][2]+0.01));
         }
         glEnd();
     }

     if(drawImages)
     {

         glColor4f(1,1,1,1);
         glPointSize(10);
         //double ep=0.1;
         double fl = 1.8;
         for(unsigned int i=0; i<waypoints.size();i++)
         {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, gluintsRaw[i]);
            glBegin(GL_QUADS);
//            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glTexCoord2f (0.0, 0.0);
//            glVertex3f(footprints[i][0]+ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glTexCoord2f (1.0, 0.0);
//            glVertex3f(footprints[i][2]-ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glTexCoord2f (1.0, 1.0);
//            glVertex3f(footprints[i][2]-ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glTexCoord2f (0.0, 1.0);

            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glTexCoord2f (0.0, 0.0);
            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glTexCoord2f (1.0, 0.0);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glTexCoord2f (1.0, 1.0);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glTexCoord2f (0.0, 1.0);

            glEnd();
            glDisable(GL_TEXTURE_2D);

            //glBindTexture(GL_TEXTURE_2D, gluintsRaw[i]);
//            glColor3f(1,1,1);
//            glLineWidth(5);
//            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//            glBegin(GL_POLYGON);
//            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,1/waypoints[i][2]);
//            glVertex3f(footprints[i][0]+ep,footprints[i][3]-ep,1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][3]-ep,1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][1]+ep,1/waypoints[i][2]);
//            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,1/waypoints[i][2]);
//            glEnd();
         }


         for(unsigned int i=0; i<waypoints.size();i++)
         {
            glColor3f(0,0,0);
            glLineWidth(3);
            //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//            glBegin(GL_LINES);
//            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][0]+ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][0]+ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][3]-ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][2]-ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,zorder*1/waypoints[i][2]);
//            glEnd();

            glBegin(GL_LINES);
            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]+fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] + fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glVertex3f(waypoints[i][0] - fl,waypoints[i][1]-fl,zorder*waypoints[i][2]);
            glEnd();
         }

     }

     if(drawSensedImages)
     {
         glEnable(GL_TEXTURE_2D);
         glColor4f(1,1,1,1);
         glPointSize(10);
         double ep=0.1;
         for(unsigned int i=0; i<waypoints.size();i++)
         {
            glBindTexture(GL_TEXTURE_2D, gluintsSensed[i]);
            glBegin(GL_QUADS);
            glVertex3f(footprints[i][0]+ep,footprints[i][1]+ep,zorder*waypoints[i][2]);
            glTexCoord2f (0.0, 0.0);
            glVertex3f(footprints[i][0]+ep,footprints[i][3]-ep,zorder*waypoints[i][2]);
            glTexCoord2f (1.0, 0.0);
            glVertex3f(footprints[i][2]-ep,footprints[i][3]-ep,zorder*waypoints[i][2]);
            glTexCoord2f (1.0, 1.0);
            glVertex3f(footprints[i][2]-ep,footprints[i][1]+ep,zorder*waypoints[i][2]);
            glTexCoord2f (0.0, 1.0);
            glEnd();
         }
         glDisable(GL_TEXTURE_2D);
     }



}

TooN::Vector<3> PlayLog::GetColor(double h)
{
    Vector<3> c;
    double small_dh=0.01;
    static std::vector<Vector<3> > colors;
    static std::vector<Vector<2> > h2c;

    if(colors.empty())
    {
        colors.push_back(makeVector(0,0,0));
        colors.push_back(makeVector(0.5,0.5,0.5));
        colors.push_back(makeVector(0,1,1));
        colors.push_back(makeVector(1,0,1));
        colors.push_back(makeVector(1,1,0));
        colors.push_back(makeVector(1,0,0));
        colors.push_back(makeVector(0,1,0));
        colors.push_back(makeVector(0,0,1));

        std::reverse(colors.begin(), colors.end());
    }

    if(h2c.empty())
    {
        Vector<2> i;
        i[0] = h;
        i[1] = 0;
        c = colors[0];
        h2c.push_back(i);
    }
    else
    {
        bool flag = false;
        for(unsigned int i=0; i<h2c.size(); i++)
        {
            if(fabs(h2c[i][0]-h) < small_dh)
            {
                c = colors[(int)h2c[i][1]];
                flag = true;
                break;
            }
        }

        if(!flag)
        {
            Vector<2> hc;
            hc[0] = h;
            hc[1] = h2c.size();
            h2c.push_back(hc);
            c = colors[h2c.size()];
        }
    }

    return c;
}

void PlayLog::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    updateKey = false;

    if(Key['-'])
        Clear();
    else if(Key['1'])
        drawImages = !drawImages;
    else if(Key['2'])
        drawSensedImages = !drawSensedImages;
    else if(Key['3'])
        drawTrajectory = !drawTrajectory;
    else if(Key['4'])
        drawWaypoints = !drawWaypoints;
    else if(Key['5'])
        ortho = !ortho;

//    if(key['`'])
//    {
//        world->PopulateWorld(20);
//    }

}

void PlayLog::idle()
{

    if(ros::ok())
    {
       // ROS_INFO_THROTTLE(0.5, "Ros spinning ...");
        ros::spinOnce();
    }
    else
    {
        exit(0);
    }

}

void PlayLog::mainLoop()
{

    glutInitWindowSize(800, 600);
    glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow("PositionVisualization");
    glClearColor(1,1,1,0);
    glEnable(GL_POINT_SMOOTH);

    // register callbacks
    glutDisplayFunc(render_event);
    glutReshapeFunc(resize_event);
    glutKeyboardFunc(keyboard_event);
    glutKeyboardUpFunc(keyboard_up_event);
    glutMouseFunc(mouse_event);
    glutTimerFunc(updateMS, update_event, updateMS);
    glutIdleFunc(idle_event);
    glutMotionFunc(mouse_drag_event);
    glutPassiveMotionFunc(mouse_move_event);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);
    // define global state
    //glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glutIgnoreKeyRepeat(true);

    translateCamera(0, 0, 30);
    rotateCamera(0,-1.57,0);

    // run glut
    //ros::spin();
    glutMainLoop();

//    while(true)
//    {
//        idle();
//        sleep(0.01);
//    }
}




int main(int argc, char ** argv)
{
    ros::init(argc, argv, "PositionVis");
    PlayLog::Instance(argc, argv)->mainLoop();
    return 0;
}
