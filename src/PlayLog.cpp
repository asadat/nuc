
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

    gluLookAt(Cam_Target[0], Cam_Target[1], Cam_Target[2],
            Cam_Plane[0], Cam_Plane[1], Cam_Plane[2],
            Cam_Normal[0], Cam_Normal[1], Cam_Normal[2]);



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
        while(fscanf(f,"%f %f %f %d %d %d %f %f %f %f", &p[0], &p[1], &p[2], &d[0], &d[1], &d[2], &r[0], &r[1], &r[2], &r[3]) != EOF)
        {
            Vector<3> wp = makeVector(p[0], p[1], p[2]);
            waypoints.push_back(wp);
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
     float w=10;
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

     glPointSize(2);
     glBegin(GL_POINTS);
     for(unsigned int i=0; i<positions.size(); i++)
     {
         glColor3f(1, 0, 0);
         glVertex3f(positions[i][0]-p_orig[0], positions[i][1]-p_orig[1], positions[i][2]-p_orig[2]);
     }
     glEnd();

     if(!positions.empty())
     {

         glLineWidth(3);
         glBegin(GL_LINES);
         glColor3f(1,0,0);

         Vector<3> ep = makeVector(1,0,0);
         Vector<3> p = positions.back();
         p[0] -= p_orig[0];
         p[1] -= p_orig[1];
         p[2] -= p_orig[2];

         Vector<4> q = curAtt;//p_att.back();
         tf::Quaternion qu(q[0], q[1], q[2], q[3]);
         tf::Matrix3x3 m(qu);
         Matrix<3> rot;
         rot[0][0] = m[0][0]; rot[1][0] = m[1][0]; rot[2][0] = m[2][0];
         rot[0][1] = m[0][1]; rot[1][1] = m[1][1]; rot[2][1] = m[2][1];
         rot[0][2] = m[0][2]; rot[1][2] = m[1][2]; rot[2][2] = m[2][2];

         glColor3f(1,0,0);
         ep = rot*ep+p;
         glVertex3f(p[0], p[1], p[2]);
         glVertex3f(ep[0], ep[1], ep[2]);

	 glColor3f(0,1,0);
         ep = rot*makeVector(0,1,0)+p;
         glVertex3f(p[0], p[1], p[2]);
         glVertex3f(ep[0], ep[1], ep[2]);

	 glColor3f(0,0,1);
         ep = rot*makeVector(0,0,1)+p;
         glVertex3f(p[0], p[1], p[2]);
         glVertex3f(ep[0], ep[1], ep[2]);

         glColor3f(0,1,0);
         Vector<3> velDir = p + 5*vel;
         glVertex3f(p[0], p[1], p[2]);
         glVertex3f(velDir[0], velDir[1], velDir[2]);

         glEnd();
     }

     glColor3f(0,0,1);
     glPointSize(10);
     glBegin(GL_POINTS);
     for(unsigned int i=0; i<waypoints.size();i++)
        glVertex3f(waypoints[i][0],waypoints[i][1],waypoints[i][2]);
     glEnd();


}

void PlayLog::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    updateKey = true;

    if(Key['-'])
        Clear();

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
