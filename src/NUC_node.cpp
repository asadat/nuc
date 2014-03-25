
#include "PositionVis.h"
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

PositionVis * PositionVis::instance = NULL;
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

    PositionVis::Instance()->hanldeKeyPressed(Key, doUpdate);
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
 PositionVis::Instance()->idle();

    //if(!ros::ok())
    // exit(0);
}



void render_event()
{
    ROS_INFO_THROTTLE(0.5,"gl rendering ...");
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



   PositionVis::Instance()->glDraw();

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

PositionVis::PositionVis(int argc, char **argv)
{


    gpsPos_sub = nh.subscribe("/fcu/gps_position", 100, &PositionVis::gpsPositionCallback, this);
    gpsPose_sub = nh.subscribe("/fcu/gps_pose", 100, &PositionVis::gpsPoseCallback, this);



    glutInit(&argc, argv);
}

PositionVis::~PositionVis()
{

}
void PositionVis::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    Vector<3> p;
    p[0] = msg->pose.pose.position.x;
    p[1] = msg->pose.pose.position.y;
    p[2] = msg->pose.pose.position.z;

    p_pos.push_back(p);

    Vector<4> att;
    att[0] = msg->pose.pose.orientation.x;
    att[1] = msg->pose.pose.orientation.y;
    att[2] = msg->pose.pose.orientation.z;
    att[3] = msg->pose.pose.orientation.w;

    p_att.push_back(att);

}

void PositionVis::gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg)
{
    Vector<3> p;
    p[0] = msg->position.x;
    p[1] = msg->position.y;
    p[2] = msg->position.z;

    positions.push_back(p);

}

void PositionVis::glDraw()
{
     float w=10;
     glLineWidth(2);
     glColor3f(0,0,0);
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


//    glPointSize(5);
//    glBegin(GL_POINTS);
//    for(int i=0; i<positions.size(); i++)
//    {
//        float c = ((float)(i))/positions.size();
//        glColor3f(1-c, 1-c, 1-c);
//        glVertex3f(positions[i][0], positions[i][1], 10+positions[i][2]);
//    }
//    glEnd();

     glPointSize(5);
     glBegin(GL_POINTS);
     for(int i=0; i<p_pos.size(); i++)
     {
         float c = ((float)(i))/positions.size();
         glColor3f(1-c, 1-c, 1-c);
         glVertex3f(p_pos[i][0], p_pos[i][1], 10+p_pos[i][2]);
     }
     glEnd();

     if(!p_pos.empty())
     {

         glLineWidth(3);
         glBegin(GL_LINES);
         glColor3f(1,0,0);

         Vector<3> ep = makeVector(1,0,0);
         Vector<3> p = p_pos.back();
         Vector<4> q = p_att.back();
         tf::Quaternion qu(q[0], q[1], q[2], q[3]);
         tf::Matrix3x3 m(qu);
         Matrix<3> rot;
         rot[0][0] = m[0][0]; rot[1][0] = m[1][0]; rot[2][0] = m[2][0];
         rot[0][1] = m[0][1]; rot[1][1] = m[1][1]; rot[2][1] = m[2][1];
         rot[0][2] = m[0][2]; rot[1][2] = m[1][2]; rot[2][2] = m[2][2];

         ep = rot*ep+p;
         glVertex3f(p[0], p[1], 10+p[2]);
         glVertex3f(ep[0], ep[1], 10+ep[2]);

         ep = rot*makeVector(0,1,0)+p;
         glVertex3f(p[0], p[1], 10+p[2]);
         glVertex3f(ep[0], ep[1], 10+ep[2]);

         ep = rot*makeVector(0,0,1)+p;
         glVertex3f(p[0], p[1], 10+p[2]);
         glVertex3f(ep[0], ep[1], 10+ep[2]);

         glEnd();
     }
}

void PositionVis::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    updateKey = true;

//    if(key['`'])
//    {
//        world->PopulateWorld(20);
//    }

}

void PositionVis::idle()
{

    if(ros::ok())
    {
        ROS_INFO_THROTTLE(0.5, "Ros spinning ...");
        ros::spinOnce();
    }
    else
    {
        exit(0);
    }

}

void PositionVis::mainLoop()
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

//int main(int argc, char **argv)
//{

//   // printf("\n");
//   // printf("\t1 ....... Toggle Environment Drawing\n");
//   // printf("\t2 ....... Toggle Sensing\n\n");
//    SamplingSim::Instance(argc, argv);
//    SamplingSim::Instance()->mainLoop();

//    return 0;
//}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "PositionVis");
    PositionVis::Instance(argc, argv)->mainLoop();
    return 0;
}
