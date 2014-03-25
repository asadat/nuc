
#include "NUC.h"
#include <string.h>
#include "GL/glut.h"
#include <map>

#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include "TooN/TooN.h"
#include <ros/ros.h>


using namespace TooN;

NUC *nucObj;

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

    //NUC::Instance()->hanldeKeyPressed(Key, doUpdate);
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
 NUC::Instance()->idle();

    //if(!ros::ok())
    // exit(0);
}



void render_event()
{
    if(!nucObj->VisEnabled())
        return;

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



   NUC::Instance()->glDraw();

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

void mainLoop()
{

    glutInitWindowSize(800, 600);
    glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow("NUCualization");
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
    ros::init(argc, argv, "NUC");
    NUC::Instance(argc, argv);
    nucObj = NUC::Instance();
    mainLoop();
    return 0;
}
