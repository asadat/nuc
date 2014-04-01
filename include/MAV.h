#ifndef _MAV_
#define _MAV_
#include "TooN/TooN.h"

class MAV
{
public:
    MAV();
    ~MAV();

    void glDraw();
    void SetGoal(TooN::Vector<3> goalpos);
    void Update(double dt);
    bool AtGoal(){return atGoal;}
    static void ChangeSpeed(double ds){speed +=ds;}
    static double speed;
private:
    TooN::Vector<3> pos;
    TooN::Vector<3> goal;
    TooN::Vector<3> toGoalNorm;
    bool atGoal;
};

#endif
