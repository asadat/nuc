#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "dubins.h"
#include "TooN/TooN.h"
#include <vector>

using namespace std;
using namespace TooN;

class Trajectory
{
public:
    static int GenerateDubinTrajectory(Vector<2> s1, Vector<2> e1, Vector<2> s2, Vector<2> e2, double minR, double stepSize, vector<Vector<2> > &wps);
};

#endif
