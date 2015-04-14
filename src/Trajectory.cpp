#include "Trajectory.h"
#include <math.h>

int InsertWaypoint(double q[], double x, void *user_data)
{
    vector<Vector<2> > * wps = (vector<Vector<2> > *)user_data;
    Vector<2> v = makeVector(q[0], q[1]);
    wps->push_back(v);

    return 0;
}

int Trajectory::GenerateDubinTrajectory(Vector<2> s1, Vector<2> e1, Vector<2> s2, Vector<2> e2,
                                        double minR, double stepSize, vector<Vector<2> > &wps)
{

    DubinsPath * dp= new DubinsPath();
    double p1[] = {e1[0], e1[1], 0};
    double p2[] = {s2[0], s2[1], 0};

    p1[2] = atan2(e1[1]-s1[1], e1[0]-s1[0]);
    p2[2] = atan2(e2[1]-s2[1], e2[0]-s2[0]);

    dubins_init(p1, p2, minR, dp);
    dubins_path_sample_many(dp, InsertWaypoint, stepSize, &wps);

    int type = dubins_path_type(dp);

    delete dp;

    return type;
}
