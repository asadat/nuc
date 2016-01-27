#ifndef _TARGET_TOUR_
#define _TARGET_TOUR_

#include <TargetPolygon.h>
#include <vector>
#include "NUCParam.h"

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))
#define D2(a,b) (a-b)*(a-b)
#define D1(a,b) sqrt((a-b)*(a-b))

#define COLLINEAR(a,b,c) ((fabs(fabs(ANGLE(a,b,c))-3.14) > 0.1)?1.0:0)
#define IN(a,r) ((a[0]>= r[0] && a[0] <= r[2]) && (a[1] >= r[1] && a[1] <= r[3]))

using namespace std;
using namespace TooN;

class TargetTour
{
    public:
        TargetTour(){}
        ~TargetTour(){}

        static double GetPlanExecutionTime(std::vector<TooN::Vector<3> > & wps, const TooN::Vector<3>& curpos,
                                           const TooN::Vector<3>& endpos, bool initalTurn, bool endTurn);

        static double GetPlanExecutionTime(std::vector<CNode*> & wps, const TooN::Vector<3>& curpospos,
                                           const TooN::Vector<3>& endpos, bool initalTurn, bool endTurn);

        static double GetPlanExecutionTime(std::vector<TooN::Vector<3> > & wps, bool ignoreFirstSegment, bool ignoreLastSegment);

        static double GetTargetTour(vector<TargetPolygon*> &targets, const Vector<3>& start, const Vector<3>& end);
        static double GetTargetTour_Utility(vector<TargetPolygon*> &targets, const Vector<3>& start, const Vector<3>& end);

    private:

        static double GetTourCost(vector<TargetPolygon*> &targets, const Vector<3>& start, const Vector<3>& end);

};

#endif
