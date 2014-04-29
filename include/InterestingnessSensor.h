#ifndef _INTERESTINGNESS_SENSOR_
#define _INTERESTINGNESS_SENSOR_
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/ml/ml.hpp"
#include "interestingness/ROIs.h"
#include "TooN/TooN.h"

class InterestingnessSensor
{
public:
    ~InterestingnessSensor();

    static InterestingnessSensor * Instance(ros::NodeHandle * nh_=NULL)
    {
        if(instance == NULL && nh_!=NULL)
        {
            instance = new InterestingnessSensor(nh_);            
        }

        return instance;
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void interestingCallback(const interestingness::ROIsConstPtr &msg);
    void TrainDTree();

    void GetInterestingnessGrid(TooN::Matrix<10,10,int> & int_grd, int grd_s);

private:

    InterestingnessSensor(ros::NodeHandle * nh_);
    static InterestingnessSensor *instance;
    void TestDTree(char* filename);
    inline bool exists_test(const std::string& name) {
        return ( access( name.c_str(), F_OK ) != -1 );
    }

    int image_w,image_h;

    ros::Subscriber int_sub;
    image_transport::Subscriber img_sub;
    ros::NodeHandle * nh;
    cv_bridge::CvImagePtr imagePtr;

    std::vector< std::vector<sensor_msgs::RegionOfInterest> > ROIs;
    cv::DecisionTree dtree;
    std::vector<char*> labels;

    bool human_interesting;
    std::string interesting_label;
    std::string dtreeFile;
    std::string trainingSetDir;

};

#endif
