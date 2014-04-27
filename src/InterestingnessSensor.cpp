
#include "InterestingnessSensor.h"
#include "SuperPixelFeatures.h"
#include "ros/ros.h"
#include "interestingness/ROIs.h"

//#include "opencv2/opencv.hpp"

InterestingnessSensor * InterestingnessSensor::instance;

InterestingnessSensor::InterestingnessSensor(ros::NodeHandle * nh_)
{
    nh = nh_;
    image_transport::ImageTransport it(*nh);
    img_sub = it.subscribe("camera/image_raw", 1, &InterestingnessSensor::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));

    ROS_INFO("before int sub");
    int_sub = nh->subscribe<interestingness::ROIs>("/interesting/regions", 10,&InterestingnessSensor::interestingCallback, this);
    ROS_INFO("after int sub");

   // TrainDTree();

    nh->param<int>("image_w",image_w,640);
    nh->param<int>("image_h",image_h,480);

//    cv::Mat data(100,5,CV_32F);
//    cv::Mat res(100,1,CV_32F);

//    for(int i=0; i<100; i++)
//    {
//        int r = (int)floor(i/20);
//        //ROS_INFO("ROW: %d", r);
//        data.at<float>(i, r) = rand()%100;
//        res.at<float>(i) = r;
//    }

//    dtree.train(data, CV_ROW_SAMPLE, res);

//    int n=0;
//    for(int i=0; i<100; i++)
//    {
//       n+= ((dtree.predict(data.row(i))->value - (int)floor(i/20))<0.001)?1:0;
//    }

//    ROS_INFO("DTREE: correct response: %d", n);

}

InterestingnessSensor::~InterestingnessSensor()
{
    while(!labels.empty())
    {
        char* s = labels.back();
        labels.pop_back();
        delete [] s;
    }
}

void InterestingnessSensor::GetInterestingnessGrid(TooN::Matrix<10,10,int> & int_grd, int grd_s)
{
   // ROS_INFO("int_grid: test");
    //return;


    double grd_xstep = image_w/grd_s;
    double grd_ystep = image_h/grd_s;

    std::vector<sensor_msgs::RegionOfInterest> rois = ROIs.back();
    for(int i=0; i<rois.size(); i++)
    {
        sensor_msgs::RegionOfInterest roi = rois[i];
        for(int grd_i=0; grd_i < grd_s; grd_i++)
            for(int grd_j=0; grd_j < grd_s; grd_j++)
            {
                double grdl,grdr,grdt,grdd;
                grdl = grd_i*grd_xstep;
                grdr = grdl+grd_xstep;
                grdt =grd_j*grd_ystep;
                grdd = grdt + grd_ystep;

                bool int_cell = !( roi.x_offset > grdr
                       || roi.x_offset+roi.width < grdl
                       || roi.y_offset > grdd
                       || roi.y_offset+roi.height < grdt );
                if(int_cell)
                {
                    int_grd[grd_j][grd_i] +=1;
                }
            }

    }

    ROS_INFO("int_grid: START");
    for(int grd_j=0; grd_j < grd_s; grd_j++)
    {
        for(int grd_i=0; grd_i < grd_s; grd_i++)
        {
            printf("%d\t", int_grd[grd_j][grd_i]);
        }
        printf("\n");
    }
   ROS_INFO("int_grid: END");
}

void InterestingnessSensor::interestingCallback(const interestingness::ROIsConstPtr &msg)
{
   // ROS_INFO("int_grid: here1");
    if(ROIs.size() > 10)
    {
       // ROS_INFO("int_grid: here3");
        ROIs.erase(ROIs.begin());
    }

    ROIs.push_back(msg->regions);
    //ROS_INFO("int_grid: here2");
}

void InterestingnessSensor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imagePtr = cv_bridge::toCvCopy(msg);
}

void InterestingnessSensor::TrainDTree()
{
    bool loadedTree = false;
    vector<int> train_size;
    vector<vector<cv::Mat> > features;

    char label[32];
    FILE* f = fopen("labels.txt", "r");
    int train_n = 0;
    //printf("Reading ....");

    while(fscanf(f, "%s %d\n", label, &train_n ) != EOF)
    {
        char* str = new char(strlen(label)+1);
        strcpy(str,label);
        labels.push_back(str);
        train_size.push_back(train_n);
        ROS_INFO("Sample: %s %d \n", label, train_n);
    }
    fclose(f);

    if(this->exists_test("decision_tree"))
    {
        dtree.load("decision_tree");
        loadedTree = true;
    }
    else
    {
        int nrows = 0;

        for(int i=0; i<labels.size(); i++)
        {
            for(int j=0; j<train_size[i]; j++)
            {
                char num[50];
                sprintf(num, "%s%d.jpg",labels[i],j);

                ROS_INFO("reading: %s %d\n", num, j);
                cv::Mat img = cv::imread(num);
                SuperPixelFeatures sp(img);
                vector<cv::Mat> ftv;
                sp.GetSuperPixelFeatures(ftv);
                features.push_back(ftv);
                nrows += ftv.size();
            }
        }

        cv::Mat trainMat(nrows, 4, CV_32F);
        cv::Mat trainRe(nrows, 1, CV_32F);

        int it=0;
        int imgn = 0;
        for(int i=0; i<labels.size(); i++)
        {
            for(int j=0; j<train_size[i]; j++)
            {
                for(int k=0; k<features[imgn].size(); k++)
                {
                    trainRe.at<float>(it) = i;
                    trainMat.at<float>(it,0) = features[imgn][k].at<float>(0,0);
                    trainMat.at<float>(it,1) = features[imgn][k].at<float>(0,1);
                    trainMat.at<float>(it,2) = features[imgn][k].at<float>(0,2);
                    trainMat.at<float>(it,3) = features[imgn][k].at<float>(0,3);

                    it++;
                }
                imgn++;
            }
        }

        dtree.train(trainMat, CV_ROW_SAMPLE, trainRe);
    }


    ros::Time t1 = ros::Time::now();
    TestDTree("test9.jpg");
    ROS_INFO("dt: %f", (ros::Time::now()-t1).toSec());
    t1 = ros::Time::now();
    TestDTree("test8.jpg");
    ROS_INFO("dt: %f", (ros::Time::now()-t1).toSec());

    if(!loadedTree)
    {
        dtree.save("decision_tree");
    }

//    int n=0;
//    for(int i=0; i<nrows; i++)
//    {
//        n+= ((dtree.predict(trainMat.row(i))->value - trainRe.at<float>(i))<0.001)?1:0;
//    }
//    printf("Test Data: %d out of %d\n",n,nrows);
}

void InterestingnessSensor::TestDTree(char *filename)
{
    CvScalar cl[3];
    cl[0].val[0] = 100;cl[0].val[1] = 250;cl[0].val[2] = 100; // grass
    cl[1].val[0] = 200;cl[1].val[1] = 150;cl[1].val[2] = 100; // sand
    cl[2].val[0] = 0;cl[2].val[1] = 100;cl[2].val[2] = 0; // tree

    cv::Mat testMat= cv::imread(filename);
    SuperPixelFeatures sptest(testMat);
    vector<cv::Mat> fs;
    sptest.GetSuperPixelFeatures(fs);
    vector<CvScalar> labelcolor;
    for(int i=0; i<fs.size(); i++)
    {

        double c = dtree.predict(fs[i].row(0))->value;
        labelcolor.push_back(cl[(int)c]);
    }

    cv::Mat labelmap(testMat.rows, testMat.cols, CV_8UC3);
    sptest.SuperPixelLabelMap(labelmap, labelcolor);
    char outputf[128];
    sprintf(outputf, "%s%s","output-",filename);
    cv::imwrite(outputf, labelmap);

}
