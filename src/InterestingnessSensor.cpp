
#include "InterestingnessSensor.h"
#include "SuperPixelFeatures.h"

//#include "opencv2/opencv.hpp"

InterestingnessSensor * InterestingnessSensor::instance;

InterestingnessSensor::InterestingnessSensor(ros::NodeHandle * nh_)
{
    nh = nh_;
    image_transport::ImageTransport it(*nh);
    img_sub = it.subscribe("camera/image_raw", 1, &InterestingnessSensor::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));

    TrainDTree();

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

void InterestingnessSensor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imagePtr = cv_bridge::toCvCopy(msg);
}

void InterestingnessSensor::TrainDTree()
{
    vector<int> train_size;
    vector<vector<cv::Mat> > features;

    char label[32];
    FILE* f = fopen("labels.txt", "r");
    int train_n = 0;
    while(fscanf(f, "%s %d\n", label, &train_n ) != EOF)
    {
        char* str = new char(strlen(label)+1);
        strcpy(str,label);
        labels.push_back(str);
        train_size.push_back(train_n);
        printf("Sample: %s %d\n", label, train_n);
    }
    fclose(f);

    int nrows = 0;

    for(int i=0; i<labels.size(); i++)
    {
        for(int j=0; j<train_size[i]; j++)
        {
            char num[50];
            sprintf(num, "%s%d.jpg",labels[i],j);

            printf("reading: %s %d\n", num, j);
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

    cv::Mat testMat= cv::imread("test1.jpg");
    SuperPixelFeatures sptest(testMat);
    vector<cv::Mat> fs;
    sptest.GetSuperPixelFeatures(fs);
    vector<CvScalar> labelcolor;
    for(int i=0; i<fs.size(); i++)
    {
        double c = dtree.predict(fs[i].row(0))->value;
        CvScalar cl;
        cl.val[0] = c*150;
        cl.val[1] = 0;
        cl.val[2] = c*150;
        labelcolor.push_back(cl);
    }

    cv::Mat labelmap(testMat.rows, testMat.cols, CV_8UC3);
    sptest.SuperPixelLabelMap(labelmap, labelcolor);
    cv::imwrite("ouput.jpg", labelmap);


//    int n=0;
//    for(int i=0; i<nrows; i++)
//    {
//        n+= ((dtree.predict(trainMat.row(i))->value - trainRe.at<float>(i))<0.001)?1:0;
//    }
//    printf("Test Data: %d out of %d\n",n,nrows);
}
