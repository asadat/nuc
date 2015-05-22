
#include "InterestingnessSensor.h"
//#include "SuperPixelFeatures.h"
#include "ros/ros.h"
//#include "interestingness/ROIs.h"
#include "NUCParam.h"

//#include "opencv2/opencv.hpp"
using namespace cv;

InterestingnessSensor * InterestingnessSensor::instance;
int iLowH = 60;
int iHighH = 179;

int iHighH2 = 23;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;



InterestingnessSensor::InterestingnessSensor(ros::NodeHandle * nh_)
{
    //namedWindow( "Circle", CV_WINDOW_AUTOSIZE );
    //namedWindow( "output1", CV_WINDOW_AUTOSIZE );
    //namedWindow( "output2", CV_WINDOW_AUTOSIZE );
    namedWindow( "output3", CV_WINDOW_AUTOSIZE );

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  cvCreateTrackbar("HighH2", "Control", &iHighH2, 179);


  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //iHighH2 = 32;
  //iLowH = 60;
  //iLowV = 139;

    sensingCounter = 0;
    nh = nh_;
    interesting_label = NUCParam::interesting_label;
    //nh->param<std::string>("interesting_label", interesting_label, "human");
    human_interesting = (interesting_label == "human");

    ROS_INFO("interesting_label: %s", interesting_label.c_str());

    if(!human_interesting)
    {
        //nh->param<std::string>("decision_tree_file",dtreeFile, "");
        //nh->param<std::string>("training_set_dir",trainingSetDir, "");
        trainingSetDir = NUCParam::training_set_dir;

        image_transport::ImageTransport it(*nh);
        img_sub = it.subscribe("/ueyecamera/image_raw", 1, &InterestingnessSensor::imageCallback, this, image_transport::TransportHints("compressed", ros::TransportHints().tcpNoDelay(true)));
        TrainDTree();
    }
    else
    {
        //ROS_INFO("before int sub");
        //int_sub = nh->subscribe<interestingness::ROIs>("/interesting/regions", 10,&InterestingnessSensor::interestingCallback, this);
        //ROS_INFO("after int sub");
    }

    image_w = NUCParam::image_w;
    image_h = NUCParam::image_h;
    //nh->param<int>("image_w",image_w,640);
    //nh->param<int>("image_h",image_h,480);

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
bool InterestingnessSensor::InterestingColor(double b, double g, double r)
{
    ROS_INFO("COLOR: %f %f %f", b, g, r);
    if( r > 250 && r < 255 &&
        b > 250 && b < 255 &&
        g > 250 && g < 255)
    {
        return true;
    }

    return false;
}

void InterestingnessSensor::GetInterestingnessGrid(TooN::Matrix<10,10,int> & int_grd, int grd_s)
{
    sensingCounter++;
   // ROS_INFO("int_grid: test");
    //return;


    double grd_xstep = image_w/grd_s;
    double grd_ystep = image_h/grd_s;

    if(human_interesting) // humans are interesting
    {

        for(unsigned int j=0; j<ROIs.size(); j++)
        {
            if((ros::Time::now().toSec() - ROIs[j].first) > NUCParam::sensingTime)
                continue;

            std::vector<sensor_msgs::RegionOfInterest> rois = ROIs[j].second;
            for(unsigned int i=0; i<rois.size(); i++)
            {
                sensor_msgs::RegionOfInterest roi = rois[i];
                for( int grd_i=0; grd_i < grd_s; grd_i++)
                    for( int grd_j=0; grd_j < grd_s; grd_j++)
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
                            int_grd[grd_s-grd_i-1][grd_s-grd_j-1] +=1;
                        }
                    }

            }
        }
    }
    else // a specific label is interesting
    {

        if(imagePtr != NULL && (ros::Time::now()-imagePtr->header.stamp).toSec() < NUCParam::sensingTime)
        {
//            int interestingLabelIdx = -1;
//            for(unsigned int i=0; i<labels.size();i++)
//            {
//                if(interesting_label == std::string(labels[i]))
//                {
//                    ROS_INFO("Looking for \"%s\" ...", labels[i]);
//                    interestingLabelIdx = i;
//                    break;
//                }
//            }

            //{ count the white pixels
            for(int i=0; i<imgThresholded.cols; i++)
                for(int j=0; j<imgThresholded.rows; j++)
                {
                    if(imgThresholded.at<uchar>(j,i) > 250)
                    {
                        int grdx = floor(i/grd_xstep);
                        int grdy = floor(j/grd_ystep);
                        int_grd[grdy][grdx] += 1;
                    }
                }

            char outputf[128];
            sprintf(outputf, "%s%f-%d","image",ros::Time::now().toSec(),sensingCounter);
            std::string path(outputf);
            std::string imgpath = NUCParam::log_folder+"/"+path+".jpg";
            std::string predpath = NUCParam::log_folder+"/"+path+"-pred.jpg";

            cv::imwrite(predpath.c_str(), imgThresholded);
            cv::imwrite(imgpath.c_str(), imagePtr->image);

            //}

            //{ decision tree based
            /*
            vector<CvScalar> labelcolor;

            CvScalar cl[3];
            cl[0].val[0] = 100;cl[0].val[1] = 250;cl[0].val[2] = 100; // grass
            cl[2].val[0] = 200;cl[2].val[1] = 100;cl[2].val[2] = 100; // sand
            cl[1].val[0] = 0;  cl[1].val[1] = 50; cl[1].val[2] = 230; // carpet

            SuperPixelFeatures sptest(imagePtr->image);
            ROS_INFO("img format: %s", imagePtr->encoding.c_str());
            vector<cv::Mat> fs;
            sptest.GetSuperPixelFeatures(fs);

//            for(unsigned int i=0; i<fs.size(); i++)
//            {

//                double c = dtree.predict(fs[i].row(0))->value;
//                if(((int)c) == interestingLabelIdx)
//                {
//                    int x=0,y=0;
//                    if(sptest.GetSuperPixelCenter(i,x,y))
//                    {
//                        int grdx = floor(x/grd_xstep);
//                        int grdy = floor(y/grd_ystep);
//                        int_grd[grdy][grdx] += 1;
//                    }
//                }
//                labelcolor.push_back(cl[(int)c]);
//            }



            cv::Mat labelmap(imagePtr->image.rows, imagePtr->image.cols, CV_8UC3);
            //sptest.SuperPixelLabelMap(labelmap, labelcolor);
            sptest.GetSuperpixelAverageColor(labelmap);
            imshow("output3",labelmap);
            waitKey(30);

            for(unsigned int i=0; i<fs.size(); i++)
            {

                    int x=0,y=0, cc=0;
                    if(sptest.GetSuperPixelCenter(i,x,y))
                    {
                        CvScalar c = sptest.Get2D(labelmap,y,x);
                        //ROS_INFO("XY: %d %d", x, y);
                        if(InterestingColor(c.val[0],c.val[1],c.val[2]))
                        {
                            cc=1;
                            int grdx = floor(x/grd_xstep);
                            int grdy = floor(y/grd_ystep);
                            int_grd[grdy][grdx] += 1;
                        }
                    }

                     labelcolor.push_back(cl[(int)cc]);
            }

            sptest.SuperPixelLabelMap(labelmap, labelcolor);

            char outputf[128];
            sprintf(outputf, "%s%f-%d","image",ros::Time::now().toSec(),sensingCounter);
            std::string path(outputf);
            std::string imgpath = NUCParam::log_folder+"/"+path+".jpg";
            std::string predpath = NUCParam::log_folder+"/"+path+"-pred.jpg";

            //cv::Mat trp_img(imagePtr->image.cols, imagePtr->image.rows, CV_8UC3);
            //cv::Mat trp_pred(imagePtr->image.cols, imagePtr->image.rows, CV_8UC3);

            //cv::transpose(imagePtr->image, trp_img);
            //cv::transpose(labelmap, trp_pred);

            cv::imwrite(predpath.c_str(), imagePtr->image);
            cv::imwrite(imgpath.c_str(), labelmap);
            //decision tree bsed }*/

        }
        else
        {
            ROS_WARN("No recent image is available!");
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

/*void InterestingnessSensor::interestingCallback(const interestingness::ROIsConstPtr &msg)
{
   // ROS_INFO("int_grid: here1");
    if(ROIs.size() > 10)
    {
       // ROS_INFO("int_grid: here3");
        ROIs.erase(ROIs.begin());
    }

    ROIs.push_back(std::pair<double, std::vector<sensor_msgs::RegionOfInterest > >(ros::Time::now().toSec(), msg->regions));
    //ROS_INFO("int_grid: here2");
}*/

void InterestingnessSensor::runCircleDetection(Mat src)
{
    using namespace cv;

     Mat src_gray;

      /// Read the image
     //src = imread("/home/autolab/test.jpg", 1 );

      if( !src.data )
        { return;}

      /// Convert it to gray
      cvtColor( src, src_gray, CV_BGR2GRAY );

      /// Reduce the noise so we avoid false circle detection
      GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

      vector<Vec3f> circles;

      /// Apply the Hough Transform to find the circles
      HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/16, 40, 40, 5, 100 );

      /// Draw the circles detected
      ROS_INFO("circles: %d", (int)circles.size());
      for( size_t i = 0; i < circles.size(); i++ )
      {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
       }

      /// Show your results
      //imshow("Hough Circle Transform Demo", src );

      waitKey(30);

}

void InterestingnessSensor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imagePtr = cv_bridge::toCvCopy(msg, "bgr8");
    imagePtr->header.stamp = ros::Time::now();

    Mat imgHSV;
    Mat imgThreshTmp;
    //Mat imgThresholded;

    cvtColor(imagePtr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    inRange(imgHSV, Scalar(0, iLowS, iLowV), Scalar(iHighH2, iHighS, iHighV), imgThreshTmp); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    erode(imgThreshTmp, imgThreshTmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThreshTmp, imgThreshTmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    dilate( imgThreshTmp, imgThreshTmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThreshTmp, imgThreshTmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    cv::add(imgThreshTmp, imgThresholded, imgThresholded);

    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    waitKey(30);


    //  imshow("Original", imgOriginal); //show the original image



    //imshow("output1",imagePtr->image);
   // runCircleDetection(imagePtr->image);
}

void InterestingnessSensor::TrainDTree()
{
//    //system("pwd");
//    bool loadedTree = false;
//    vector<int> train_size;
//    vector<vector<cv::Mat> > features;

//    char label[32];
//    FILE* f = fopen((trainingSetDir+"/labels.txt").c_str(), "r");
//    if(!f)
//    {
//        ROS_INFO("ERROR reading the %s",(trainingSetDir+"/labels.txt").c_str());
//    }

//    int train_n = 0;
//    //printf("Reading ....");

//    while(fscanf(f, "%s %d\n", label, &train_n ) != EOF)
//    {
//        char* str = new char(strlen(label)+1);
//        strcpy(str,label);
//        labels.push_back(str);
//        train_size.push_back(train_n);
//        ROS_INFO("Sample: %s %d \n", label, train_n);
//    }
//    fclose(f);

//    if(this->exists_test((trainingSetDir+"/decision_tree").c_str()))
//    {
//        dtree.load((trainingSetDir+"/decision_tree").c_str());
//        loadedTree = true;
//    }
//    else
//    {
//        int nrows = 0;

//        for(unsigned int i=0; i<labels.size(); i++)
//        {
//            for( int j=0; j<train_size[i]; j++)
//            {
//                char num[256];
//                sprintf(num, "%s/%s%d.jpg",trainingSetDir.c_str(),labels[i],j);

//                ROS_INFO("reading: %s %d\n", num, j);
//                cv::Mat img = cv::imread(num);

//                SuperPixelFeatures sp(img);
//                vector<cv::Mat> ftv;
//                sp.GetSuperPixelFeatures(ftv);
//                features.push_back(ftv);
//                nrows += ftv.size();
//            }
//        }

//        cv::Mat trainMat(nrows, 4, CV_32F);
//        cv::Mat trainRe(nrows, 1, CV_32F);

//        int it=0;
//        int imgn = 0;
//        for(unsigned int i=0; i<labels.size(); i++)
//        {
//            for( int j=0; j<train_size[i]; j++)
//            {
//                for(unsigned int k=0; k<features[imgn].size(); k++)
//                {
//                    trainRe.at<float>(it) = i;
//                    trainMat.at<float>(it,0) = features[imgn][k].at<float>(0,0);
//                    trainMat.at<float>(it,1) = features[imgn][k].at<float>(0,1);
//                    trainMat.at<float>(it,2) = features[imgn][k].at<float>(0,2);
//                    trainMat.at<float>(it,3) = features[imgn][k].at<float>(0,3);

//                    it++;
//                }
//                imgn++;
//            }
//        }

//        dtree.train(trainMat, CV_ROW_SAMPLE, trainRe);
//    }


////    ros::Time t1 = ros::Time::now();
////    TestDTree("test9.jpg");
////    ROS_INFO("dt: %f", (ros::Time::now()-t1).toSec());
////    t1 = ros::Time::now();
////    TestDTree("test8.jpg");
////    ROS_INFO("dt: %f", (ros::Time::now()-t1).toSec());

//    if(!loadedTree)
//    {
//        dtree.save((trainingSetDir+"/decision_tree").c_str());
//    }

////    int n=0;
////    for(int i=0; i<nrows; i++)
////    {
////        n+= ((dtree.predict(trainMat.row(i))->value - trainRe.at<float>(i))<0.001)?1:0;
////    }
////    printf("Test Data: %d out of %d\n",n,nrows);
}

void InterestingnessSensor::TestDTree(char *filename)
{
//    CvScalar cl[3];
//    cl[0].val[0] = 100;cl[0].val[1] = 250;cl[0].val[2] = 100; // grass
//    cl[1].val[0] = 200;cl[1].val[1] = 150;cl[1].val[2] = 100; // sand
//    cl[2].val[0] = 0;cl[2].val[1] = 100;cl[2].val[2] = 0; // tree

//    cv::Mat testMat= cv::imread(filename);
//    SuperPixelFeatures sptest(testMat);
//    vector<cv::Mat> fs;
//    sptest.GetSuperPixelFeatures(fs);
//    vector<CvScalar> labelcolor;
//    for(unsigned int i=0; i<fs.size(); i++)
//    {

//        double c = dtree.predict(fs[i].row(0))->value;
//        labelcolor.push_back(cl[(int)c]);
//    }

//    cv::Mat labelmap(testMat.rows, testMat.cols, CV_8UC3);
//    sptest.SuperPixelLabelMap(labelmap, labelcolor);
//    char outputf[128];
//    sprintf(outputf, "%s%s","output-",filename);
//    cv::imwrite(outputf, labelmap);

}
