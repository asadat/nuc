#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    system("pwd");
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("ueyecamera/image_raw", 1);

    //cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
    //sensor_msgs::ImagePtr msg = cv_bridge:: cvToImgMsg(image.Ipl(), "bgr8");
    int n = atoi(argv[2]);
    int hz = atoi(argv[3]);
    int i=100;

    ros::Rate loop_rate(hz);

    char imgName[256];
    ROS_INFO("%s %d %d", argv[1], n, hz);

    while (nh.ok())
    {
        if(i-100>n)
        {
            i=100;
        }

        i++;
        char tmp[5];
        if(i<10)
            sprintf(tmp,"%s","00");
        else if(i<100)
            sprintf(tmp,"%s","0");
        else
            sprintf(tmp,"%s","");

        sprintf(imgName,"%s%s%d.jpg", argv[1], tmp, i);
        ROS_INFO("image: %s", imgName);
        cv_bridge::CvImage img;
        cv::Mat mt = cv::imread(imgName);
        cv::cvtColor(mt,img.image,CV_RGB2RGBA);
        img.encoding="bgra8";
        img.header.stamp = ros::Time::now();
        //mt.copyTo(img.image);
        pub.publish(img.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
}
