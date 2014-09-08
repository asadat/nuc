
#include "SuperPixelFeatures.h"

SuperPixelFeatures::SuperPixelFeatures(cv::Mat &img_):Slic()
{
    img.copySize(img_);
    cv::cvtColor(img_,img,CV_BGR2Lab);
    //img_.copyTo(img);

}

SuperPixelFeatures::~SuperPixelFeatures()
{

}

void SuperPixelFeatures::GetSuperPixelFeatures(vector<cv::Mat> &features)
{
    //cv::Mat lab_image(img);
    //cv::cvtColor(img, lab_image, CV_BGR2Lab);

    int w = img.cols, h = img.rows;
    int nr_superpixels = 2000;
    int nc = 100;
    double step = sqrt((w * h) / (double) nr_superpixels);
    generate_superpixels(img,step,nc);
    PopulateFeatures(features);
}


void SuperPixelFeatures::PopulateFeatures(vector<cv::Mat> &features)
{
    vector<CvScalar> colours(centers.size());
    vector<double> meanIntensity(centers.size());

    /* Gather the colour values per cluster. */
    for (int i = 0; i < img.cols; i++) {
        for (int j = 0; j < img.rows; j++) {
            int index = clusters[i][j];
            CvScalar colour = Get2D(img, j, i);

            colours[index].val[0] += colour.val[0];
            colours[index].val[1] += colour.val[1];
            colours[index].val[2] += colour.val[2];

        }
    }

//    img.copyTo(meanImg);

//    for (int i = 0; i < img.cols; i++) {
//        for (int j = 0; j < img.rows; j++) {
//            int index = clusters[i][j];

//            colours[index].val[0] / center_counts[index];
//            colours[index].val[1] / center_counts[index];
//            colours[index].val[2] / center_counts[index];

//        }
//    }

    for (int i = 0; i < (int)colours.size(); i++) {
        colours[i].val[0] /= center_counts[i];
        colours[i].val[1] /= center_counts[i];
        colours[i].val[2] /= center_counts[i];
        meanIntensity[i] = colours[i].val[0]+colours[i].val[1]+colours[i].val[2];

        cv::Mat featurevec(1,4,CV_32F);
        featurevec.at<float>(0,0) = colours[i].val[0];
        featurevec.at<float>(0,1) = colours[i].val[1];
        featurevec.at<float>(0,2) = colours[i].val[2];
        featurevec.at<float>(0,3) = meanIntensity[i];
        features.push_back(featurevec);

        //printf("FS: %f %f %f %f %d \n",colours[i].val[0],colours[i].val[1],colours[i].val[2], meanIntensity[i], i);
    }
}

void SuperPixelFeatures::GetSuperpixelAverageColor(Mat &spmat)
{
    Mat tmp;
    img.copyTo(tmp);
    this->colour_with_cluster_means(tmp);
    cvtColor(tmp,spmat,CV_Lab2BGR);
}

void SuperPixelFeatures::SuperPixelLabelMap(cv::Mat &labelMap, std::vector<CvScalar> &labelColor)
{
    //labelMap.resize(img.size());
    for (int i = 0; i < img.cols; i++) {
        for (int j = 0; j < img.rows; j++) {
            int index = clusters[i][j];
            CvScalar colour = labelColor[index];
            cv::Vec3b cl;
            cl[0] = colour.val[0];cl[1] = colour.val[1];cl[2] = colour.val[2];
            labelMap.at<cv::Vec3b>(j,i) = cl;
        }
    }

}

bool SuperPixelFeatures::GetSuperPixelCenter(int super_pixel_i, int &x, int &y)
{
    if(super_pixel_i >= SPCount())
    {
        return false;
    }
    else
    {
        x = centers[super_pixel_i][3];
        y = centers[super_pixel_i][4];
    }
    return true;
}
