#ifndef _SUPER_PIXELFEATURES_
#define _SUPER_PIXELFEATURES_
#include "slic.h"

class SuperPixelFeatures : public Slic
{
public:
    SuperPixelFeatures(cv::Mat &img_);
    ~SuperPixelFeatures();
    void GetSuperPixelFeatures(std::vector<cv::Mat> &features);
    int SPCount(){return centers.size();}
    void SuperPixelLabelMap(cv::Mat &labelMap, std::vector<CvScalar> &labelColor);
    void GetSuperpixelAverageColor(cv::Mat &spmat);
    bool GetSuperPixelCenter(int super_pixel_i, int &x, int &y);
private:

    void PopulateFeatures(std::vector<cv::Mat> &features);

    cv::Mat meanImg;
    cv::Mat img;
};

#endif
