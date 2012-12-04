#ifndef DSVISUALMETHOFOPENCV_H
#define DSVISUALMETHOFOPENCV_H
#include "detectorscan.h"

class dsVisualMethOfOpenCV : public DetectorScan
{
public:
    dsVisualMethOfOpenCV();
    dsVisualMethOfOpenCV(IplImage* image);
    dsVisualMethOfOpenCV(const cv::Mat& image);
    ~dsVisualMethOfOpenCV();

    IplImage* SelectErodePixel(IplImage* image, METHODERODE method = ERODE);
    void dsSelectErodePixel(cv::Mat &image, METHODERODE method = ERODE);

    IplImage* dsVisualCannyAlgorithm(IplImage* image, int *parameters);
    cv::Mat dsVisualCannyAlgorithm(const cv::Mat& image1ch, int* parameters);

    IplImage* VisualSelectChannel(IplImage* image3ch);
    void VisualSelectChannel(cv::Mat& image3ch);

    IplImage* SelectThreshold(IplImage* image1ch);
    void SelectThreshold(cv::Mat &image1ch);

    /// BEGIN TEMP METHODS
    void temprun(cv::Mat& matscan);
    /// END TEMP METHODS

    // BEGIN OLD METHODS
    ParForHoughCicles dsParameterFindCircles(IplImage* image, IplImage* image1ch);
    ParForHoughCicles dsParameterFindCircles(const cv::Mat& image3ch,cv::Mat& image1ch, vector<cv::Vec3f>& circles);
    ParForHoughCicles cppParameterFindCircles(IplImage* image, IplImage* image1ch);
    ParForHoughCicles TEMPcppParameterFindCircles(IplImage* image, IplImage* image1ch);
    // END OLD METHODS
};

#endif // DSVISUALMETHOFOPENCV_H
