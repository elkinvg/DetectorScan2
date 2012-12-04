#ifndef DETECTORSCAN_H
#define DETECTORSCAN_H

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "commonstruct.h"

using namespace std;

class DetectorScan
{
public:
    DetectorScan();
    DetectorScan(IplImage* image);
    DetectorScan(IplImage* image, int canal, int size_roi);
    DetectorScan(cv::Mat& image);
    DetectorScan(cv::Mat& image, int canal,int ifnegative, int Methoderode, int eroderadius, int erodeiter, int Threshold, int MaxThreshold, int shapematch,double valforshapes);
    ~DetectorScan();

    void InitializationDS();

    IplImage* ErodePixel(IplImage* imageint,int radius = 1, int iterations = 1);
    IplImage* DilatePixel(IplImage* image,int radius =1, int iterations = 1);
    void dsErodePixel(cv::Mat& image,int radius = 1, int iterations =1);
    void dsDilatePixel(cv::Mat& image,int radius = 1, int iterations =1);

    IplImage* SelectChannel(IplImage* image, int canal = GRAY);
    void SelectChannel(cv::Mat &image, int ifnegative, int canal = GRAY);

    IplImage* dsPostThreshold(IplImage* image1ch,double threshold, double maxval);
    /*cv::Mat*/
    void dsPostThreshold(cv::Mat& image1ch,double val_threshold, double maxval);

    void rotateImage(cv::Mat& source, double angle, int interpolmethod =cv::INTER_AREA);
    IplImage* rotateImage(IplImage* image, double angle);


    void Negative1ch(cv::Mat& image1ch);
    void dsShablonContourFile(char *shabloncontourimagename);

    vector<cv::Vec3f> dsFindPixelsUsingContours(const cv::Mat &binaryimage,vector<vector<cv::Point2i> > &outputcontours, double valuesforshapes, int method, double minSquare);
    vector<cv::Vec3f> dsFindPixelsUsingContours(const cv::Mat &binaryimage,double valuesforshapes, int method);
    vector<vector<int> > dsFindDefects(cv::Mat& source, vector<cv::Vec2i> &gridx, vector<cv::Vec2i> &gridy, vector<cv::Vec3f> &foundpixels,cv::Vec6i indexofpict);
    void dsShiftImage(cv::Mat& source, double shiftx, double shifty, bool ifresize=false);
    void dsGetContrast(const cv::Mat &image3chorgray,int sizeofmatforcon, vector<vector<double> > &valofcon);
    void dsResizeImage(const cv::Mat& source,cv::Mat &output,double resize);


    /// BEGIN TEMP METHODS
    //void run(cv::Mat& image);
    void dsCutROIofImage(cv::Mat &image,cv::Point2i pointUpLeft, cv::Point2i pointDownRight);
    void dsCutROIofImage(cv::Mat &image,cv::Vec4i XYWidthHeight);

    void dsFindRelatedDefect(vector<vector<int> > &ifdefects, vector<cv::Vec2i> &coordefects, vector<cv::Vec4i>& DirectionOfDefect);
    double dsFindOptimalThreshold(const cv::Mat &image1ch, double valuesforshapes, int method, double beginthresh = 0, double minArea = 10.);
    void dsAnalysisDefect(cv::Mat &image1ch,vector<vector<int> > &ifdefects, vector<cv::Vec2i> &gridx, vector<cv::Vec2i> &gridy, cv::Vec3f square);
    void dsEqualizeHistMethod(const cv::Mat& in, cv::Mat& out);
    void dsEqualizeHistMethod1ch(cv::Mat& InAndOut);


    /// END TEMP METHODS




    InputParametersFFC InputParam;
    cv::Vec2i SizePict;

    ParForThreshold forthresh;
    ForGaussianBlur forgaussblur;
    ParForErode forerode;
    //ParForCanny forcanny;
    cv::Rect rect;
    int SIZE_ROI;

private:
    void dsUpdateListOfContours( vector<vector<cv::Point2i> > &inputcontours,vector<cv::Point2i> shabloncontour,double valuesforshapes, int method, bool ifusingsquare = false, double minArea = 10.);
    void dsUpdateListOfContoursUsingSquare(vector<vector<cv::Point2i> > &inputcontours,double minArea = 10.);
    void dsAnalysisIzolatedDefect(cv::Mat &image1ch,vector<vector<int> > &ifdefects, vector<cv::Vec2i> &coordefects, vector<vector<cv::Point2i> > &tempcontours, cv::Vec3f square);
    int dsRoundValue(float input);
    double dsValueOfContrast(const cv::Mat &image1ch);
    double dsSigmaForMat(const cv::Mat &image1ch);

};

#endif // DETECTORSCAN_H
