#ifndef FOUNDCIRCLES_H
#define FOUNDCIRCLES_H

#include "commonstruct.h"
using namespace std;

class FoundCircles
{
public:
    FoundCircles();
    void SortPointsOfCirclesOnX(vector<cv::Vec3f> &circles, cv::Vec2d AverageDistAngle, int* columns,  bool CoutComments = 1);
    vector<cv::Vec2d> SortForDistance(const vector<cv::Vec3f>& circles);
    double dsAngleForRotateImageDegrees(const vector<cv::Vec3f> &vec,int Method = SIMPLE,int direct = HORISONT, bool CoutComments = 1);

    cv::Vec2d dsMeanForVector2d(vector<cv::Vec2d> &DistAngle);

    void dsSearchBorders(const vector<cv::Vec3f> &circles,vector<cv::Vec6i> &Vert,vector<cv::Vec6i> &Horis,cv::Vec2d AverageDistAngle,bool CoutComments = 1);

    void dsGetParametersOfGrid(const vector<cv::Vec6i>& Horis, const vector<cv::Vec6i>& Vert, vector<cv::Vec2i>& gridx, vector<cv::Vec2i>& gridy, float AverageDist, cv::Vec2i sizeofpict);


    //TMP METHODS \/

    cv::Vec4i dsCornerDetect(const vector<cv::Vec3f> &circles, cv::Vec2i SizePict); //North East South West
    //TMP METHODS /\

private:
    void SortPointsOfTempCirclesX(vector<cv::Vec3f>& tempcircles,  bool CoutComments = 1);
    double dsAngleOfRotationRadian(const vector<cv::Vec3f> &circles,cv::Vec2d AverageDistAngle, int Method = SIMPLE, int direct = HORISONT, bool CoutComments = 1);
    double dsAngleBetweenBegEndRadian(const vector<cv::Vec3f> &tempcircles, bool CoutComments = 1);
    cv::Vec2d LSPFitLineRadian(const vector<cv::Vec3f>& circles,int direct  = HORISONT, bool CoutComments = 1);
    vector<cv::Vec6i> dsSearchBoundaryPoints(const vector<cv::Vec3f> &circles,cv::Vec2d AverageDistAngle, int DIRECT, bool CoutComments = 1);
    cv::Vec6i dsTempBoundaryPoints(const vector<cv::Vec3f>& circles,int DIRECT, bool CoutComments = 1);

    double DistBet2points(cv::Point2d pt1, cv::Point2d pt2);
    double AngleBetLinesDeg(cv::Point2d pt1, cv::Point2d pt2);
    int fcRoundValue(float input);

};

#endif // FOUNDCIRCLES_H
