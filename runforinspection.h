#ifndef RUNFORINSPECTION_H
#define RUNFORINSPECTION_H

#include "detectorscan.h"
#include "foundcircles.h"

class runforinspection : public DetectorScan, FoundCircles
{
public:
    runforinspection();
    runforinspection(cv::Mat& image, int canal,int ifnegative, int Methoderode, int eroderadius, int erodeiter, int Threshold, int MaxThreshold, int shapematch,double valforshapes);
    void run(cv::Mat& image);
};

#endif // RUNFORINSPECTION_H
