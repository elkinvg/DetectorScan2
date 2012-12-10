#ifndef QDSANDFC_H
#define QDSANDFC_H

#include "detectorscan.h"
#include <QImage>

class QDSandFC: public DetectorScan
{
public:
    QDSandFC();
    explicit QDSandFC(const QImage &qimage);
    QDSandFC(const uchar*,long int);

    cv::Mat Loadimage; //

    cv::Mat QDSQImage2MatConvert(const QImage &qimage);
    QImage QDSMat2QImageConvert(const cv::Mat &matimage);

    // Get value of contrast \/
    double QGetContrast(const QImage &qimage, int grid, int koorx, int koory);
    vector<vector<double > > QGetContrast(const cv::Mat &MatImage, int grid);
    vector<vector<double > > QGetContrast(const QImage &qimage, int grid);
    // ---
    vector<vector<double > > QGetContrast(int grid);
    double QGetContrast(int grid, int koorx, int koory);
    // Get value of contrast /\

    cv::Mat QDSGetMatFromCharData(const uchar*,long int);


    //temp method \/
    void QSetDSpreamp(QImage &qimage,int radius, int iteration, int canal, int ifnegative, int ifthreshold, int threshold);
    //temp method /\

private:

};

#endif
