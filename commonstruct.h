#ifndef COMMONSTRUCT_H
#define COMMONSTRUCT_H

#include <opencv/cv.h>
#include <string.h>
#include <iostream>

const double pi = 3.141592653589793238462643;
const int NUMINCHANELL = 256;

const float FactorOfBorder = 2.;

#ifdef MINNUMPIX
const int MinNumOfPixels = MINNUMPIX;
#else
const int MinNumOfPixels = 100;
#endif

const int JPEG_QUAL = 70; //In the case of JPEG it can be a quality ( CV_IMWRITE_JPEG_QUALITY ), from 0 to 100 (the higher is the better), 95 by default.
const int PNG_QUAL = 5;// For PNG, it can be the compression level ( CV_IMWRITE_PNG_COMPRESSION ) from 0 to 9. A higher value means a smaller size and longer compression time. Default value is 3.

const int PixelMinSquareInd = 2;

typedef cv::Vec<double, 7> Vec7d;
typedef std::vector<std::vector<int> > VecInt2;
typedef std::vector<std::vector<double> > VecDbl2;
typedef std::vector<std::vector<cv::Point2i> > VecP2i;

const std::string tmpimdir = "./tempimage/";
const std::string tmpresdir = "./tmpres/";

enum { // save picture as ..
    SAVEASPNG = 0,
    SAVEASJPEG = 1
};

enum METHODOFFINDOFANGLE {
    SIMPLE = 1,
    ROOTFIT = 2,
    LSPFIT = 3
};

enum {
    MATRIXONEEL = 1,
    MATRIXTHREE = 3,
    MATRIXFIVE = 5,
    MATRIXSEVEN = 7,
    MATRIXNINE = 9
};

enum {
    R_CANAL = 1, G_CANAL = 2, B_CANAL = 3,
    H_CANAL = 4, S_CANAL = 5, V_CANAL = 6,
    GRAY = 7
    };
enum METHODERODE {
    ERODE = 1,
    DILATE = 2
};

enum DIRECTION {
    HORISONT = 1,
    VERTICAL = 2
};

enum CLASSIFICDEFECT {
    NOPIXEL = 0, NODEFECT = 1, PIXELSHIFT = 2, NOPERFCONTOUR = 3, SMALLPIXEL = 4, DISRUPT = 5, BIGPIXEL = 6
};

enum EXITSTATUS
{
    NOSHABLONFILE = 1, // template of contour is not found
    PIXELSFOUNDLESSTHAN =2, // number of found pixels less than const int MinNumOfPixels
    HAVNTDATAFORSHABLON = 3 // Image havn`t data for shablon
};

struct ParForThreshold
{
    double told;
    double value;
};

struct ParForErode
{
    int radius;
    int iterations;
    METHODERODE method;
    float resize;
};

struct InputParametersFFC
{
    int canal;          /// enum CANAL

    int Methoderode;    //      \/
    int eroderadius;    /// Parameters For Erode
    int erodeiter;      //      /\

    int ifnegative; /// Negative for image

    int Threshold;      /// Threshold
    int MaxThreshold;   // _____________

    double valforshapes; /// Value of shape (Contour)
    int shapematch;     /// Method for compares two Shapes
};


struct ParForHoughCicles
{
    double  mindist;
    double  parameter1;
    double  parameter2;
    int  radiuslow;
    int  radiushigh;
};

struct ForGaussianBlur
{
    /*
    # ksize – The Gaussian kernel size; ksize.width and ksize.height can differ, but they both
    must be positive and odd. Or, they can be zero’s, then they are computed from sigma*
    # sigmaX, sigmaY – The Gaussian kernel standard deviations in X and Y direction. If sigmaY is zero,
    it is set to be equal to sigmaX . If they are both zeros, they are computed from ksize.width
    and ksize.height , respectively, see getGaussianKernel() . To fully control the result regardless
    of possible future modification of all this semantics, it is recommended to specify all of ksize , sigmaX and sigmaY
    */
    cv::Size ksize;
    double sigma1;
    double sigma2;
    bool boolfgb;
};

inline void coutline()
{
    std::cout << "_______________________________________________________"<<std::endl;
}

/*struct ParForScanPicture // общие параметры для сканирования изображения
{
    ParForHoughCicles hough;
    ParForMask mask;
    CANAL canal;
};*/

#endif // COMMONSTRUCT_H
