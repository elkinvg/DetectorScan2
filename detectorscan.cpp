#include "detectorscan.h"
//#include "foundcircles.h"
#include <string>
#include <fstream>
#include <stdlib.h>

//#include <sstream>
//#include <ios>
#include <iostream>
#include <iomanip>
#include <math.h>

//#ifdef __MINGW_GCC
//#include <dir.h>
//#endif
//#ifdef unix
//#include <sys/stat.h>
//#include <stdio.h>
//#endif

using namespace cv;

DetectorScan::DetectorScan()
{
}

DetectorScan::~DetectorScan()
{
}

DetectorScan::DetectorScan(cv::Mat& image)
{
#ifdef DEBUG
    cout << endl;
    cout << endl;
    cout << "matscan.channels() " << image.channels() << endl;
    cout << "image.depth() " << image.depth() << endl;
    cout << "image.cols " << image.cols << endl;
    cout << "image.dims " << image.dims << endl;
    cout << "image.rows " << image.rows << endl;
    cout << "image.flags " << image.flags << endl;
    cout << "image.elemSize() " << image.elemSize() << endl;
    cout << "image.elemSize1() " << image.elemSize1() << endl;
    cout << "image.step1() " << image.step1() << endl;
    cout << image.size().area() << endl;
    //cout << "image.row() " << image.row()  << endl;
    cout << endl;
    cout << endl;
#endif
}

DetectorScan::DetectorScan(Mat &image, int canal, int ifnegative, int Methoderode, int eroderadius, int erodeiter, int Threshold, int MaxThreshold, int shapematch, double valforshapes)
{
    string Ch;
    switch (canal)
    {
    case 1:
        Ch = "R_CANAL";
        break;
    case 2:
        Ch = "G_CANAL";
        break;
    case 3:
        Ch = "B_CANAL";
        break;
    case 4:
        Ch = "H_CANAL";
        break;
    case 5:
        Ch = "S_CANAL";
        break;
    case 6:
        Ch = "V_CANAL";
        break;
    case 7:
        Ch = "GRAY";
        break;
    }
    string Erod;
    switch (Methoderode)
    {
    case 1:
        Erod = "ERODE";
        break;
    case 2:
        Erod = "DILATE";
        break;
    }
    string meth;
    switch (shapematch)
    {
    case 1:
        meth = "CV_CONTOUR_MATCH_I1";
        break;
    case 2:
        meth = "CV_CONTOUR_MATCH_I2";
        break;
    case 3:
        meth = "CV_CONTOUR_MATCH_I3";
        break;
    }
#ifdef DEBUG
    cout << "DEBUG"<< endl;
    cout << endl;
    cout << " image info: " << endl;
    cout << endl;
    cout << setw(70) << setiosflags(ios::left) << " Number of channels. " << cout.fill('.')<< setiosflags(ios::right)  << image.channels() << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Image height in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image.rows << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Image width in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image.cols << resetiosflags(ios::right) << endl;
    cout << endl;
    cout << setw(70) << setiosflags(ios::left) << " Chosen channel " << cout.fill('.')<< setiosflags(ios::right)  << Ch << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Negative " << cout.fill('.')<< setiosflags(ios::right)  << ifnegative << resetiosflags(ios::right) << endl;

    cout << endl;

    cout << setw(70) << setiosflags(ios::left) << " morphological operations " << cout.fill('.')/*<< setiosflags(ios::right)  << Erod << resetiosflags(ios::right) */<< endl;
    cout << setw(70) << setiosflags(ios::left) << " Method " << cout.fill('.')<< setiosflags(ios::right)  << Erod << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Radius " << cout.fill('.')<< setiosflags(ios::right)  << eroderadius << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Iterations " << cout.fill('.')<< setiosflags(ios::right)  << erodeiter << resetiosflags(ios::right) << endl;
    cout << endl;

    cout << setw(70) << setiosflags(ios::left) << " Parameters for Threshold " << cout.fill('.')/*<< setiosflags(ios::right)  << Erod << resetiosflags(ios::right) */<< endl;
    cout << setw(70) << setiosflags(ios::left) << " Threshold " << cout.fill('.')<< setiosflags(ios::right)  << Threshold << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Maximum Threshold " << cout.fill('.')<< setiosflags(ios::right)  << MaxThreshold << resetiosflags(ios::right) << endl;
    cout << endl;

    cout << setw(70) << setiosflags(ios::left) << " Compares shapes. " << cout.fill('.') /* << setiosflags(ios::right)<< Erod << resetiosflags(ios::right) */<< endl;
    cout << setw(55) << setiosflags(ios::left) << " Method " << cout.fill('.')<< setiosflags(ios::right)  << meth << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Maximum value " << cout.fill('.')<< setiosflags(ios::right)  << valforshapes << resetiosflags(ios::right) << endl;
    cout << endl;
    cout << endl;
#endif

    InputParam.canal = canal;
    InputParam.ifnegative = ifnegative;
    InputParam.Methoderode = Methoderode;
    InputParam.erodeiter = erodeiter;
    InputParam.eroderadius = eroderadius;
    InputParam.Threshold = Threshold;
    InputParam.MaxThreshold = MaxThreshold;
    InputParam.valforshapes = valforshapes;
    InputParam.shapematch = shapematch;
}

DetectorScan::DetectorScan(IplImage* image)
{
    /*
      информация о структуре IplImage

    int nSize;  размер структуры IplImage
    int ID;  версия (=0)
    int nChannels;  Большинвство функций OpenCV поддерживают 1,2,3 или 4 канала
    int alphaChannel;  игнорируется OpenCV
    int depth;  глубина пикселя в байтах, поддерживаются: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16U,
    IPL_DEPTH_16S, IPL_DEPTH_32S, IPL_DEPTH_32F и IPL_DEPTH_64F
    char colorModel[4];  игнорируется OpenCV
    char channelSeq[4];  то же самое
    int dataOrder; 0 - чередующиеся цветовые каналы, 1 - отдельные цветовые каналы.
    cvCreateImage может создавать только чередующиеся цветовые каналы
    int origin;  0 - верхний левый угол принимается за начало координат,1 - нижний левый угол принимается
    за начало координат (стиль Windows)

    int align;  выравнивание столбцов изображения (4 or 8). OpenCV игнорирует этот параметр и использует вместо него widthStep
    int width;  ширина изображения в пикселях
    int height;  высота изображения в пикселях
    struct _IplROI *roi; область интереса - (ROI). когда не NULL, орпеделяет область для обработки
    struct _IplImage *maskROI;  должен быть NULL в OpenCV
    void *imageId;  то же самое
    struct _IplTileInfo *tileInfo; то-же самое
    int imageSize;  размер данных изображения в байтах=image->height*image->widthStep в случае чередующихся цветовых данных)
    char *imageData;  указатель на выровненные данные изображения
    int widthStep;  казмер выровненных строк изображения в байтах
    int BorderMode[4]; игнорируется OpenCV
    int BorderConst[4]; то-же самое
    char *imageDataOrigin;  указатель на начало данных изображения (не обязательно выровненные)
    нужно для правильного освобождения выделенных данных
      */

#ifdef DEBUG
    cout << endl;
    cout << " image info: " << endl;

    cout << setw(70) << setiosflags(ios::left) << " size of image in bytes "<< cout.fill('.')<< setiosflags(ios::right)  << image->imageSize << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " The size of an aligned image row, in bytes " << cout.fill('.')<< setiosflags(ios::right)  << image->widthStep << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " Image data size in bytes. For interleaved data " <<  cout.fill('.')<< setiosflags(ios::right)  << image->height*image->widthStep << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " Number of channels. " << cout.fill('.')<< setiosflags(ios::right)  << image->nChannels << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " Pixel depth in bits. " << cout.fill('.')<< setiosflags(ios::right)  << image->depth << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " Image height in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->height << resetiosflags(ios::right) << endl;

    cout << setw(70) << setiosflags(ios::left) << " Image width in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->width << resetiosflags(ios::right) << endl;

    cout << endl;
#endif

    InitializationDS();

    SIZE_ROI = 600;

   //MaskParameter.angle = 0;

    if (image->height > SIZE_ROI && image->width > SIZE_ROI)
    {
       rect.height = SIZE_ROI; rect.width = SIZE_ROI;
       rect.x = (int)round(image->width/2-SIZE_ROI/2); rect.y = (int)round(image->height/2-SIZE_ROI/2);
    }
    else
    {
        rect.x = 0; rect.y = 0; rect.width = image->width; rect.height = image->height;
    }

}

DetectorScan::DetectorScan(IplImage* image, int canal, int size_roi)
{
#ifdef DEBUG
    cout << endl;
    cout << " image info: " << endl;

    cout << setw(70) << setiosflags(ios::left) << " size of image in bytes "<< cout.fill('.')<< setiosflags(ios::right)  << image->imageSize << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " The size of an aligned image row, in bytes " << cout.fill('.')<< setiosflags(ios::right)  << image->widthStep << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Image data size in bytes. For interleaved data " <<  cout.fill('.')<< setiosflags(ios::right)  << image->height*image->widthStep << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Number of channels. " << cout.fill('.')<< setiosflags(ios::right)  << image->nChannels << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Pixel depth in bits. " << cout.fill('.')<< setiosflags(ios::right)  << image->depth << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Image height in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->height << resetiosflags(ios::right) << endl;
    cout << setw(70) << setiosflags(ios::left) << " Image width in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->width << resetiosflags(ios::right) << endl;
    cout << endl;
#endif

    InitializationDS();

    SIZE_ROI = size_roi;

    //MaskParameter.angle = 0;

    if (image->height > SIZE_ROI && image->width > SIZE_ROI)
    {
       rect.height = SIZE_ROI; rect.width = SIZE_ROI;
       rect.x = (int)round(image->width/2-SIZE_ROI/2); rect.y = (int)round(image->height/2-SIZE_ROI/2);
    }
    else
    {
        rect.x = 0; rect.y = 0; rect.width = image->width; rect.height = image->height;
    }
}

void DetectorScan::InitializationDS()
{
    forgaussblur.ksize = Size(9,9);
    forgaussblur.sigma1 = 2;
    forgaussblur.sigma2 = 2;
    forgaussblur.boolfgb = 1;

    forerode.resize = 1;
    //MiniMax = Vec2d(0,0);
}

IplImage* DetectorScan::ErodePixel(IplImage* image,int radius, int iterations)
{
    /*
      Применение метода эрозии.
    размывает(операция сужения) изображение с использованием фильтра(ядра) IplConvKernel* Kern
    значения (radius = 4; iteration = 1;) выставлены по умолчанию.
    "...Эрозия (размывание/сужение) изображения обычно используется для избавления от случайных
    вкраплений на изображении. Идея состоит в том, что вкрапления при размывании устранятся,
    тогда как крупные и соответсвенно более визуально-значимые регионы остаются..."
    Возвращает новую picture.
    */
    IplImage* newimage = 0;

    //int radius, iterations;
    //radius = 4; iterations = 1;

    newimage = cvCloneImage(image);

    // создаём ядро
    IplConvKernel* Kern = cvCreateStructuringElementEx(radius*2+1, radius*2+1, radius, radius, CV_SHAPE_ELLIPSE);

    // выполняем преобразования
    cvErode(image, newimage, Kern, iterations);
    // удаляем ресурсы
    cvReleaseStructuringElement(&Kern);

    //cvReleaseImage(&image); <-- NONE
    return newimage;
}

void DetectorScan::dsErodePixel(Mat &image,int radius, int iterations)
{
    //Mat outimage;
    Point anchor = Point(-1,-1);

    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(radius+1,radius+1),Point(radius,radius));

    erode(image,/*out*/image,element,anchor,iterations);
    //return outimage;
}

IplImage* DetectorScan::DilatePixel(IplImage* image,int radius, int iterations)
{
    /*
— растягивает(операция расширения) изображение с использованием фильтра(ядра) один или несколько раз,
если element == NULL используется ядро 3х3
(изображение формируется из локальных максимумов — т.е. будут увеличиваться светлые области)
cvDilate( const CvArr* src, CvArr* dst,
                       IplConvKernel* element CV_DEFAULT(NULL),
                       int iterations CV_DEFAULT(1) );
src — исходное изображение
dst — получаемое изображение
element — структурирующий элемент (ядро) по-умолчанию NULL — соответствует ядру 3x3 с якорем по-центру.

[ ][ ][ ]
[ ][+][ ]
[ ][ ][ ]

    Dilates an image by using a specific structuring element.
    Parameters:

        * src – Source image
        * dst – Destination image
        * element – Structuring element used for dilation. If it is NULL, a $3\times 3$ rectangular structuring element is used
        * iterations – Number of times dilation is applied

    The function dilates the source image using the specified structuring element that determines the shape of a pixel neighborhood over which the maximum is taken:

    \max _{(x',y') \, in \, \texttt{element}}src(x+x',y+y')

    The function supports the in-place mode. Dilation can be applied several (iterations) times. For color images, each channel is processed independently.
*/
    IplImage* newimage = 0;

    //int radius, iterations;
    //radius = 4; iterations = 1;

    newimage = cvCloneImage(image);

    // создаём ядро
    IplConvKernel* Kern = cvCreateStructuringElementEx(radius*2+1, radius*2+1, radius, radius, CV_SHAPE_ELLIPSE);

    // выполняем преобразования
    cvDilate(image, newimage, Kern, iterations);
    // удаляем ресурсы
    cvReleaseStructuringElement(&Kern);

    //cvReleaseImage(&image); <-- NONE
    return newimage;
}

void DetectorScan::dsDilatePixel(Mat &image,int radius, int iterations)
{
    //Mat outimage;
    Point anchor = Point(-1,-1);

    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(2*radius+1,2*radius+1),Point(radius,radius));

    dilate(image,/*out*/image,element,anchor,iterations);
    //return outimage;
}

IplImage* DetectorScan::SelectChannel(IplImage* image, int canal)
{
    /*
      Выбор канала, для преобразования.
      Входное изображение должно быть 3х канальным.
      */


    if (image->nChannels == 1)
    {
        return image;
    }

    IplImage*  hsv = 0;
    IplImage* tempin = cvCreateImage( cvGetSize(image), 8, 1 );

    if(canal == H_CANAL || canal == S_CANAL || canal == V_CANAL)
    {
        hsv = cvCreateImage( cvGetSize(image), 8, 3 );
        cvCvtColor( image, hsv, CV_BGR2HSV );
    }


    switch (canal)
    {
    case B_CANAL:
        cvSplit( image, tempin, 0, 0, 0 );
        break;
    case G_CANAL:
        cvSplit( image, 0, tempin, 0, 0 );
        break;
    case R_CANAL:
        cvSplit( image, 0, 0, tempin, 0 );
        break;
    case H_CANAL:
        cvCvtPixToPlane( hsv, tempin, 0, 0, 0 );
        break;
    case S_CANAL:
        cvCvtPixToPlane( hsv, 0, tempin, 0, 0 );
        break;
    case V_CANAL:
        cvCvtPixToPlane( hsv, 0, 0, tempin, 0 );
        break;
    case GRAY:
        cvCvtColor( image, tempin, CV_BGR2GRAY);
        break;
   default:
        cvCvtColor( image, tempin, CV_BGR2GRAY);
    }
    cvReleaseImage(&hsv);
    return tempin;
}

void DetectorScan::SelectChannel(Mat &image, int ifnegative, int canal)
{
    if (image.channels() == 1)
    {
        return /*image*/;
    }

    vector<Mat> tempin;

    if(canal == H_CANAL || canal == S_CANAL || canal == V_CANAL)
    {
        cvtColor( image, image, CV_BGR2HSV );
    }

    switch (canal)
    {
    case B_CANAL:
        split(image,tempin);
        image = tempin[0];
        break;
    case G_CANAL:
        split(image,tempin);
        image = tempin[1];
        break;
    case R_CANAL:
        split(image,tempin);
        image = tempin[2];
        break;
    case H_CANAL:
        split(image,tempin);
        image = tempin[0];
        break;
    case S_CANAL:
        split(image,tempin);
        image = tempin[1];
        break;
    case V_CANAL:
        split(image,tempin);
        image = tempin[2];
        break;
    case GRAY:
        cvtColor( image, image, CV_BGR2GRAY);
        break;
   default:
        cvtColor( image, image, CV_BGR2GRAY);
    }

    if (ifnegative == 1)
    {
        Negative1ch(image);
    }
}

IplImage* DetectorScan::dsPostThreshold(IplImage* image1ch,double threshold, double maxval)
{
    /*
double cvThreshold(const CvArr* src, CvArr* dst, double threshold, double maxValue, int thresholdType)

    Applies a fixed-level threshold to array elements.
    Parameters:

        * src Source array (single-channel, 8-bit or 32-bit floating point)
        * dst Destination array; must be either the same type as src or 8-bit
        * threshold  Threshold value
        * maxValue Maximum value to use with CV_THRESH_BINARY and CV_THRESH_BINARY_INV thresholding types
        * thresholdType Thresholding type (see the discussion)

    The function applies fixed-level thresholding to a single-channel array. The function is typically used to get a bi-level (binary)
image out of a grayscale image (CmpS could be also used for this purpose) or for removing a noise, i.e. filtering out pixels with too small
or too large values. There are several types of thresholding that the function supports that are determined by thresholdType:

            *

              CV_THRESH_BINARY -

              \texttt{dst}(x,y) = \fork {\texttt{maxValue}}{if $\texttt{src}(x,y) > \texttt{threshold}$}{0}{otherwise}
            *

              CV_THRESH_BINARY_INV -

              \texttt{dst}(x,y) = \fork {0}{if $\texttt{src}(x,y) > \texttt{threshold}$}{\texttt{maxValue}}{otherwise}
            *

              CV_THRESH_TRUNC -

              \texttt{dst}(x,y) = \fork {\texttt{threshold}}{if $\texttt{src}(x,y) > \texttt{threshold}$}{\texttt{src}(x,y)}{otherwise}
            *

              CV_THRESH_TOZERO -

              \texttt{dst}(x,y) = \fork {\texttt{src}(x,y)}{if $\texttt{src}(x,y) > \texttt{threshold}$}{0}{otherwise}
            *

              CV_THRESH_TOZERO_INV -

              \texttt{dst}(x,y) = \fork {0}{if $\texttt{src}(x,y) > \texttt{threshold}$}{\texttt{src}(x,y)}{otherwise}

*/
    IplImage* thresh = 0;
    thresh = cvCreateImage( cvGetSize(image1ch), IPL_DEPTH_8U, 1 );
    cvThreshold(image1ch,thresh,threshold,maxval,CV_THRESH_BINARY);
    return thresh;
}

void DetectorScan::dsPostThreshold(Mat& image1ch,double val_threshold, double maxval)
{
    threshold(image1ch,image1ch,val_threshold,maxval,CV_THRESH_BINARY);
}

/*Mat*/ void DetectorScan::rotateImage(cv::Mat& source, double angle,int interpolmethod)
{
    /*
      Поворот изображения на angle градусов
    interpolation method:
    resize() method
    INTER_NEAREST - a nearest-neighbor interpolation
    INTER_LINEAR - a bilinear interpolation (used by default)
    INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
    INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood
    INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
      */
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = cv::getRotationMatrix2D(src_center,angle, 1.0);

    Mat warp_dst;// = Mat::zeros( source.rows, source.cols, source.type() );
    Size dsize = source.size();// (source.rows,source.cols);

    warpAffine(source, warp_dst, rot_mat, dsize,interpolmethod/*cv::INTER_CUBIC,BORDER_TRANSPARENT*/);
#ifdef DEBUG
    cout << endl;
    cout << " Rotate of the image " << angle << " degrees " << endl;
    cout << endl;
#endif
    source = warp_dst.clone();
}

IplImage* DetectorScan::rotateImage(IplImage* image, double angle)
{
    /*
      Поворот изображения на angle градусов
      */
    //Point2f src_center(image->width/2.0F, image->height/2.0F);
    CvPoint2D32f src_center = cvPoint2D32f(image->width/2.0F, image->height/2.0F);
    IplImage* dst =0;
    CvMat* rot_mat = cvCreateMat(2,3,CV_32FC1);
    cv2DRotationMatrix(src_center,angle,1.0,rot_mat);
    if (image->nChannels == 3) dst = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 3 );
    else dst = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
#ifdef DEBUG
    cout << " source.size() " /*<< source.size()*/ << endl;
#endif

    cvWarpAffine/*cvWarpPerspective*/(image,dst,rot_mat,cv::INTER_CUBIC);

#ifdef DEBUG
    cout << endl;
    cout << " Rotate of the image " << angle << " degrees " << endl;
    cout << endl;
#endif

    return dst;
}

void DetectorScan::Negative1ch(Mat &image1ch)
{
    if (image1ch.channels()!=1)
    {
#ifdef DEBUG
        cout << " void DetectorScan::Negative1ch(Mat& image1ch) " << endl;
        cout << " input image must be 1 channel " << endl;
#endif
        return;
    }
    uchar value,newvalue;
    for (int i = 0; i<image1ch.cols; i++)
    {
        for (int j=0; j<image1ch.rows;j++)
        {
            value = image1ch.at<uchar>(j,i);
            newvalue = 255 - value;
            image1ch.at<uchar>(j,i) = newvalue;
        }
    }
}

void DetectorScan::dsShiftImage(Mat &source, double shiftx,double shifty,bool ifresize)
{
    Point2f srcTri[3];
    Point2f dstTri[3];
    srcTri[0] = Point2f( 0,0 );
    srcTri[1] = Point2f( source.cols - 1, 0 );
    srcTri[2] = Point2f( 0, source.rows - 1 );

    //       dstTri[0] = Point2f(source.cols*0.0, source.rows*0.33 );
    //       dstTri[1] = Point2f( source.cols*0.85, source.rows*0.25 );
    //       dstTri[2] = Point2f( source.cols*0.15, source.rows*0.7 );

    dstTri[0] = Point2f(srcTri[0].x+shiftx, srcTri[0].y +shifty);
    dstTri[1] = Point2f( srcTri[1].x+shiftx, srcTri[1].y +shifty);
    dstTri[2] = Point2f( srcTri[2].x+shiftx, srcTri[2].y +shifty );
//    int newsizex = (int)floor(source.cols*cos(2*pi*fabs(angle)/360)+source.rows*sin(2*pi*fabs(angle)/360));
//    int newsizey = (int)floor(source.cols*sin(2*pi*fabs(angle)/360)+source.rows*cos(2*pi*fabs(angle)/360));
    cv::Size dsize;
    int newsizex,newsizey;
    if(ifresize)
    {
        newsizex = source.cols + shiftx;
        newsizey = source.rows + shifty;
    }
    else
    {
        newsizex = source.cols;
        newsizey = source.rows;
    }
#ifdef DEBUG
    //cout << "______________________________________________________________"<<endl;
    coutline();
    cout << "shift image: X= " << shiftx << " ; Y= "<< shifty <<endl;
    cout << " oldsizex " << source.cols << " oldsizey " << source.rows << endl;
    cout << " newsizex  " << newsizex << " newsizey " << newsizey << endl;
    coutline();
    //cout << "______________________________________________________________"<<endl;
#endif
    dsize = Size(newsizex,newsizey);
    Mat warp_dst;
        Mat tr_aff = getAffineTransform(srcTri, dstTri);
    warpAffine(source, warp_dst, tr_aff/*,source.size()*/, dsize,cv::INTER_CUBIC/*,BORDER_TRANSPARENT*/);
    source = warp_dst.clone();

}

void DetectorScan::dsFindRelatedDefect(vector<vector<int> > &ifdefects, vector<Vec2i> &coordefects, vector<Vec4i>& DirectionOfDefect)
{
#ifdef DEBUG
    cout << " coordefects.size() " <<  coordefects.size() << " DirectionOfDefect.size() " << DirectionOfDefect.size() << endl;
#endif
    int Niter = coordefects.size();
    int Pos;
    int PosN,PosE,PosS,PosW;
    //PosN = PosE = PosS = PosW = 0;
    for (int i=0;i<Niter;i++)
    {
        PosN = PosE = PosS = PosW = 0;
        if (DirectionOfDefect[i][0]>2) PosN = 1;
        if (DirectionOfDefect[i][1]>2) PosE = 1;
        if (DirectionOfDefect[i][2]>2) PosS = 1;
        if (DirectionOfDefect[i][3]>2) PosW = 1;

        Pos = PosN*1 + PosE*2 + PosS*4 + PosW*8;
        ifdefects[coordefects[i][0]][coordefects[i][1]] = 10 + Pos;
    }
    coordefects.clear();
    DirectionOfDefect.clear();
}

double DetectorScan::dsFindOptimalThreshold(const Mat& image1ch, double valuesforshapes, int method, double beginthresh, double minArea)
{
    vector<vector<cv::Point2i> > shabloncontour;

    if (cvLoadImage("./shabloncontour2.png",0) != NULL)
    {
        IplImage* tmpim = cvLoadImage("./shabloncontour2.png",0);
        Mat shabloncontourimage = tmpim;
        findContours(shabloncontourimage,shabloncontour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    }
    else
    {
        if (ifstream("contourround.dsshcon") != NULL)
        {
                ifstream ishablons;

                ishablons.open("contourround.dsshcon",ios_base::binary);
                vector<Point2i> tmp;
                int tmp1,tmp2;
                //for (int i=0;i<shabloncontour.size();i++)
                while(ishablons.peek()!=-1)
                {
                    ishablons.read((char*)&tmp1,sizeof(int));
                    ishablons.read((char*)&tmp2,sizeof(int));

                    //shabloncontour[0].push_back(Point2i(tmp1,tmp2));
                    tmp.push_back(Point2i(tmp1,tmp2));
                    //cout << tmp1 << " " << tmp1 << endl;
                }
                shabloncontour.push_back(tmp);
                ishablons.close();
        }
    }

        vector<vector<cv::Point2i> > contours;
        vector<int> NumCirclesToThresholds;
        int tmpsize;

        Mat binaryimage;

    for (int i=dsRoundValue(beginthresh); i<NUMINCHANELL;i++)
    {
        binaryimage = image1ch.clone();
        dsPostThreshold(binaryimage,(double)(i),NUMINCHANELL-1);
        findContours(binaryimage,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
#ifdef DEBUG
        cout << " NNNNN " << contours.size() << "___"<<i/*<< " ____ " << contours[0] */;//<<endl;
#endif
        if (contours.size()<1)
        {
            contours.clear();
            NumCirclesToThresholds.push_back(0);
        }
        else
        {
            dsUpdateListOfContours(contours,shabloncontour[0],valuesforshapes,method,minArea);

            tmpsize = contours.size();
            NumCirclesToThresholds.push_back(tmpsize);
            //contours.clear();
        }
        //dsFindPixelsUsingContours(binaryimage,valuesforshapes,method);
#ifdef DEBUG
        cout << " " << tmpsize << endl;
#endif
    }
#ifdef DEBUG
    cout << " Size NumCirclesToThresholds " << NumCirclesToThresholds.size() << endl;
#endif
    return 0;
}

void DetectorScan::dsAnalysisDefect(Mat &image1ch,vector<vector<int> > &ifdefects, vector<Vec2i> &gridx, vector<Vec2i> &gridy, Vec3f square)
{
//    Один из способов доступа к пикселам для изображений, у которых известен тип -использование метода at
//    Для одноканального изображения 0...255 это делаетсятак://Взятие значения
//    int value = imageGray.at<uchar>(y, x);
//    //Установка значения
//    imageGray.at<uchar>(y, x) = 100;
//    Обратите внимание, что x и y в вызове переставлены местами.

    Mat tempimage,tempimage2;
    vector<Vec2i> coordefects;
    vector<Vec2i> tempdefects;
    const int Nx = ifdefects.size();
    const int Ny = ifdefects[0].size();

    int numofcont;
    Vec2i tmpvec2i;
    Mat copyimage;
    Rect rectimage;

    for (int i=0;i<Nx;i++)
    {
        for (int j=0;j<Ny;j++)
        {
            if (ifdefects[i][j]==0) coordefects.push_back(Vec2i(i,j));
        }
    }
#ifdef DEBUG
    cout << " Tentative Number of defects " << coordefects.size() << endl;
#endif
//    while (coordefects.size()!=0)
//    {
    int Nfor = coordefects.size();
    int Iter = 0;
    int Iter2 = 0;
    vector<vector<cv::Point2i> > tempcontours;
    vector<vector<cv::Point2i> > izolatedcontour;
    vector<Vec4i> NESWdirection;


    for (int i=0;i<Nfor;i++)
    {
        tempcontours.clear();
        tmpvec2i = coordefects[Iter];
        rectimage = Rect(gridx[tmpvec2i[0]][0],gridy[tmpvec2i[1]][0],gridx[tmpvec2i[0]][1],gridy[tmpvec2i[1]][1]);
        tempimage = image1ch(rectimage).clone();
        //tempimage2 = image1ch(rectimage);
        cv::findContours(tempimage,tempcontours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        tempimage = image1ch(rectimage).clone();
        if (tempcontours.size()>=1)
        {
            Iter2=0;
            numofcont = tempcontours.size();
            for (int j=0;j<numofcont;j++)
            {
                if (fabs(contourArea(tempcontours[Iter2])) < (square[0]/PixelMinSquareInd))
                {
//                    cout << " DRAW  " << fabs(contourArea(tempcontours[Iter2])) << "  " <<  square[0]/PixelMinSquareInd << endl;
//                    if (fabs(contourArea(tempcontours[Iter2]))==0) cout << " NULL " << tempcontours[Iter2].size() << endl;
                    drawContours(tempimage,tempcontours,Iter2,Scalar(/*155*/0),CV_FILLED);
                    tempcontours.erase(tempcontours.begin()+Iter2);
                    Iter2 = Iter2 - 1;
                }
                Iter2++;
            }
        }
        if (tempcontours.size()==0)
        {
            ifdefects[tmpvec2i[0]][tmpvec2i[1]] = NOPIXEL;
            coordefects.erase(coordefects.begin()+Iter);
//            Iter--;
//            Iter++;
            continue;
        }
        tempimage2 = tempimage.clone();
        int tmpstat = 0;
        int Nindicator,Sindicator,Windicator,Eindicator;
        int Nstat,Sstat,Wstat,Estat;
        Nindicator = Sindicator = Windicator = Eindicator = 0;
        Nstat = Sstat = Wstat = Estat = 0;


        //int

        for (int j=1;j<tempimage2.cols-1;j++)
        {
            if (tempimage2.at<uchar>(1,j)>10)
            {
                tmpstat++; Nstat++;
            }
            if (tempimage2.at<uchar>((tempimage2.rows-2),j)>10)
            {
                tmpstat++; Sstat++;
            }
        }
//        for (int j=1;j<tempimage2.cols-1;j++)
//        {
//            if (tempimage2.at<uchar>((tempimage2.rows-2),j)>10) tmpstat++;
//        }
        for (int j=1;j<tempimage2.rows-1;j++)
        {
            if (tempimage2.at<uchar>(j,1)>10)
            {
                tmpstat++; Estat++;
            }
            if (tempimage2.at<uchar>(j,tempimage2.cols-2)>10)
            {
                tmpstat++; Wstat++;
            }
        }
//        for (int j=1;j<tempimage2.rows-1;j++)
//        {
//            if (tempimage2.at<uchar>(j,tempimage2.cols-2)>10) tmpstat++;
//        }

        if (tmpstat==0)
        {
            //cout << " tempcontours.size() " << tempcontours.size() << endl;


            if (tempcontours.size()>1)
            {
                ifdefects[coordefects[Iter][0]][coordefects[Iter][1]] = DISRUPT;
                coordefects.erase(coordefects.begin()+Iter);
                Iter--;
            }
            else
            {
                tempdefects.push_back(coordefects[Iter]);
                coordefects.erase(coordefects.begin()+Iter);
                izolatedcontour.push_back(tempcontours[0]);
                Iter--;
            }

            //Iter--;
        }
        else
        {
            NESWdirection.push_back(Vec4i(Nstat,Estat,Sstat,Wstat));
        }
        Iter++;
    }
#ifdef DEBUG
    cout << " Tentative Number of defects " << coordefects.size() << endl;
#endif
    dsAnalysisIzolatedDefect(image1ch,ifdefects,tempdefects,izolatedcontour,square);
    if (coordefects.size()==0) return;

    //else cout << " EST ESHO DEFECTI" << endl;
#ifdef DEBUG
    cout << " coordefects.size() " << coordefects.size() << " NESWdirection.size() " << NESWdirection.size() << endl;
#endif
    dsFindRelatedDefect(ifdefects,coordefects,NESWdirection);
#ifdef DEBUG
    cout << " coordefects.size() POST " << coordefects.size() << " NESWdirection.size() POST " << NESWdirection.size() << endl;
#endif
}


 void DetectorScan::dsGetContrast(const Mat& image3chorgray, int sizeofmatforcon, vector<vector<double> > &valofcon)
{
//    better values for int sizeofmatforcon
//    MATRIXONEEL = 1,
//    MATRIXTHREE = 3,
//    MATRIXFIVE = 5,
//    MATRIXSEVEN = 7,
//    MATRIXNINE = 9

    int Ncols,Nrows;
    Mat image3chorgraycopy;
    if (image3chorgray.channels()==3)
    {
        image3chorgraycopy = image3chorgray.clone();
        /*image3chorgray = */SelectChannel(image3chorgraycopy,0,GRAY);
    }
    else
    {
        image3chorgraycopy = image3chorgray;
    }
    Ncols = image3chorgraycopy.cols;
    Nrows = image3chorgraycopy.rows;
    int colstep = (int)Ncols/sizeofmatforcon;
    int rowstep = (int)Nrows/sizeofmatforcon;
    int tempcol = 0;
    int temprow;// = 0;
    vector<double> tempvec;
    Rect rectroi;
    for (int i=0;i<sizeofmatforcon;i++)
    {
        temprow = 0;
        rectroi.x = tempcol;
        if (i==sizeofmatforcon-1)
        {
            rectroi.width = Ncols - tempcol;
        }
        else
        {
            rectroi.width = colstep;
        }
        tempvec.clear();
        for (int j=0;j<sizeofmatforcon;j++)
        {
            rectroi.y = temprow;
            if (j==sizeofmatforcon-1)
            {
                rectroi.height = Nrows - temprow;
            }
            else
            {
                rectroi.height = rowstep;
                temprow += rowstep;
            }
            tempvec.push_back(dsValueOfContrast(image3chorgraycopy(rectroi)));
        }
        tempcol += colstep;
        valofcon.push_back(tempvec);
        //dsValueOfContrast(image3chorgraycopy());
    }
    //dsValueOfContrast(image3chorgraycopy);
 }

 void DetectorScan::dsResizeImage(const Mat &source, Mat& output, double resize)
 {
     cv::Size dsize(source.cols*resize,source.rows*resize);
     cv::resize(source,output,dsize);
 }

 void DetectorScan::dsCutROIofImage(Mat &image, Point2i pointUpLeft, Point2i pointDownRight)
 {
     if ((pointUpLeft.x>pointDownRight.x)||(pointUpLeft.y>pointDownRight.y)) return;
     if ( (pointUpLeft.x>(image.cols-1)) || (pointUpLeft.y>(image.rows-1)) ) return;

     if ( (pointDownRight.x>image.cols) || (pointDownRight.y>image.rows) )
     {
         if (pointDownRight.x>image.cols) pointDownRight.x = image.cols;
         if (pointDownRight.y>image.rows) pointDownRight.y = image.rows;
     }
     Rect ROI(pointUpLeft.x,pointUpLeft.y,(pointDownRight.x-pointUpLeft.x),(pointDownRight.y-pointUpLeft.y));
     image = image(ROI);
 }

 void DetectorScan::dsCutROIofImage(Mat &image, Vec4i XYWidthHeight)
 {
     // XYWidthHeight [0] X pointUpLeft [1] Y pointUpLeft [2] Width of ROI [3] Height of ROI
     if ((XYWidthHeight[0]>image.cols) || (XYWidthHeight[1]>image.rows)) return;

     if ( ((XYWidthHeight[0]+XYWidthHeight[2])>image.cols) || ((XYWidthHeight[1]+XYWidthHeight[3])>image.rows) )
     {
         if ((XYWidthHeight[0]+XYWidthHeight[2])>image.cols) XYWidthHeight[2]=image.cols-XYWidthHeight[0];
         if ((XYWidthHeight[1]+XYWidthHeight[3])>image.rows) XYWidthHeight[3]=image.rows-XYWidthHeight[1];
     }

     image = image(Rect(XYWidthHeight[0],XYWidthHeight[1],XYWidthHeight[2],XYWidthHeight[3]));
 }

void DetectorScan::dsAnalysisIzolatedDefect(Mat &image1ch, vector<vector<int> > &ifdefects, vector<Vec2i> &coordefects, vector<vector<cv::Point2i> > &tempcontours ,Vec3f square)
{
    // vector<Vec2i> coordefects;
    // два значения для номера пикселя по х и по у
    // Vec3f (square,squaremin,squaremax);

    while (coordefects.size()>0)
    {
        if (fabs(contourArea(tempcontours[0])) < square[1])
        {
            ifdefects[coordefects[0][0]][coordefects[0][1]] = SMALLPIXEL;
            coordefects.erase(coordefects.begin());
            tempcontours.erase(tempcontours.begin());
            continue;
        }
        if (fabs(contourArea(tempcontours[0])) > square[2])
        {
            ifdefects[coordefects[0][0]][coordefects[0][1]] = BIGPIXEL;
            coordefects.erase(coordefects.begin());
            tempcontours.erase(tempcontours.begin());
            continue;
        }
        ifdefects[coordefects[0][0]][coordefects[0][1]] = NOPERFCONTOUR;
        coordefects.erase(coordefects.begin());
        tempcontours.erase(tempcontours.begin());
        continue;
    }
}

vector<vector<int> > DetectorScan::dsFindDefects(Mat &source, vector<Vec2i> &gridx, vector<Vec2i> &gridy, vector<Vec3f> &foundpixels, Vec6i indexofpict)
{
    int Nx, Ny;
    Nx = gridx.size();
    Ny = gridy.size();
    vector<vector<int> > ifdefects;

    string NameFile,NameFilePng,NameFileDat,NameFileGrids,IndexBorder;
    string tmpxind,tmpyind;
    char tmpch[3];
#ifdef __MINGW_GCC
    itoa(indexofpict[0],tmpch,10);
#endif
#ifdef unix
    //char str[16]; int i = 10; sprintf(str,"%i",i);
    sprintf(tmpch,"%i",indexofpict[0]);
#endif

    if ( indexofpict[0] > 9 && indexofpict[0] > 0)
    {
        if (indexofpict[0] > 99 && indexofpict[0] < 999) tmpxind = string(tmpch);
        else tmpxind = "0" + string(tmpch);
    }
    else tmpxind = "00" + string(tmpch);

#ifdef __MINGW_GCC
    itoa(indexofpict[1],tmpch,10);
#endif
#ifdef unix
    sprintf(tmpch,"%i",indexofpict[1]);
#endif
    if ( indexofpict[1] > 9 && indexofpict[1] > 0)
    {
        if (indexofpict[1] > 99 && indexofpict[1] < 999) tmpyind = string(tmpch);
        else tmpyind = "0" + string(tmpch);
    }
    else tmpyind = "00" + string(tmpch);


    if (indexofpict[2] == 0) IndexBorder = "0";
    else  IndexBorder = IndexBorder + "1";
    if (indexofpict[3] == 0) IndexBorder = IndexBorder + "0";
    else  IndexBorder = IndexBorder + "1";
    if (indexofpict[4] == 0) IndexBorder = IndexBorder + "0";
    else  IndexBorder = IndexBorder + "1";
    if (indexofpict[5] == 0) IndexBorder = IndexBorder + "0";
    else  IndexBorder = IndexBorder + "1";
    IndexBorder = "B" + IndexBorder;

    NameFile = "X" + tmpxind + "Y" + tmpyind + IndexBorder;
    NameFilePng = tmpresdir + NameFile + ".jpeg";
    NameFileDat =  tmpresdir + NameFile + ".dat";
    NameFileGrids = tmpresdir + NameFile + "-grids.dat";


    ofstream gridout(NameFileGrids.c_str());
    for (int i=0; i < Nx;i++)
    {
        vector<int> tmpvec;
        for (int j=0; j<Ny; j++)
        {
            tmpvec.push_back(0);
            if (i==0)
            {
                if (j>0) gridout << "  " << "[" << gridy[j][0] << ";" << gridy[j][1] << "]";
                else {gridout << "Y" << endl;gridout << "[" << gridy[j][0] << ";" << gridy[j][1] << "]";}
            }
        }
        if (i==0)
        {
            gridout << endl;
            gridout << "X" << endl;
            gridout << "[" << gridx[i][0] << ";" << gridx[i][1] << "]";
        }
        else gridout << "  " << "[" << gridx[i][0] << ";" << gridx[i][1] << "]";
        ifdefects.push_back(tmpvec);
        tmpvec.clear();
    }
    gridout.close();


    int Xdef,Ydef;
    float square = 0;
    float squaremin,squaremax;

    for (int i=0;i<foundpixels.size();i++)
    {
        for (int j=0;j<gridx.size();j++)
        {
            if ((foundpixels[i][0]>gridx[j][0]) && (foundpixels[i][0]<(gridx[j][0]+gridx[j][1]))) {Xdef = j; break;}
        }
        for (int j=0;j<gridy.size();j++)
        {
            if ((foundpixels[i][1]>gridy[j][0]) && (foundpixels[i][1]<(gridy[j][0]+gridy[j][1]))) {Ydef = j; break;}
        }
        ifdefects[Xdef][Ydef] = 1;

        if (i==0)
        {
            squaremin = squaremax = foundpixels[i][2];
        }
        else
        {
            if (foundpixels[i][2]>squaremax) squaremax = foundpixels[i][2];
            if (foundpixels[i][2]<squaremin) squaremin = foundpixels[i][2];
            square =  square + foundpixels[i][2];
        }
    }

    //////// TEMP BEGIN///////////////////////////////////////////////////////
    square = square/foundpixels.size();


    vector<Vec2i> coordefects;
    Vec3f Vecsquare(square,squaremin,squaremax);
//    int Nxc = ifdefects.size();
//    int Nyc = ifdefects[0].size();
//    for (int i=0;i<Nxc;i++)
//    {
//        for (int j=0;j<Nyc;j++)
//        {
//            if (ifdefects[i][j]==0) coordefects.push_back(Vec2i(i,j));
//        }
//    }
//    cout << " SIZEEE " << coordefects.size() << endl;

//    dsAnalysisIzolatedDefect(source,ifdefects,gridx,gridy,coordefects,Vecsquare);
//    coordefects.clear();
    dsAnalysisDefect(source,ifdefects,gridx,gridy,Vecsquare);


    //////// TEMP END///////////////////////////////////////////////////////

    vector<int> param;
    param.push_back(CV_IMWRITE_JPEG_QUALITY);
    param.push_back(JPEG_QUAL);

    imwrite(NameFilePng,source,param);

    ofstream defout(NameFileDat.c_str());
    for (int i=0; i < Ny;i++)
    {
        if (i>0) defout << endl;
        for (int j=0; j<Nx; j++)
        {
            if (j>0) defout << " " << ifdefects[j][i];
            else defout << ifdefects[j][i];
        }
    }
    defout.close();
    //dsFindRelatedDefect(ifdefects);
    return ifdefects;
}

vector<Vec3f> DetectorScan::dsFindPixelsUsingContours(const Mat& binaryimage/*, Vec7d shabloncontours*/, vector<vector<Point2i> >& outputcontours, double valuesforshapes, int method, double minSquare)
{

    // shabloncontours - shablon for search contours
    // valuesforshapes
    //
    // http://opencv.willowgarage.com/documentation/cpp/imgproc_structural_analysis_and_shape_descriptors.html?highlight=matchshapes#matchShapes
    // method CV_CONTOUR_MATCH_I1 CV_CONTOUR_MATCH_I2 CV_CONTOUR_MATCH_I3
    // CV_CONTOUR_MATCH_I1 >> summa(1,7) |1/mu1 - 1/mu2|
    // CV_CONTOUR_MATCH_I2 >> summa(1,7) |mu1 - mu2|
    // CV_CONTOUR_MATCH_I3 >> summa(1,7) |mu1 - mu2|/|mu1|
    // where mu1 and mu2 is sign(humoments(i)*log(humoments(i))
    //
    /**

    Трассировка контуров
    void
    findContours(const Mat& image, //Входное изображение,
    // 1-канальное, 8-битное
    // трактуется как бинарное (0 и не 0)
    vector<vector<Point> >&contours, //Найденные контуры
    int mode, // Режим поиска контуров
    int method, //Способ аппроксимации контуров
    Point offset=Point() //Сдвиг всех результрующих контуров)

    Значения mode:
    CV_RETR_EXTERNAL - только внешние контуры,
    CV_RETR_LIST - список всех контуров
    Используется другая форма функции, см. документацию:
    CV_RETR_CCOMP - 2-уровневая иерархия - внешние границы и границы дырок,
    CV_RETR_TREE - стоит дерево вложенных контуров,
    Значения method:
    CV_CHAIN_APPROX_NONE - без аппроксимации
    CV_CHAIN_APPROX_SIMPLE - выбрасявает горизонтальные и вертикальные точки внутри отрезков
    CV_CHAIN_APPROX_TC89_L1, CV_CHAIN_APPROX_TC89_KCOS - аппроксимация методом Teh-Chin

    C++: void findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset=Point())
    C++: void findContours(InputOutputArray image, OutputArrayOfArrays contours, int mode, int method, Point offset=Point())

    Parameters:

    image – Source, an 8-bit single-channel image. Non-zero pixels are treated as 1’s. Zero pixels remain 0’s, so the image is treated as binary . You can use compare() , inRange() , threshold() , adaptiveThreshold() , Canny() , and others to create a binary image out of a grayscale or color one. The function modifies the image while extracting the contours.
    contours – Detected contours. Each contour is stored as a vector of points.
    hiararchy – Optional output vector containing information about the image topology. It has as many elements as the number of contours. For each contour contours[i] , the elements hierarchy[i][0] , hiearchy[i][1] , hiearchy[i][2] , and hiearchy[i][3] are set to 0-based indices in contours of the next and previous contours at the same hierarchical level: the first child contour and the parent contour, respectively. If for a contour i there are no next, previous, parent, or nested contours, the corresponding elements of hierarchy[i] will be negative.
    mode –

    Contour retrieval mode.
        CV_RETR_EXTERNAL retrieves only the extreme outer contours. It sets hierarchy[i][2]=hierarchy[i][3]=-1 for all the contours.
        CV_RETR_LIST retrieves all of the contours without establishing any hierarchical relationships.
        CV_RETR_CCOMP retrieves all of the contours and organizes them into a two-level hierarchy. At the top level, there are external boundaries of the components. At the second level, there are boundaries of the holes. If there is another contour inside a hole of a connected component, it is still put at the top level.
        CV_RETR_TREE retrieves all of the contours and reconstructs a full hierarchy of nested contours. This full hierarchy is built and shown in the OpenCV contours.c demo.
    method –

    Contour approximation method.
        CV_CHAIN_APPROX_NONE stores absolutely all the contour points. That is, any 2 subsequent points (x1,y1) and (x2,y2) of the contour will be either horizontal, vertical or diagonal neighbors, that is, max(abs(x1-x2),abs(y2-y1))==1.
        CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments and leaves only their end points. For example, an up-right rectangular contour is encoded with 4 points.
        CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS applies one of the flavors of the Teh-Chin chain approximation algorithm. See [TehChin89] for details.
    offset – Optional offset by which every contour point is shifted. This is useful if the contours are extracted from the image ROI and then they should be analyzed in the whole image context.
    */


    vector<vector<cv::Point2i> > contours;
    Mat binaryimageclone = binaryimage.clone();
    cv::findContours(binaryimageclone,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    //cv::findContours(binaryimage,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

    vector<Vec3f> outputpixels;

    Moments mu;
    float Xpix,Ypix,Square;
    // TMP BEGIN
            //Mat dst0 = Mat::zeros(binaryimage.rows, binaryimage.cols, CV_8UC3);
            dsUpdateListOfContoursUsingSquare(contours,minSquare);
            outputcontours = contours;
            //drawContours(dst0,contours,-1,Scalar(0,255,255));
            //imwrite("./tempimage/nullcontors.png",dst0);
    // TMP END

    vector<vector<cv::Point2i> > shabloncontour;

    if (ifstream("contourround.dsshcon") != NULL)
    {
            ifstream ishablons;

            ishablons.open("contourround.dsshcon",ios_base::binary);
            vector<Point2i> tmp;
            int tmp1,tmp2;
            //for (int i=0;i<shabloncontour.size();i++)
            while(ishablons.peek()!=-1)
            {
                ishablons.read((char*)&tmp1,sizeof(int));
                ishablons.read((char*)&tmp2,sizeof(int));

                //shabloncontour[0].push_back(Point2i(tmp1,tmp2));
                tmp.push_back(Point2i(tmp1,tmp2));
                //cout << " round cont " << tmp1 << " " << tmp2 << endl;
            }
            ishablons.close();
            shabloncontour.push_back(tmp);
//                cout<<"file size is "<<ishablons.tellg()
//                      <<" bytes"<<endl;
//                ishablons.close();
            dsUpdateListOfContours(contours,shabloncontour[0],valuesforshapes,method);
            for (int i=0;i<contours.size();i++)
            {
                mu = moments(contours[i],false);
                Xpix = mu.m10/mu.m00;
                Ypix = mu.m01/mu.m00;
                Square = /*sqrt(*/fabs(contourArea(contours[i]))/*/pi)*/;
                outputpixels.push_back(Vec3f(Xpix,Ypix,Square));
            }
#ifdef DEBUG
            cout << "  using \"./contourround.dsshcon\" " << endl;
#endif
            // TMP BEGIN
                    Mat dst = Mat::zeros(binaryimage.rows, binaryimage.cols, CV_8UC3);
                    drawContours(dst,contours,-1,Scalar(0,255,255));
                    imwrite("./tempimage/updatecontors.png",dst);
            // TMP END

    }
    else
    {
        cerr << "template of contour is not found " << endl;
        exit(NOSHABLONFILE);
    }

//    if (cvLoadImage("./shabloncontour2.png",0) != NULL)
//    {
//        IplImage* tmpim = cvLoadImage("./shabloncontour2.png",0);
//        Mat shabloncontourimage = tmpim;

//        findContours(shabloncontourimage,shabloncontour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
//        if (shabloncontour.size()==1)
//        {
//            dsUpdateListOfContours(contours,shabloncontour[0],valuesforshapes,method);
//        }


//        for (int i=0;i<contours.size();i++)
//        {
//            mu = moments(contours[i],false);
//            Xpix = mu.m10/mu.m00;
//            Ypix = mu.m01/mu.m00;
//            Radius = sqrt(fabs(contourArea(contours[i]))/pi);
//            outputpixels.push_back(Vec3f(Xpix,Ypix,Radius));
//        }

//        // TMP BEGIN
//        cout << "  using \"./shabloncontour2.png\" " << endl;
//                Mat dst = Mat::zeros(binaryimage.rows, binaryimage.cols, CV_8UC3);
//                drawContours(dst,contours,-1,Scalar(0,255,255));
//                imwrite("./tempimage/updatecontors.png",dst);
//        // TMP END

//    }
//    else
//    {
//        if (ifstream("contourround.dsshcon") != NULL)
//        {
//                ifstream ishablons;

//                ishablons.open("contourround.dsshcon",ios_base::binary);
//                vector<Point2i> tmp;
//                int tmp1,tmp2;
//                //for (int i=0;i<shabloncontour.size();i++)
//                while(ishablons.peek()!=-1)
//                {
//                    ishablons.read((char*)&tmp1,sizeof(int));
//                    ishablons.read((char*)&tmp2,sizeof(int));

//                    //shabloncontour[0].push_back(Point2i(tmp1,tmp2));
//                    tmp.push_back(Point2i(tmp1,tmp2));
//                    //cout << tmp1 << " " << tmp1 << endl;
//                }
//                ishablons.close();
//                shabloncontour.push_back(tmp);
////                cout<<"file size is "<<ishablons.tellg()
////                      <<" bytes"<<endl;
////                ishablons.close();
//                dsUpdateListOfContours(contours,shabloncontour[0],valuesforshapes,method);
//                for (int i=0;i<contours.size();i++)
//                {
//                    mu = moments(contours[i],false);
//                    Xpix = mu.m10/mu.m00;
//                    Ypix = mu.m01/mu.m00;
//                    Radius = sqrt(fabs(contourArea(contours[i]))/pi);
//                    outputpixels.push_back(Vec3f(Xpix,Ypix,Radius));
//                }

//                // TMP BEGIN
//                        Mat dst = Mat::zeros(binaryimage.rows, binaryimage.cols, CV_8UC3);
//                        drawContours(dst,contours,-1,Scalar(0,255,255));
//                        imwrite("./tempimage/updatecontors.png",dst);
//                // TMP END

//        }
//    }

    return outputpixels;
}

vector<Vec3f> DetectorScan::dsFindPixelsUsingContours(const Mat &binaryimage, double valuesforshapes, int method)
{
    // dsFindPixelsUsingContours(const Mat& binaryimage, vector<vector<Point2i> > outputcontours, double valuesforshapes, int method, double minSquare)
    vector<vector<cv::Point2i> > contours;
    Mat binaryimageclone = binaryimage.clone();
    cv::findContours(binaryimageclone,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    vector<Vec3f> outputpixels;
    Moments mu;
    float Xpix,Ypix,Square;
    vector<vector<cv::Point2i> > shabloncontour;

    if (ifstream("contourround.dsshcon") != NULL)
    {
            ifstream ishablons;

            ishablons.open("contourround.dsshcon",ios_base::binary);
            vector<Point2i> tmp;
            int tmp1,tmp2;
            //for (int i=0;i<shabloncontour.size();i++)
            while(ishablons.peek()!=-1)
            {
                ishablons.read((char*)&tmp1,sizeof(int));
                ishablons.read((char*)&tmp2,sizeof(int));

                //shabloncontour[0].push_back(Point2i(tmp1,tmp2));
                tmp.push_back(Point2i(tmp1,tmp2));
                //cout << " round cont " << tmp1 << " " << tmp2 << endl;
            }
            ishablons.close();
            shabloncontour.push_back(tmp);
//                cout<<"file size is "<<ishablons.tellg()
//                      <<" bytes"<<endl;
//                ishablons.close();
            dsUpdateListOfContours(contours,shabloncontour[0],valuesforshapes,method,true);
            for (int i=0;i<contours.size();i++)
            {
                mu = moments(contours[i],false);
                Xpix = mu.m10/mu.m00;
                Ypix = mu.m01/mu.m00;
                Square = /*sqrt(*/fabs(contourArea(contours[i]))/*/pi)*/;
                outputpixels.push_back(Vec3f(Xpix,Ypix,Square));
            }
#ifdef DEBUG
            cout << "  using \"./contourround.dsshcon\" " << endl;
#endif
//            // TMP BEGIN
//                    Mat dst = Mat::zeros(binaryimage.rows, binaryimage.cols, CV_8UC3);
//                    drawContours(dst,contours,-1,Scalar(0,255,255));
//                    imwrite("./tempimage/updatecontors.png",dst);
//            // TMP END

    }
    else
    {
        cerr << "template of contour is not found " << endl;
        exit(NOSHABLONFILE);
    }
    return outputpixels;
}

void DetectorScan::dsUpdateListOfContours(vector<vector<Point2i> > &inputcontours, vector<Point2i> shabloncontour, double valuesforshapes, int method, bool ifusingsquare, double minArea)
{
    // CV_CONTOURS_MATCH_I1 CV_CONTOURS_MATCH_I2 CV_CONTOURS_MATCH_I3,

    int Niter_t = inputcontours.size();
    int i2 = 0;

    for( int i = 0; i < Niter_t; i++ )
    {
        if (ifusingsquare)
        {
            if (inputcontours[i2].size()<10)
            {
                inputcontours.erase(inputcontours.begin() + i2);
                i2 = i2 - 1;
            }
            else
            {
                //if ()
                if ( (fabs(contourArea(inputcontours[i2]))) < minArea )
                {
                    inputcontours.erase(inputcontours.begin() + i2);
                    i2 = i2 - 1;
                }
                else
                {
                    if (matchShapes(inputcontours[i2],shabloncontour,method,0) > valuesforshapes)
                    {
                        inputcontours.erase(inputcontours.begin() + i2);
                        i2 = i2 - 1;
                    }
                    //i2 = i2 + 1;
                }
            }
        }
        else
        {
            if (matchShapes(inputcontours[i2],shabloncontour,method,0) > valuesforshapes)
            {
                inputcontours.erase(inputcontours.begin() + i2);
                i2 = i2 - 1;
            }
        }
        i2 = i2 + 1;
    }
}

void DetectorScan::dsUpdateListOfContoursUsingSquare(vector<vector<Point2i> > &inputcontours, double minArea)
{
    int Niter_t = inputcontours.size();
    int nstap = 0;
    for (int i=0;i<Niter_t;i++)
    {
        if (inputcontours[nstap].size()<10)
        {
            inputcontours.erase(inputcontours.begin() + nstap);
            nstap = nstap - 1;
        }
        else
        {
            if((fabs(contourArea(inputcontours[nstap]))) < minArea)
            {
                inputcontours.erase(inputcontours.begin() + nstap);
                nstap = nstap - 1;
            }
        }
        nstap++;
    }
}

int DetectorScan::dsRoundValue(float input)
{
    int output;
    int menval = (int)floor(input);
    int bolval = (int)ceil(input);
    if (fabs(input-menval)<0.5) output = menval;
    if (fabs(bolval-input)<=0.5) output = bolval;
    return output;
}

double DetectorScan::dsValueOfContrast(const Mat &image1ch)
{
    return dsSigmaForMat(image1ch);
}

double DetectorScan::dsSigmaForMat(const Mat &image1ch)
{
    int numcol = image1ch.cols;
    int numrow = image1ch.rows;
    Scalar meanval=mean(image1ch);
    double sigma;
    double squarediv = 0;
    int iter = 0;
    for (int i=0;i<numcol;i++)
    {
        for (int j=0;j<numrow;j++)
        {
            squarediv = squarediv + pow((image1ch.at<uchar>(j,i) - meanval[0]),2.0);
            iter++;
        }
    }
    sigma = sqrt((squarediv)/iter);
    return sigma;
}

void DetectorScan::dsShablonContourFile(char* shabloncontourimagename)
{
    Mat shabloncontourimage = imread(shabloncontourimagename,0);
    if (shabloncontourimage.data == NULL)
    {
        cerr << " Image havn`t data for shablon" << endl;
        //return;
        exit(HAVNTDATAFORSHABLON);
    }
    vector<vector<cv::Point2i> > shabloncontour;

    findContours(shabloncontourimage,shabloncontour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    if (shabloncontour.size() != 1) { cerr << " Image havn`t data for shablon" << endl; return;}

    ofstream shablons;
    Point2i tmp2i;
    shablons.open("contourround.dsshcon",ios_base::binary);
    for (int i=0;i<shabloncontour[0].size();i++)
    {
        tmp2i = shabloncontour[0][i];
        shablons.write((char*)&tmp2i.x, sizeof(int));
        shablons.write((char*)&tmp2i.y, sizeof(int));
        //cout << " round cont " << tmp2i.x << " " << tmp2i.y << endl;

    }

    shablons.close();

}

void DetectorScan::dsEqualizeHistMethod(const Mat &in, Mat &out)
{
    vector<Mat> tempin;
    split(in,tempin);
    dsEqualizeHistMethod1ch(tempin[0]);
    dsEqualizeHistMethod1ch(tempin[1]);
    dsEqualizeHistMethod1ch(tempin[2]);
    merge(tempin,out);
}

void DetectorScan::dsEqualizeHistMethod1ch(Mat &InAndOut)
{
    cv::equalizeHist(InAndOut,InAndOut);
}
