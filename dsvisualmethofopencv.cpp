#include "dsvisualmethofopencv.h"
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

#ifdef __MINGW_GCC
#include <dir.h>
#endif
#ifdef unix
#include <sys/stat.h>
#endif

#include "foundcircles.h"

using namespace cv;


dsVisualMethOfOpenCV::dsVisualMethOfOpenCV()
{
}

dsVisualMethOfOpenCV::~dsVisualMethOfOpenCV()
{
}

dsVisualMethOfOpenCV::dsVisualMethOfOpenCV(IplImage* image)
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

dsVisualMethOfOpenCV::dsVisualMethOfOpenCV(const Mat &image)
{
#ifdef DEBUG
    cout << "DEBUG:" << endl;
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
//    cout << endl;
//    cout << " image info: " << endl;

//    cout << setw(70) << setiosflags(ios::left) << " size of image in bytes "<< cout.fill('.')<< setiosflags(ios::right)  << image.size /*->imageSize*/ << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " The size of an aligned image row, in bytes " << cout.fill('.')<< setiosflags(ios::right)  << image->widthStep << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " Image data size in bytes. For interleaved data " <<  cout.fill('.')<< setiosflags(ios::right)  << image->height*image->widthStep << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " Number of channels. " << cout.fill('.')<< setiosflags(ios::right)  << image->nChannels << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " Pixel depth in bits. " << cout.fill('.')<< setiosflags(ios::right)  << image->depth << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " Image height in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->height << resetiosflags(ios::right) << endl;

//    cout << setw(70) << setiosflags(ios::left) << " Image width in pixels " << cout.fill('.')<< setiosflags(ios::right)  << image->width << resetiosflags(ios::right) << endl;

//    cout << endl;

    //InitializationDS();

    SIZE_ROI = 600;

   //MaskParameter.angle = 0;

    if (image.rows > SIZE_ROI && image.cols > SIZE_ROI)
    {
       rect.height = SIZE_ROI; rect.width = SIZE_ROI;
       rect.x = (int)round(image.cols/2-SIZE_ROI/2); rect.y = (int)round(image.rows/2-SIZE_ROI/2);
    }
    else
    {
        rect.x = 0; rect.y = 0; rect.width = image.cols; rect.height = image.rows;
    }
}

IplImage* dsVisualMethOfOpenCV::SelectErodePixel(IplImage* image, METHODERODE method )
{
    /*
     Применение метода эрозии.
   Подбор значений int* value, int count;

   "...
    int cvCreateTrackbar(const char* trackbarName, const char* windowName, int* value, int count, CvTrackbarCallback onChange)

    Creates a trackbar and attaches it to the specified window
    Parameters:

        * trackbarName – Name of the created trackbar.
        * windowName – Name of the window which will be used as a parent for created trackbar.
        * value – Pointer to an integer variable, whose value will reflect the position of the slider. Upon creation, the slider position is defined by this variable.
        * count – Maximal position of the slider. Minimal position is always 0.
        * onChange – Pointer to the function to be called every time the slider changes position. This function should be prototyped as void Foo(int); Can be NULL if callback is not required.

    The function cvCreateTrackbar() creates a trackbar (a.k.a. slider or range control) with the specified name and range,
    assigns a variable to be syncronized with trackbar position and specifies a callback function to be called on trackbar position change.
    The created trackbar is displayed on the top of the given window
    ..."

   "...Эрозия (размывание/сужение) изображения обычно используется для избавления от случайных
   вкраплений на изображении. Идея состоит в том, что вкрапления при размывании устранятся,
   тогда как крупные и соответсвенно более визуально-значимые регионы остаются..."

    Возвращает новую picture.
   */

   IplImage* erode = 0;
   IplImage* erodeclon = 0;

   int radius2 = 1;
   int radius_max=10;
   int iterations2 = 1;
   int iterations_max = 10;
   int radius, iterations;

   cvSetImageROI(image, rect);
   erode = cvCloneImage(image);
   erodeclon = cvCloneImage(erode);

   cvNamedWindow("erode",1);

   cvCreateTrackbar("Radius", "erode", &radius2, radius_max, NULL);
   cvCreateTrackbar("Iterations", "erode", &iterations2, iterations_max, NULL);

   while(1){
       // создаём ядрo
       IplConvKernel* Kern = cvCreateStructuringElementEx(radius2*2+1, radius2*2+1, radius2, radius2, CV_SHAPE_ELLIPSE);

       // выполняем преобразования
       if (method == ERODE) cvErode(erode, erodeclon, Kern, iterations2);
       else cvDilate(erode, erodeclon, Kern, iterations2);

       //cvDilate(image, erode, Kern, iterations);
       // показываем результат

       cvShowImage("erode",erodeclon);

       cvReleaseStructuringElement(&Kern);

       if (waitKey(33) >= 0) break;
   }

   cvDestroyWindow("erode");
   cvStartWindowThread();

   cvResetImageROI(image);

   radius = radius2;
   iterations = iterations2;

   cout << " radius " << radius << " iterations " << iterations << endl;

   cvReleaseImage(&erodeclon);
   cvReleaseImage(&erode);

//   forerode.method = method;
//   forerode.iterations = iterations;
//   forerode.radius = radius;

   if (method == ERODE) return ErodePixel(image,radius,iterations);
   else return DilatePixel(image,radius,iterations);

}

void dsVisualMethOfOpenCV::dsSelectErodePixel(cv::Mat& image, METHODERODE method)
{
    Mat showimage = image(rect);
    Mat tempshowimage;// = image(rect);
    int radius =1;
    int radius_max = 10;
    int iterations = 1;
    int iterations_max = 10;

    int endrad,enditer;




    namedWindow("erode",CV_WINDOW_AUTOSIZE);
    createTrackbar("Radius", "erode", &radius, radius_max, NULL);
    createTrackbar("Iterations", "erode", &iterations, iterations_max, NULL);


    while (1)
    {
//        Mat element = getStructuringElement(MORPH_ELLIPSE,Size(2*radius+1,2*radius+1),Point(radius,radius));
//        Point anchor = Point(-1,-1);


//        if (method == ERODE) erode(tempshowimage,showimage,element,anchor,iterations);
//        else {if (method == DILATE) dilate(tempshowimage,showimage,element,anchor,iterations);
//        else {return image;}}

        if (method == ERODE)
        {
            tempshowimage = showimage.clone();
            dsErodePixel(tempshowimage,radius,iterations);
        }
        else {
            if (method == DILATE)
            {
                tempshowimage = showimage.clone();
                dsDilatePixel(tempshowimage,radius,iterations);
            }
        else {return /*image*/;}}

        imshow("erode",tempshowimage);

        if (waitKey(33) >= 0) break;
    }

    destroyWindow("erode");
    startWindowThread();

    endrad = radius;
    enditer = iterations;

    if (method == ERODE) /*return *//*image = */dsErodePixel(image,radius,iterations);
    else if (method == DILATE) { /*return *//*image = */dsDilatePixel(image,radius,iterations);}
    else return /*image*/;
}

Mat dsVisualMethOfOpenCV::dsVisualCannyAlgorithm(const Mat& image1ch, int* parameters)
{
    int tolddown = 0;
    int toldup = 256;
    int toldmax = 256;
    int aperture = 3;

    Mat tempimage1ch = image1ch.clone();

    namedWindow("original",CV_WINDOW_AUTOSIZE);
    imshow("original",image1ch(rect));
    waitKey(0);
    destroyWindow("original");
    startWindowThread();

    namedWindow("canny",CV_WINDOW_AUTOSIZE);
    createTrackbar("threshold_up", "canny", &toldup, toldmax, NULL);
    createTrackbar("threshold_down", "canny", &tolddown, toldmax, NULL);

    while(1)
    {
        Canny( image1ch, tempimage1ch, tolddown, toldup, aperture );
        imshow("canny",tempimage1ch(rect));
        char c = waitKey(33);
        if (c>0)
        {
            break;
        }
    }
    destroyAllWindows();
    startWindowThread();
    parameters[0] = tolddown;
    parameters[1] = toldup;

    return tempimage1ch;

}

IplImage* dsVisualMethOfOpenCV::dsVisualCannyAlgorithm(IplImage* tempin, int *parameters)
{
    IplImage* tempout = 0;
    tempout = cvCloneImage(tempin);

    int tolddown = 0;       // Порог. Нижнее значение для порога = 0
    int toldup = 256;       // Порог. Верхнее значение для порога = 256
    int toldmax = 256;

    int aperture = 3;       // Параметр функции Канни.

    cvNamedWindow("original",CV_WINDOW_AUTOSIZE);
    // показываем оригинальную картинку
    cvShowImage("original",tempin);
    cvWaitKey(0);
    cvDestroyWindow("original");
    cvStartWindowThread();

    cvNamedWindow("canny",CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("threshold down", "canny", &tolddown, toldmax, NULL/*myTrackbarTOldDown*/);
    cvCreateTrackbar("threshold up", "canny", &toldup, toldmax, NULL/*myTrackbarTOldUp*/);

    while(1)
    {
        // выполняем преобразования
        cvCanny( tempin, tempout, tolddown, toldup, aperture );

        // показываем результат
        cvShowImage("canny",tempout);

        char c = cvWaitKey(33);

        if (c == 27)
               { // если нажата ESC - выходим
                   break;
               }
    }

    cvDestroyAllWindows();
    cvStartWindowThread();

    parameters[0] = tolddown;
    parameters[1] = toldup;

    cout << endl;
    cout << " thresholds Canny alghorytm: " << endl;
    cout << " down threshold    " << parameters[0] << endl;
    cout << " up threshold      " << parameters[1] << endl;
    cout << endl;

//    forcanny.thresholdown = threshold[0];
//    forcanny.thresholdup = threshold[1];


    //cvReleaseImage(&tempin);
    //cvReleaseImage(&tempout);
    cvCanny(tempin,tempout,parameters[0],parameters[1]);
    return tempout;
}

IplImage* dsVisualMethOfOpenCV::VisualSelectChannel(IplImage* image3ch)
{
    if (image3ch->nChannels != 3)
    {
        cout << endl;
        cout << " Number of channels must be 3 " << endl;
        cout << endl;
        return image3ch;
    }

    IplImage* matimage3ch = 0;
    IplImage* matimage1ch = 0;
    cvSetImageROI(image3ch, rect);
    matimage3ch = cvCloneImage(image3ch);

    int sel_canal = 1;
    int numofchannel = 7;
    int swcan;
    int canal;
    string selectchan;

    cvNamedWindow("select_channel",CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("select_channel","select_channel",&sel_canal,numofchannel);

    while(1)
    {
        swcan = sel_canal;
        switch(swcan)
        {
        case 1:
            selectchan = " R_CANAL ";
            canal = R_CANAL;
            matimage1ch = SelectChannel(matimage3ch,R_CANAL);
            break;
        case 2:
            selectchan = " G_CANAL ";
            canal = G_CANAL;
            matimage1ch = SelectChannel(matimage3ch,G_CANAL);
            break;
        case 3:
            selectchan = " B_CANAL ";
            canal = B_CANAL;
            matimage1ch = SelectChannel(matimage3ch,B_CANAL);
            break;
        case 4:
            selectchan = " H_CANAL ";
            canal = H_CANAL;
            matimage1ch = SelectChannel(matimage3ch,H_CANAL);
            break;
        case 5:
            selectchan = " S_CANAL ";
            canal = S_CANAL;
            matimage1ch = SelectChannel(matimage3ch,S_CANAL);
            break;
        case 6:
            selectchan = " V_CANAL ";
            canal = V_CANAL;
            matimage1ch = SelectChannel(matimage3ch,V_CANAL);
            break;
        case 7:
            selectchan = " GRAY ";
            canal = GRAY;
            matimage1ch = SelectChannel(matimage3ch,GRAY);
            break;
        default:
            selectchan = " GRAY ";
            canal = GRAY;
            matimage1ch = SelectChannel(matimage3ch,GRAY);
        }

        cvShowImage("select_channel",matimage1ch);

        if (waitKey(33) > 0 ) break;
    }

//    forcanny.canal_c = canal;
    cvDestroyWindow("select_channel");
    cvStartWindowThread();

    cout << endl;
    cout << " you have chosen" << selectchan << "canal " << endl;
    cout << endl;

    cvResetImageROI(image3ch);
    cvReleaseImage(&matimage3ch);
    cvReleaseImage(&matimage1ch);

    return SelectChannel(image3ch,canal);
}

void dsVisualMethOfOpenCV::VisualSelectChannel(Mat& image3ch)
{
    if (image3ch.channels() != 3)
    {
        cout << endl;
        cout << " Number of channels must be 3 " << endl;
        cout << endl;
        //return image3ch;
    }

    Mat matimage3ch = image3ch(rect);
    Mat matimage1ch;// = matimage3ch.clone();
    int sel_canal = 1;
    int numofchannel = 7;
    int ifnegative = 1;
    int ifnegbeg = 1;
    int swcan;
    int canal;
    string selectchan;

    namedWindow("select_channel",CV_WINDOW_AUTOSIZE);
    createTrackbar("select_channel","select_channel",&sel_canal,numofchannel);
    createTrackbar("negative","select_channel",&ifnegative,ifnegbeg);

    while(1)
    {
        swcan = sel_canal;
        matimage1ch = matimage3ch.clone();
        switch(swcan)
        {
        case 1:
            selectchan = " R_CANAL ";
            canal = R_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,R_CANAL);
            break;
        case 2:
            selectchan = " G_CANAL ";
            canal = G_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,G_CANAL);
            break;
        case 3:
            selectchan = " B_CANAL ";
            canal = B_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,B_CANAL);
            break;
        case 4:
            selectchan = " H_CANAL ";
            canal = H_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,H_CANAL);
            break;
        case 5:
            selectchan = " S_CANAL ";
            canal = S_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,S_CANAL);
            break;
        case 6:
            selectchan = " V_CANAL ";
            canal = V_CANAL;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,V_CANAL);
            break;
        case 7:
            selectchan = " GRAY ";
            canal = GRAY;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,GRAY);
            break;
        default:
            selectchan = " GRAY ";
            canal = GRAY;
            /*matimage1ch = */SelectChannel(matimage1ch,ifnegative,GRAY);
        }



        imshow("select_channel",matimage1ch);

        if (waitKey(33) > 0 ) break;
    }

//    forcanny.canal_c = canal;
    destroyWindow("select_channel");
    startWindowThread();

    cout << endl;
    cout << " you have chosen" << selectchan << "canal " << endl;
    cout << endl;

    /*image3ch = */SelectChannel(image3ch,ifnegative,canal);
}

ParForHoughCicles dsVisualMethOfOpenCV::dsParameterFindCircles(IplImage* image, IplImage* image1ch)
{
    int param1 = 50;
    int param2 = 100;
    int parammax = 300;
    int radiuslo = 50;
    int radiushi = 250;
    int radiusmax = 500;
    int min_dist = (int)(image->width/20);
    int min_distM = 500;

    ParForHoughCicles HCircles;

    //Rect rect(0,0,500,500);
    //image = GetDetectorROI(image, rect);
    cvSetImageROI(image,rect);
    cvSetImageROI(image1ch,rect);
    IplImage* image1 = 0;

    IplImage* s_plane = cvCreateImage( cvGetSize(image1ch), 8, 1 );
    s_plane = cvCloneImage(image1ch);

    cvNamedWindow( "cvHoughCircles", CV_WINDOW_AUTOSIZE);
    cvNamedWindow( "cvHough", CV_WINDOW_AUTOSIZE );


    cvCreateTrackbar("Param1", "cvHough", &param1,parammax, NULL);
    cvCreateTrackbar("Param2", "cvHough", &param2,parammax, NULL);
    cvCreateTrackbar("Min Radius", "cvHough", &radiuslo, radiusmax, NULL);
    cvCreateTrackbar("Max Radius", "cvHough", &radiushi, radiusmax, NULL);
    cvCreateTrackbar("min distation", "cvHough", &min_dist,min_distM, NULL);
    CvSeq* results = 0;

    while (1)
    {
      image1 = cvCloneImage(image);
        CvMemStorage* storage = cvCreateMemStorage(0);
      cvSmooth(s_plane, s_plane, CV_GAUSSIAN, 1, 1 );
      /*CvSeq*/ results = cvHoughCircles(
        //image,
        //gray,
        s_plane,
        storage,
        CV_HOUGH_GRADIENT,
        2,
        //s_plane->width/20,
        (double)min_dist+1,
        (double)param1+1,(double)param2+1,radiuslo,radiushi
      );

      for( int i = 0; i < results->total; i++ ) {
        float* p = (float*) cvGetSeqElem( results, i );
        CvPoint pt = cvPoint( cvRound( p[0] ), cvRound( p[1] ));
        cvCircle(image1, pt, cvRound( p[2] ),CV_RGB(/*0xff,0xff,0xff*/0xff,0x00,0x00));
        //cout << i << "   ";
      }
     //cout << endl;

     cvShowImage( "cvHoughCircles", image1);
     cvShowImage( "cvHough", s_plane);

     char c = cvWaitKey(33);

     if (c == /*27*/13 || c == 27)
            { // если нажата ESC - выходим
                /*cvDestroyWindow("original");
                cvDestroyWindow("erode");*/
         //cvDestroyAllWindows();
         cvReleaseImage(&image1);
                break;
            }
    cvReleaseImage(&image1);
        }
    double md,p1,p2;
    int rh,rl;
    md = (double)min_dist + 1;
    p1 = (double)param1+1;
    p2 = (double)param2+1;
    rh = radiushi;
    rl = radiuslo;

    HCircles.mindist = md;
    HCircles.parameter1 = p1;
    HCircles.parameter2 = p2;
    HCircles.radiushigh = rh;
    HCircles.radiuslow = rl;

    cout << " md " << md << " p1 " << p1 << " p2 " << p2 << " rh " << rh  <<  " rl " << rl << endl;

    cout << " results->total " << results->total << endl;

    for( int i = 0; i < results->total; i++ ) {
        float* p = (float*) cvGetSeqElem( results, i );
        cout << " results: " << endl;
        cout << " x: " << p[0] << " y: " << p[1] << endl;
        cout << " radius: " << p[2] << endl;
        cout << endl;
    }
    //results2 = results;
    /*HCircles->mindist = (double)min_dist + 1;
    HCircles->parameter1 = (double)param1+1;
    HCircles->parameter2 = (double)param2+1;
    HCircles->radiushigh = radiushi;
    HCircles->radiuslow = radiuslo;*/

    cout << " number of circles found " << results->total << endl;

    cvDestroyAllWindows();
    cvStartWindowThread();

    cvResetImageROI(image);
    cvResetImageROI(image1ch);
    //cvReleaseImage(&image1);
    /*cvReleaseImage(&gray);
    cvReleaseImage(&hsvimage);
    cvReleaseImage(&h_plane);*/
    cvReleaseImage(&s_plane);
    //StartWindowThread;

    //cvReleaseImage(&v_plane);


    cout << " ee " << endl;
    return HCircles;
}

//ParForHoughCicles dsVisualMethOfOpenCV::cppParameterFindCircles(IplImage* image, IplImage* image1ch)
//{
//    //Rect rect(0,0,500,500);


//    /*
//    cout << " 3ch cols " << newimage.cols  << " rows " << newimage.rows << " temp " << newimage.elemSize() << endl;
//    cout << " 1ch cols " << newimage1ch.cols  << " rows " << newimage1ch.rows << " temp " << newimage1ch.elemSize() << endl;
//    cout << " ch " << newimage.channels() <<  " 1ch " << newimage1ch.channels() << endl;
//    cout << " type " << newimage.type() <<  " 1ch " << newimage1ch.type() << endl;
//    cout << " total " << newimage.total() <<  " 1ch " << newimage1ch.total() << endl;*/

//    int param1 = 30;
//    int param2 = 75;
//    int parammax = 300;
//    int radiuslo = 30;
//    int radiushi = 70;
//    //int radiushi = 34;
//    int radiusmax = 500;
//    //int min_dist = (int)newimage.rows/20;
//    int min_dist = 50;
//    int min_distM = 500;

//    ParForHoughCicles HCircles;

//    Mat newimage1ch = Mat(image1ch,1);
//    if (forgaussblur.boolfgb)
//    GaussianBlur(newimage1ch, newimage1ch,
//                 forgaussblur.ksize,
//                 forgaussblur.sigma1,
//                 forgaussblur.sigma2);

//    newimage1ch = newimage1ch(rect);

//    namedWindow( "cvHough", CV_WINDOW_AUTOSIZE );
//    //Mat tempimage = Mat(image1ch,rect);

//    imshow("cvHough",newimage1ch);
//    waitKey(0);
//    destroyWindow("cvHough");
//    startWindowThread();

//    namedWindow( "cvHoughCircles", CV_WINDOW_AUTOSIZE);

//    createTrackbar("Param1", "cvHoughCircles", &param1,parammax, NULL);
//    createTrackbar("Param2", "cvHoughCircles", &param2,parammax, NULL);
//    createTrackbar("Min Radius", "cvHoughCircles", &radiuslo, radiusmax, NULL);
//    createTrackbar("Max Radius", "cvHoughCircles", &radiushi, radiusmax, NULL);
//    createTrackbar("min distation", "cvHoughCircles", &min_dist,min_distM, NULL);

//    while (1)
//    {
//        Mat newimage = Mat(image,1);
//        //Mat newimage1ch = Mat(image1ch,1);//cvarrToMat(image1ch);
//        vector<Vec3f> circles;

//        newimage = newimage(rect);
//        //newimage1ch = newimage1ch(rect);

//      //GaussianBlur( newimage1ch, newimage1ch, Size(9, 9), 2, 2 );
//      HoughCircles(newimage1ch, circles, CV_HOUGH_GRADIENT,2,
//                   (double)min_dist+1,
//                   (double)param1+1,
//                   (double)param2+1,
//                   radiuslo,
//                   radiushi);

//      for( size_t i = 0; i < circles.size(); i++ )
//          {
//               Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//               int radius = cvRound(circles[i][2]);
//               // draw the circle center
//               circle( newimage, center, 3, Scalar(0,255,0), -1, 2, 0 );
//               // draw the circle outline
//               circle( newimage, center, radius, Scalar(0,0,255), 3, 2, 0 );
//          }

//      imshow("cvHoughCircles",newimage);


//      char c = waitKey(33);
//      if (c == /*27*/13 || c == 27)
//      { // если нажата ESC - выходим
//                break;
//            }
//        } // while

//    destroyWindow("cvHoughCircles");
//    startWindowThread();

//    double md,p1,p2;
//    int rh,rl;
//    md = (double)min_dist + 1;
//    p1 = (double)param1+1;
//    p2 = (double)param2+1;
//    rh = radiushi;
//    rl = radiuslo;

//    HCircles.mindist = md;
//    HCircles.parameter1 = p1;
//    HCircles.parameter2 = p2;
//    HCircles.radiushigh = rh;
//    HCircles.radiuslow = rl;

//    return HCircles;
//}

//ParForHoughCicles dsVisualMethOfOpenCV::dsParameterFindCircles(const Mat& image3ch,Mat& image1ch, vector<Vec3f>& circles)
//{
//    //Rect rect(0,0,500,500);


//    /*
//    cout << " 3ch cols " << newimage.cols  << " rows " << newimage.rows << " temp " << newimage.elemSize() << endl;
//    cout << " 1ch cols " << newimage1ch.cols  << " rows " << newimage1ch.rows << " temp " << newimage1ch.elemSize() << endl;
//    cout << " ch " << newimage.channels() <<  " 1ch " << newimage1ch.channels() << endl;
//    cout << " type " << newimage.type() <<  " 1ch " << newimage1ch.type() << endl;
//    cout << " total " << newimage.total() <<  " 1ch " << newimage1ch.total() << endl;*/

//    circles.clear();
//    int param1 = 30;
//    int param2 = 75;
//    int parammax = 300;
//    int radiuslo = 30;
//    int radiushi = 70;
//    //int radiushi = 34;
//    int radiusmax = 500;
//    //int min_dist = (int)newimage.rows/20;
//    int min_dist = 50;
//    int min_distM = 500;

//    ParForHoughCicles HCircles;

//    if (image1ch.channels()!=1)
//    {
//        cerr << " in function cppParameterFindCircles(image), image must be have 1 channel";
//        /*image1ch = */VisualSelectChannel(image1ch);
//    }

//    //Mat newimage1ch = image1ch;
//    if (forgaussblur.boolfgb)
//    GaussianBlur(image1ch, image1ch,
//                 forgaussblur.ksize,
//                 forgaussblur.sigma1,
//                 forgaussblur.sigma2);

//    //newimage1ch = newimage1ch(rect);

//    namedWindow( "cvHough", CV_WINDOW_AUTOSIZE );
//    //Mat tempimage = Mat(image1ch,rect);

//    imshow("cvHough",image1ch(rect));
//    waitKey(0);
//    destroyWindow("cvHough");
//    startWindowThread();

//    namedWindow( "cvHoughCircles", CV_WINDOW_AUTOSIZE);

//    createTrackbar("Param1", "cvHoughCircles", &param1,parammax, NULL);
//    createTrackbar("Param2", "cvHoughCircles", &param2,parammax, NULL);
//    createTrackbar("Min Radius", "cvHoughCircles", &radiuslo, radiusmax, NULL);
//    createTrackbar("Max Radius", "cvHoughCircles", &radiushi, radiusmax, NULL);
//    createTrackbar("min distation", "cvHoughCircles", &min_dist,min_distM, NULL);

//    while (1)
//    {
//        vector<Vec3f> tempcircles;
//        Mat newimage = image3ch.clone();
//        newimage = newimage(rect);
//        //Mat newimage1ch = Mat(image1ch,1);//cvarrToMat(image1ch);
//        //vector<Vec3f> circles;

//        //newimage = newimage(rect);
//        Mat newimage1ch = image1ch(rect);

//      //GaussianBlur( newimage1ch, newimage1ch, Size(9, 9), 2, 2 );
//      HoughCircles(newimage1ch, tempcircles, CV_HOUGH_GRADIENT,2,
//                   (double)min_dist+1,
//                   (double)param1+1,
//                   (double)param2+1,
//                   radiuslo,
//                   radiushi);

//      for( size_t i = 0; i < tempcircles.size(); i++ )
//          {
//               Point center(cvRound(tempcircles[i][0]), cvRound(tempcircles[i][1]));
//               int radius = cvRound(tempcircles[i][2]);
//               // draw the circle center
//               circle( newimage, center, 3, Scalar(0,255,0), -1, 2, 0 );
//               // draw the circle outline
//               circle( newimage, center, radius, Scalar(0,0,255), 3, 2, 0 );
//          }

//      imshow("cvHoughCircles",newimage);
//      imwrite("rrr.png",newimage);


//      char c = waitKey(33);
//      if (c>0)
//      { // если нажата ESC - выходим
//         tempcircles.clear();
//         break;
//      }
//      tempcircles.clear();
//      newimage.empty();
//    } // while

//    destroyWindow("cvHoughCircles");
//    startWindowThread();

//    double md,p1,p2;
//    int rh,rl;
//    md = (double)min_dist + 1;
//    p1 = (double)param1+1;
//    p2 = (double)param2+1;
//    rh = radiushi;
//    rl = radiuslo;



//    HCircles.mindist = md;
//    HCircles.parameter1 = p1;
//    HCircles.parameter2 = p2;
//    HCircles.radiushigh = rh;
//    HCircles.radiuslow = rl;

//    HoughCircles(image1ch,circles,CV_HOUGH_GRADIENT,2,
//                 HCircles.mindist,
//                 HCircles.parameter1,
//                 HCircles.parameter2,
//                 HCircles.radiuslow,
//                 HCircles.radiushigh);

//    cout << " found " << circles.size() << " circles "  << endl;

//    return HCircles;
//}


//ParForHoughCicles dsVisualMethOfOpenCV::TEMPcppParameterFindCircles(IplImage* image, IplImage* image1ch)
//{
//    //Rect rect(0,0,500,500);


//    /*
//    cout << " 3ch cols " << newimage.cols  << " rows " << newimage.rows << " temp " << newimage.elemSize() << endl;
//    cout << " 1ch cols " << newimage1ch.cols  << " rows " << newimage1ch.rows << " temp " << newimage1ch.elemSize() << endl;
//    cout << " ch " << newimage.channels() <<  " 1ch " << newimage1ch.channels() << endl;
//    cout << " type " << newimage.type() <<  " 1ch " << newimage1ch.type() << endl;
//    cout << " total " << newimage.total() <<  " 1ch " << newimage1ch.total() << endl;*/

//    int param1 = 30;
//    int param2 = 75;
//    int parammax = 300;
//    int radiuslo = 33;
//    //int radiushi = 70;
//    int radiushi = 37;
//    int radiusmax = 500;
//    //int min_dist = (int)newimage.rows/20;
//    int min_dist = 50;
//    int min_distM = 500;

//    ParForHoughCicles HCircles;

//    Mat newimage1ch = Mat(image1ch,1);
//    if (forgaussblur.boolfgb)
//    GaussianBlur(newimage1ch, newimage1ch,
//                 forgaussblur.ksize,
//                 forgaussblur.sigma1,
//                 forgaussblur.sigma2);

//    newimage1ch = newimage1ch(rect);

////    namedWindow( "cvHough", CV_WINDOW_AUTOSIZE );
////    //Mat tempimage = Mat(image1ch,rect);

////    imshow("cvHough",newimage1ch);
////    waitKey(0);
////    destroyWindow("cvHough");
////    startWindowThread();

////    namedWindow( "cvHoughCircles", CV_WINDOW_AUTOSIZE);

////    createTrackbar("Param1", "cvHoughCircles", &param1,parammax, NULL);
////    createTrackbar("Param2", "cvHoughCircles", &param2,parammax, NULL);
////    createTrackbar("Min Radius", "cvHoughCircles", &radiuslo, radiusmax, NULL);
////    createTrackbar("Max Radius", "cvHoughCircles", &radiushi, radiusmax, NULL);
////    createTrackbar("min distation", "cvHoughCircles", &min_dist,min_distM, NULL);

//    while (1)
//    {
//        Mat newimage = Mat(image,1);
//        //Mat newimage1ch = Mat(image1ch,1);//cvarrToMat(image1ch);
//        vector<Vec3f> circles;

//        newimage = newimage(rect);
//        //newimage1ch = newimage1ch(rect);

//      //GaussianBlur( newimage1ch, newimage1ch, Size(9, 9), 2, 2 );
//      HoughCircles(newimage1ch, circles, CV_HOUGH_GRADIENT,2,
//                   (double)min_dist+1,
//                   (double)param1+1,
//                   (double)param2+1,
//                   radiuslo,
//                   radiushi);

//      for( size_t i = 0; i < circles.size(); i++ )
//          {
//               Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//               int radius = cvRound(circles[i][2]);
//               // draw the circle center
//               circle( newimage, center, 3, Scalar(0,255,0), -1, 2, 0 );
//               // draw the circle outline
//               circle( newimage, center, radius, Scalar(0,0,255), 3, 2, 0 );
//          }

////      imshow("cvHoughCircles",newimage);


//      char c = waitKey(33);
//      if (1)
//      //if (c == /*27*/13 || c == 27)
//      { // если нажата ESC - выходим
//                break;
//            }
//        } // while

////    destroyWindow("cvHoughCircles");
////    startWindowThread();

//    double md,p1,p2;
//    int rh,rl;
//    md = (double)min_dist + 1;
//    p1 = (double)param1+1;
//    p2 = (double)param2+1;
//    rh = radiushi;
//    rl = radiuslo;

//    HCircles.mindist = md;
//    HCircles.parameter1 = p1;
//    HCircles.parameter2 = p2;
//    HCircles.radiushigh = rh;
//    HCircles.radiuslow = rl;

//    return HCircles;
//}

IplImage* dsVisualMethOfOpenCV::SelectThreshold(IplImage* image1ch)
{
    //IplImage* outimage1ch = 0;
    Mat matimage1ch = Mat(image1ch,1);
    Mat image1chroi = matimage1ch(rect);
    int mvalue = 200;
    int ithreshold = 100;
    int maxthreshold = 255;
    int maxvalue = 255;

    namedWindow("threshold",CV_WINDOW_AUTOSIZE);
    createTrackbar("threshold","threshold",&ithreshold,maxthreshold);
    createTrackbar("value","threshold",&mvalue,maxvalue);
    Mat newimage1chroi;// = image1chroi;
    while(1)
    {
        cv::threshold(image1chroi,newimage1chroi,(double)ithreshold,(double)mvalue,CV_THRESH_BINARY);
        imshow("threshold",newimage1chroi);
        if (waitKey(33)>=0) break;
    }
    destroyWindow("threshold");
    startWindowThread();

    forthresh.told = (double)ithreshold;
    forthresh.value = (double)mvalue;

    return dsPostThreshold(image1ch,(double)ithreshold,(double)mvalue);
}

void dsVisualMethOfOpenCV::SelectThreshold(Mat& image1ch)
{
    //Mat matimage1ch = Mat(image1ch,1);
    Mat image1chroi = image1ch(rect);
    int mvalue = 200;
    int ithreshold = 100;
    int maxthreshold = 255;
    int maxvalue = 255;

    namedWindow("threshold",CV_WINDOW_AUTOSIZE);
    createTrackbar("threshold","threshold",&ithreshold,maxthreshold);
    createTrackbar("value","threshold",&mvalue,maxvalue);
    Mat newimage1chroi;// = image1chroi;
    while(1)
    {
        cv::threshold(image1chroi,newimage1chroi,(double)ithreshold,(double)mvalue,CV_THRESH_BINARY);
        imshow("threshold",newimage1chroi);
        if (waitKey(33)>=0) break;
    }
    destroyWindow("threshold");
    startWindowThread();

    /*image1ch =*/ dsPostThreshold(image1ch,(double)ithreshold,(double)mvalue);

    //return image1ch ;
}

void dsVisualMethOfOpenCV::temprun(Mat& matscan)
{
    //_fullpath() - polnoe imya fasila s directorii
    // _splitpath() - razlozenia i,ya faila

    char tempimagedir[40];
    char tempresdir[10];
    char bufnamefilemoments[200];
    strcpy(tempimagedir,"./tempimage");
    strcpy(tempresdir,"./tmpres/");
    std::string namemomentsfile(tempimagedir);
    namemomentsfile = namemomentsfile + "/" + "moments.dat";
    string namecontrimagefile = string(tempimagedir) + "/" + "contr.png";
    cout << namemomentsfile << endl;


    //if (chdir(tempimagedir)!=0) mkdir(tempimagedir);
#ifdef __MINGW_GCC
    if (access(tempimagedir,0)!=0) mkdir(tempimagedir);
    if (access(tempresdir,0)!=0) mkdir(tempresdir);
#endif

#ifdef unix
    if (access(&tmpresdir.c_str()[0],0)!=0) mkdir(&tmpresdir.c_str()[0],0777);
    if (access(&tmpimdir.c_str()[0],0)!=0) mkdir(&tmpimdir.c_str()[0],0777);
#endif


    dsSelectErodePixel(matscan);
    VisualSelectChannel(matscan);
    //Negative1ch(matscan);
    SelectThreshold(matscan);
    imwrite("ddf.png",matscan);

//    imwrite("./tempimage/nerotate.png",matscan);
    Mat tmpmatscan = matscan.clone();
    vector<Vec3f> foundpixels = dsFindPixelsUsingContours(tmpmatscan,0.002,CV_CONTOURS_MATCH_I1);

    imwrite("./tempimage/nerotate.png",matscan);
    FoundCircles* pixels = new FoundCircles;

    Vec2d AverageDistAng;

//    vector<cv::Vec2d> vec1 = pixels->SortForDistance(foundpixels);
//    AverageDistAng = pixels->dsMeanForVector2d(/*pixels->SortForDistance(foundpixels)*/vec1);
//    cout << "AverageDistAng[0] "  << AverageDistAng[0] << " AverageDistAng[1] " << AverageDistAng[1] << endl;
    cout << " foundpixels.size() " << foundpixels.size() << endl;
    if (foundpixels.size()==0) return;
    double rotate = pixels->dsAngleForRotateImageDegrees(foundpixels,LSPFIT);
    cout << " rotate = " << rotate << endl;
    cout << " foundpixels.size() " << foundpixels.size() << endl;
    foundpixels.clear();


//    IplImage* src = cvLoadImage("2_AG-66-1-10x.jpeg",1);
//    rotateImage(src,0);

    rotateImage(matscan,rotate);
    cout << " GI " << endl;
    imwrite("./tempimage/rotate.png",matscan);

    tmpmatscan = matscan.clone();

    foundpixels = dsFindPixelsUsingContours(tmpmatscan,0.002,CV_CONTOURS_MATCH_I1);
    //pixels->SortForDistance(foundpixels);
    vector<Vec2d> tmpvec = pixels->SortForDistance(foundpixels);
    AverageDistAng = pixels->dsMeanForVector2d(tmpvec);
    vector<Vec6i> Vert;
    vector<Vec6i> Horis;
    pixels->dsSearchBorders(foundpixels,Vert,Horis,AverageDistAng);


    ofstream Borders("Borders.out");
    Borders << " Vertical Borders " << endl;
    Borders << " number of columns  = " << Vert.size() << endl;
    for (int i=0;i<Vert.size();i++)
    {
        Borders << " [" << Vert[i][0] << ";" << "" << Vert[i][1]<< "]  ..........." << " [" << Vert[i][2] << ";" <<  Vert[i][3]<< "]" << "      " << " Fit " << Vert[i][4] << " angle (mrad) = " << Vert[i][5] << endl;
    }
    Borders << endl;
    Borders << endl;

    Borders << " Horisontal Borders " << endl;
    Borders << " number of rows  = " << Horis.size() << endl;

    for (int i=0;i<Horis.size();i++)
    {
        Borders << " [" << Horis[i][0] << ";" << "" << Horis[i][1]<< "]  ..........." << " [" << Horis[i][2] << ";" << Horis[i][3]<< "]"<< "      " << " Fit " << Horis[i][4] << " angle (mrad) = " << Horis[i][5]  << endl;
    }
    Borders.close();

    vector<Vec2i> Gridx;
    vector<Vec2i> Gridy;
//        int* Gridx[2] = new int[NV][2];
//        int* Gridy[2] = new int[NH][2];

    Vec2i imsiz(matscan.cols,matscan.rows);

    pixels->dsGetParametersOfGrid(Horis,Vert,Gridx,Gridy,AverageDistAng[0],imsiz);
    Rect temprect;

    tmpmatscan = matscan.clone();

    for(int i=0;i<Gridx.size();i++)
    {
        temprect.x = Gridx[i][0];
        temprect.width = Gridx[i][1];
        for(int j=0;j<Gridy.size();j++)
        {
            temprect.y = Gridy[j][0];
            temprect.height = Gridy[j][1];
//                rectangle(matimage,temprect,Scalar(0,0,255),3,1,0);
            rectangle(tmpmatscan,temprect,Scalar(255),1,1,0);
        }
    }


    imwrite("./tempimage/Grid.png",tmpmatscan);
}
