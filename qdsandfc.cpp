#include "qdsandfc.h"


QDSandFC::QDSandFC()
{
}

QDSandFC::QDSandFC(const QImage &qimage)
{
    Loadimage = QDSQImage2MatConvert(qimage);
}

QDSandFC::QDSandFC(const uchar *cameradata, long size)
{
    std::vector<uchar> buffer(cameradata, cameradata+size);
    Loadimage = cv::imdecode(buffer,1);
}

cv::Mat QDSandFC::QDSQImage2MatConvert(const QImage &qimage)
{
    cv::Mat mat(qimage.height(),qimage.width(),CV_8UC4,(uchar*)qimage.bits(), qimage.bytesPerLine());
    cv::Mat outputmat = cv::Mat(mat.rows, mat.cols, CV_8UC3 );
    cv::cvtColor(mat,outputmat,CV_RGBA2RGB);
    return outputmat;
}

QImage QDSandFC::QDSMat2QImageConvert(const cv::Mat &matimage)
{
    // 8-bits unsigned, NO. OF CHANNELS=1

    if(matimage.type()==CV_8UC1)
    {
        QVector<QRgb> colorTable;
        // Set the color table (used to translate colour indexes to qRgb values)
        for (int i=0;i<256;i++) colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar* qImageBuffer1 = (const uchar*)matimage.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer1,matimage.cols,matimage.rows,matimage.step,QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(matimage.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar* qImageBuffer3 = (const uchar*)matimage.data;
        QImage img(qImageBuffer3,matimage.cols,matimage.rows,matimage.step,QImage::Format_RGB888);
        //rgbSwapped() - инверсия цвета
        return img.rgbSwapped();
    }
    else
    {
        cerr << " ERROR: Mat could not be converted to QImage. " << endl;
        return QImage();
    }
}

double QDSandFC::QGetContrast(const QImage &qimage, int grid, int koorx, int koory)
{
    cv::Mat QimToMat = QDSQImage2MatConvert(qimage);
    vector<vector<double> > contrast;
    dsGetContrast(QimToMat,grid,contrast);
    if (koorx >grid || koory>grid)
    {
        if (koorx>grid) koorx = grid-1;
        if (koory>grid) koory = grid-1;
        // if koorx || koory MORE THAN grid+1
        if (koorx>grid) koorx = 0;
        if (koory>grid) koory = 0;
    }
    return contrast[koorx][koory];
}

double QDSandFC::QGetContrast(int grid, int koorx, int koory)
{
    vector<vector<double> > contrast;
    //if (!Loadimage.data) return 0;
    dsGetContrast(Loadimage,grid,contrast);
    if (koorx >grid || koory>grid)
    {
        if (koorx>grid) koorx = grid-1;
        if (koory>grid) koory = grid-1;
        // if koorx || koory MORE THAN grid+1
        if (koorx>grid) koorx = 0;
        if (koory>grid) koory = 0;
    }
    return contrast[koorx][koory];
}

cv::Mat QDSandFC::QDSGetMatFromCharData(const uchar *cameradata, long int size)
{
    std::vector<uchar> buffer(cameradata, cameradata+size);
    return imdecode(buffer,1);
}

void QDSandFC::QSetDSpreamp(QImage &qimage, int radius, int iteration, int canal, int ifnegative, int ifthreshold, int threshold)
{
    cv::Mat matimage = QDSQImage2MatConvert(qimage);
    dsErodePixel(matimage,radius,iteration);

    if (canal>0)
    {
        SelectChannel(matimage,ifnegative,canal);
        if (ifthreshold)
        {
            dsPostThreshold(matimage,(double)threshold,255.);
        }
    }


    //cout << matimage.channels() << "  ";
    qimage = QDSMat2QImageConvert(matimage);
}

vector<vector<double> > QDSandFC::QGetContrast(const cv::Mat& MatImage,int grid)
{
    vector<vector<double> > contrast;
    dsGetContrast(MatImage,grid,contrast);
    return contrast;
}

vector<vector<double> > QDSandFC::QGetContrast(const QImage &qimage, int grid)
{
    cv::Mat QimToMat = QDSQImage2MatConvert(qimage);
    vector<vector<double> > contrast;
    dsGetContrast(QimToMat,grid,contrast);
    return contrast;
}

vector<vector<double> > QDSandFC::QGetContrast(int grid)
{
    vector<vector<double> > contrast;
    dsGetContrast(Loadimage,grid,contrast);
    return contrast;
}


//QImage QDSandFC::QDSMat2QImageConvert(const cv::Mat &matimage) // OLDVERSION
//{
//    cv::Mat color;
//    if (matimage.channels() == 1)
//    {
//        std::vector<cv::Mat> mergeqimage3ch;
//        color.create(cv::Size(matimage.cols,matimage.rows),CV_8UC3);
//        mergeqimage3ch.push_back(matimage);
//        mergeqimage3ch.push_back(matimage);
//        mergeqimage3ch.push_back(matimage);
//        cv::merge(mergeqimage3ch,color);
//    }
//    else
//    {
//        if (matimage.channels() == 3)
//        {
//            color = matimage.clone;
//            cv::cvtColor(color,color,CV_BGR2RGB);

//            QImage pImage(color.cols,color.rows,QImage::Format_RGB888);

//            IplImage im = (IplImage)color;
//            //По неизвестным причинам pImage->bytesPerLine() и im.widthStep могут не совпадать
//            if(pImage.bytesPerLine()!=im.widthStep)
//            {
//                //Копирование по строкам
//                for(int i=0;i<pImage.height();i++)
//                {
//                    memcpy(pImage.bits()+i*pImage.bytesPerLine(),color.data+im.widthStep*i,im.widthStep);
//                }
//            }else{
//                //Копирование всего изображения
//                memcpy(pImage.bits(),color.data,im.imageSize);
//            }
//            return pImage;
//        }
//        else
//        {
//            cerr << " Unknown format Mat image \n";
//            return QImage();
//        }
//    }
//}
//convert(cv::Mat image)
//{
//    cv::flip(image,image,1); // process the image
//    // change color channel ordering
//    cv::cvtColor(image,image,CV_BGR2RGB); // or rgbSwapped() for Qimage???
//    // Qt image
//    QImage img= QImage((const unsigned char*)(image.data),image.cols,image.rows,QImage::Format_RGB888);
//}
