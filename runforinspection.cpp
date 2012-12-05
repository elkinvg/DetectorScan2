#include "runforinspection.h"
using namespace std;
#include <fstream>
#include <iostream>
#include <iomanip>
#ifdef __MINGW_GCC
#include <dir.h>
#endif
#ifdef unix
#include <sys/stat.h>
#include <stdio.h>
#endif

using namespace cv;

runforinspection::runforinspection()
{
}

runforinspection::runforinspection(Mat &image, int canal, int ifnegative, int Methoderode, int eroderadius, int erodeiter, int Threshold, int MaxThreshold, int shapematch, double valforshapes)
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
    cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"<<endl;
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

void runforinspection::run(cv::Mat &image)
{
//    void DetectorScan::run(Mat &image)
//    {
    //    char tempresdir[10];
    //    strcpy(tempresdir,tmpresdir.c_str());
    #ifdef __MINGW_GCC
        if (access(&tmpresdir.c_str()[0],0)!=0) mkdir(&tmpresdir.c_str()[0]);
        if (access(&tmpimdir.c_str()[0],0)!=0) mkdir(&tmpimdir.c_str()[0]);
    #endif

    #ifdef unix
        if (access(&tmpresdir.c_str()[0],0)!=0) mkdir(&tmpresdir.c_str()[0],0777);
        if (access(&tmpimdir.c_str()[0],0)!=0) mkdir(&tmpimdir.c_str()[0],0777);
    #endif
#ifdef DEBUG
        cout << " run " << endl;
#endif
        vector<vector<double> > valofcon;
        dsGetContrast(image,9,valofcon);
#ifdef DEBUG
        for (int i=0;i<valofcon.size();i++)
        {
            for (int j=0;j<valofcon[i].size();j++)
            {
                cout << valofcon[i][j] << " ";
            }
            cout << endl;
        }
#endif
        //Mat imageclon = image.clone();
    //    vector<int> param;
    //    param.push_back(CV_IMWRITE_JPEG_QUALITY);
    //    param.push_back(70);
    //    imwrite("sd.jpeg",imageclon,param);
    //    param.clear();
    //    param.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //    param.push_back(12);
    //    imwrite("sd.png",imageclon,param);
        //Mat imshift = image.clone();
        //dsShiftImage(imshift,1000,1000);
        //dsResizeImage(image,imshift,0.25);

        Mat Eqil = image.clone();
        //dsEqualizeHistMethod(image,Eqil);
        //dsCutROIofImage(Eqil,Point2i(0,0),Point2i(100,100));
        dsCutROIofImage(Eqil,Vec4i(100,100,image.cols+100,image.rows+100));
        imwrite("./tempimage/equil.png",Eqil);

        //SelectChannel(Eqil,InputParam.ifnegative,InputParam.canal);
//        imwrite("./tempimage/selectch.png",Eqil);
//        dsEqualizeHistMethod1ch(Eqil);
//        imwrite("./tempimage/selectchequil.png",Eqil);



        /*image = */dsErodePixel(image,InputParam.eroderadius,InputParam.erodeiter);
        /*image = */SelectChannel(image,InputParam.ifnegative,InputParam.canal);
        //dsEqualizeHistMethod1ch(image);



        //double FoundThreshold = dsFindOptimalThreshold(image,InputParam.valforshapes,InputParam.shapematch,0);

        dsPostThreshold(image,(double)InputParam.Threshold,(double)InputParam.MaxThreshold);
        Mat image1ch = image/*.clone()*/;

        float square = 0;
        float squaremin,squaremax;

        vector<Vec3f> foundpixels = dsFindPixelsUsingContours(image1ch,InputParam.valforshapes, InputParam.shapematch);

        if (foundpixels.size() < MinNumOfPixels)
        {
            cerr << " Number of pixels < than Minimum " << endl;
            cerr << " Minimum = " << MinNumOfPixels << endl;
            exit(PIXELSFOUNDLESSTHAN);
        }

        //FoundCircles* pixels = new FoundCircles;

        Vec2d AverageDistAng;

        //double rotate1 = /*pixels->*/dsAngleForRotateImageDegrees(foundpixels,LSPFIT,VERTICAL,1);
        double rotate = /*pixels->*/dsAngleForRotateImageDegrees(foundpixels,LSPFIT,HORISONT,1);

#ifdef DEBUG
        cout << " rotate H = " << rotate << /*" rotate V = " << rotate1 <<*/ endl;
#endif
        foundpixels.clear();

        //dsShiftImage(image,45);
        imwrite("./tempimage/prerotate.png",image1ch);


        rotateImage(image1ch,rotate);
        //image1ch = image.clone();
        imwrite("./tempimage/rotate.png",image1ch);

        VecP2i contoursofpixel;
        foundpixels = dsFindPixelsUsingContours(image1ch,contoursofpixel,InputParam.valforshapes, InputParam.shapematch,300.);

        Mat dst = Mat::zeros(image1ch.rows, image1ch.cols, CV_8UC3);
        drawContours(dst,contoursofpixel,-1,Scalar(0,255,255));
        imwrite("./tempimage/allcontours.png",dst);


        if (foundpixels.size() < MinNumOfPixels)
        {
            cerr << " Number of pixels < than Minimum " << endl;
            cerr << " Minimum = " << MinNumOfPixels << endl;
            return;
        }

        for (int i=0;i<foundpixels.size();i++)
        {
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
        square = square/foundpixels.size();


#ifdef DEBUG
        cout << " found " << foundpixels.size() << " pixels " << endl;
        cout << " equal square = " << square << "    min square = " << squaremin << "    max square = " << squaremax << endl;
#endif

        Vec4i CorDet = /*pixels->*/dsCornerDetect(foundpixels,SizePict);

#ifdef DEBUG
        cout << endl;
        cout << " North Border " << CorDet[0] << endl;
        cout << " East Border " << CorDet[1] << endl;
        cout << " South Border " << CorDet[2] << endl;
        cout << " West Border " << CorDet[3] << endl;
        cout << endl;
#endif


        vector<Vec2d> tmpvec = /*pixels->*/SortForDistance(foundpixels);

        AverageDistAng = /*pixels->*/dsMeanForVector2d(tmpvec);

#ifdef DEBUG
        cout << " AverageDistAng " << AverageDistAng[0] << " AverageDistAng " << AverageDistAng[1] << endl;
#endif

        vector<Vec6i> Vert;
        vector<Vec6i> Horis;
        /*pixels->*/dsSearchBorders(foundpixels,Vert,Horis,AverageDistAng,0);

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

        //image1ch = image.clone();

        Vec2i imsiz(image1ch.cols,image1ch.rows);

        /*pixels->*/dsGetParametersOfGrid(Horis,Vert,Gridx,Gridy,AverageDistAng[0],imsiz);
        Rect temprect;
        // Сетку получили, теперь поиск дефетков.
        vector<vector<int> > ifdefectsind;
        imwrite("./tempimage/tmp0.png",image1ch);
        ifdefectsind = dsFindDefects(image,Gridx,Gridy,foundpixels,Vec6i(0,0,1,1,1,1));
        imwrite("./tempimage/tmp1.png",image1ch);
        //dsFindRelatedDefect(findDefects);
        //dsAnalysisDefect(image1ch,ifdefectsind,Gridx,Gridy);


        // Тут мы получаем по данной картинке сетку с координатами
        // Дальше ... сохранить картинку с индексом ХкоординатаУкоордината.пнг
        // Где координата это область которую снимает камера.

        //cout << " findDefects.size() " << ifdefectsind.size() << " findDefects[0].size() " << ifdefectsind[0].size() << endl;

        for(int i=0;i<Gridx.size();i++)
        {
            temprect.x = Gridx[i][0];
            temprect.width = Gridx[i][1];
            for(int j=0;j<Gridy.size();j++)
            {
                temprect.y = Gridy[j][0];
                temprect.height = Gridy[j][1];
    //                rectangle(matimage,temprect,Scalar(0,0,255),3,1,0);
                rectangle(image1ch,temprect,Scalar(255),1,1,0);
            }
        }

            imwrite("./tempimage/Grid.png",image1ch);

//    }
}

