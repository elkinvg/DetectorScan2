#include "foundcircles.h"
#include <iostream>
#include <fstream>

using namespace cv;


FoundCircles::FoundCircles()
{
}

void FoundCircles::SortPointsOfCirclesOnX(vector<cv::Vec3f>& circles, cv::Vec2d AverageDistAngle, int *columns, bool CoutComments)
{
    vector<Vec3f> tempsort;
    //vector<Vec3f> sortcircles;
    Vec3f tempbegin;
    Vec3f xminymin;

    int iterations;// = circles.size();
    int niter;
    int I=0;

    double MeanDistance = AverageDistAngle[0];
    double MeanAngle = AverageDistAngle[1]*2*pi/360;

    //float **tyr = new float[4];

    while(circles.size()>0)
    {
        niter = 0;
        iterations = circles.size();
        for (int i=0; i<iterations; i++)
        {
            if (i == 0)
            {
                xminymin[0] = circles[i][0]; xminymin[1] = circles[i][1]; xminymin[2]=circles[i][2];
                circles.erase(circles.begin() + i + niter);
                niter--;
                tempsort.push_back(xminymin);
            }
            else
            {
                tempbegin[0] = circles[i+niter][0]; tempbegin[1] = circles[i+niter][1]; tempbegin[2] = circles[i+niter][2];
                if ((xminymin[0] - tempbegin[0])<((xminymin[1] - tempbegin[1])*tan(MeanAngle)+MeanDistance*cos(MeanAngle)/2) && (xminymin[0] - tempbegin[0])>((xminymin[1] - tempbegin[1])*tan(MeanAngle)-MeanDistance*cos(MeanAngle)/2))
                {
                    tempsort.push_back(tempbegin);
                    circles.erase(circles.begin() + i + niter);
                    niter--;
                }
            }
        }
        if (tempsort.size()>0)
        {
            if (tempsort.size()>1)
            {
                SortPointsOfTempCirclesX(tempsort);
                //AvAngle = AvAngle + Angle[0];
                I++;
            }
            else
            {
                I++;
                if (CoutComments == 1)
                {
                    cout << " 1 cell on column [" <<  tempsort[0][0] << ";" << tempsort[0][1] << "] " << endl;
                }
            }
        }
        for (int k=0;k<tempsort.size();k++)
        {
            Vec3f tmpve3f; tmpve3f[0] = tempsort[k][0];tmpve3f[1] = tempsort[k][1];tmpve3f[2] = tempsort[k][2];
            circles.push_back(tmpve3f);
        }
        tempsort.clear();
    }
#ifdef DEBUG
    if (CoutComments == 1)
    {
        cout << "****************************" << endl;
        cout << " number of columns = " << I << endl;
        cout << "****************************" << endl;
    }
#endif
    columns[0] = I;
}

void FoundCircles::SortPointsOfTempCirclesX(vector<Vec3f> &tempcircles, bool CoutComments)
{
    vector<Vec3f> sortcircles;
    sortcircles.clear();
    Vec3f xminymin;
    int iterations = tempcircles.size();
    int niter;

    for (int j=0; j<iterations;j++)
    {
        for (int i=0; i<tempcircles.size();i++)
        {
            if (i==0) {xminymin[0] = tempcircles[i][0]; xminymin[1] = tempcircles[i][1];xminymin[2] = tempcircles[i][2]; niter = i;}
        }

        sortcircles.push_back(xminymin);
        tempcircles.erase(tempcircles.begin()+niter);
    }

    tempcircles.clear();

    Point2f Beg,End;

    iterations = sortcircles.size();
#ifdef DEBUG
    if ( sortcircles.size() > 1)
    {
        Beg.x = sortcircles[0][0]; Beg.y = sortcircles[0][1];
        End.x = sortcircles[iterations-1][0]; End.y = sortcircles[iterations-1][1];

        if (CoutComments == 1)
        {
            cout << " Angle " << AngleBetLinesDeg(Beg,End) << " Dist " << DistBet2points(Beg,End)
                << " Begin [" << Beg.x << ";" << Beg.y << "] "
                << " End [" << End.x << ";" << End.y << "]" << endl;
        }

    }
#endif
    tempcircles = sortcircles;
}

double FoundCircles::DistBet2points(cv::Point2d pt1, cv::Point2d pt2)
{
    /*
      расстояние между двумя точками
      */

    /*if (coor1 == NULL && coor2 == NULL)
    {return 0;}*/

    double distance;
    float x1 = pt1.x; float y1 = pt1.y;
    float x2 = pt2.x; float y2 = pt2.y;
    distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    return distance;
}

double FoundCircles::AngleBetLinesDeg(cv::Point2d pt1, cv::Point2d pt2)
{
    double x1 = pt1.x; double y1 = pt1.y;
    double x2 = pt2.x; double y2 = pt2.y;

    return 360*(atan((y2-y1)/(x2-x1)))/(2*pi);
}

int FoundCircles::fcRoundValue(float input)
{
    int output;
    int menval = (int)floor(input);
    int bolval = (int)ceil(input);
    if (fabs(input-menval)<0.5) output = menval;
    if (fabs(bolval-input)<=0.5) output = bolval;
    return output;
}

vector<cv::Vec2d> FoundCircles::SortForDistance(const vector<cv::Vec3f>& circles)
{
    // Нахождение ближайшей ячейки для каждой ячейки
    //  disandangle - вектор из значений соответственно
    //  расстояний до ближайших ячеек и наклон линии
    //  (угла к горизонту) проведенной через центры ячеек

    //std::sort(circles.begin(),circles.end());
    Point2f pt1,pt2;
    double dist1, dist2;
    double angle;
    int iter1;

//    Point2f L_cell; //coordinate of the leftmost cell
//    Point2f R_cell; //coordinate of the rightmost cell
//    Point2f U_cell; //coordinate of the uppermost cell
//    Point2f D_cell; //coordinate of the bottom cell

    //Vec2d MiniMax;
    double minval,maxval;

    vector<Vec2d> disandangle;

    for (int i=0; i<circles.size();i++)
    {
        Vec2d disan(0,0);
        disandangle.push_back(disan);
    }

    //cout << " dis " << disandangle.size();

    for (int i=0; i<circles.size()-1;i++)
    {
        pt1.x = circles[i][0];  pt1.y = circles[i][1];
        for (int j=i+1; j<circles.size();j++)
        {
            pt2.x = circles[j][0];  pt2.y = circles[j][1];
            if(j == (i+1) ) dist1 = DistBet2points(pt1,pt2);

            if (i == 0)
            {
                if (j == (i+1)) {minval = DistBet2points(pt1,pt2); maxval = 0;}
                disandangle[j][0] = DistBet2points(pt1,pt2);
                disandangle[j][1] = AngleBetLinesDeg(pt1,pt2);
            }

            dist2 = DistBet2points(pt1,pt2);
            if(dist2<dist1) {dist1=dist2; iter1 = j; angle = AngleBetLinesDeg(pt1,pt2);}

            if (dist2 < disandangle[j][0])
            {disandangle[j][0] = dist2; disandangle[j][1] = AngleBetLinesDeg(pt1,pt2);}

        }
        if (i == 0)
        {
            disandangle[i][0] = dist1;
            disandangle[i][1] = angle;
        }
        else
        {if(dist1<disandangle[i][0]) {disandangle[i][0] = dist1; disandangle[i][1] = angle;}}

        if (disandangle[i][0]< minval) minval = disandangle[i][0];
        if (disandangle[i][0] > maxval) maxval = disandangle[i][0];

        //if (circles[i][0] < )

    }
#ifdef DEBUG
    cout << endl;
    cout << " minimum value of distance " << minval << endl;
    cout << " maximum value of distance " << maxval << endl;
    cout << endl;
#endif
    //MiniMax= Vec2d(minval,maxval);

    return disandangle;
}

cv::Vec2d FoundCircles::dsMeanForVector2d(vector<cv::Vec2d> &DistAngle)
{
    /*
      вычисление средних значений для элементов вектора 2d
      Cоответсвенно для 1й и 2й колонки (значения Vec2d = (double,double))
      */
    Vec2d Meandisangle;
    if (DistAngle.size() == 0) return Meandisangle = Vec2d(0,0);
    double MeanDis = 0;
    double MeanAngle = 0;

    int temppos,tempneg;
    temppos = tempneg = 0;
    int posorneg; // positive or negative

    for (int i=0;i<DistAngle.size();i++)
    {
        MeanDis = MeanDis + DistAngle[i][0];

        if(DistAngle[i][1]>45 || DistAngle[i][1]<-45)
        {
            if (DistAngle[i][1]>45) {MeanAngle = MeanAngle + (90.-DistAngle[i][1]);tempneg++;}
            else { MeanAngle = MeanAngle + (90.+DistAngle[i][1]);temppos++;}
        }
        else
        {
            if (DistAngle[i][1]>0) {MeanAngle = MeanAngle + DistAngle[i][1];temppos++;}
            else {MeanAngle = MeanAngle + abs(DistAngle[i][1]);tempneg++;}
        }
    }
    if (tempneg<temppos) posorneg = 1;
    else posorneg = -1;


    Meandisangle[0] = MeanDis/DistAngle.size();
    Meandisangle[1] = posorneg*MeanAngle/DistAngle.size();

    return Meandisangle;
}

void FoundCircles::dsSearchBorders(const vector<Vec3f> &circles, vector<Vec6i> &Vert, vector<Vec6i> &Horis, Vec2d AverageDistAngle, bool CoutComments)
{
    Vert = dsSearchBoundaryPoints(circles,AverageDistAngle,VERTICAL,CoutComments);
    Horis = dsSearchBoundaryPoints(circles,AverageDistAngle,HORISONT,CoutComments);
}

void FoundCircles::dsGetParametersOfGrid(const vector<Vec6i> &Horis, const vector<Vec6i> &Vert, vector<Vec2i> &gridx, vector<Vec2i> &gridy, float AverageDist,Vec2i sizeofpict)
{
    Vec2i tempvec;
    int NH = Horis.size(); // Horisontal
    int NV = Vert.size(); // Vertical

    int iter;
    int tmpwidth;
    int obwidth;

    vector<int> Hmet;
    vector<int> Vmet;

    for (int i=0;i<NH;i++)
    {
        if (i!=NH-1)
        {
            if (i==0)
            {
                if ((Horis[i+1][4]-Horis[i][4])/1.5 < AverageDist)
                {
                    if ((Horis[i][4]-((Horis[i+1][4]-Horis[i][4])/2)) < 0 ) Hmet.push_back(0);
                    else Hmet.push_back((Horis[i][4]-fcRoundValue((Horis[i+1][4]-Horis[i][4])/2)));
                    Hmet.push_back(fcRoundValue((Horis[i+1][4] + Horis[i][4])/2));
                }
                else
                {
                    iter = fcRoundValue((Horis[i+1][4]-Horis[i][4])/AverageDist);
                    tmpwidth = fcRoundValue((Horis[i+1][4]-Horis[i][4])/(2*iter));
                    obwidth = (Horis[i+1][4]-Horis[i][4]);
                    if ((Horis[i][4] - tmpwidth) < 0) Hmet.push_back(0);
                    else Hmet.push_back((Horis[i][4] - tmpwidth));
                    for (int j=0;j<iter;j++)
                    {
                        if (j==0) {Hmet.push_back(Horis[i][4]+ tmpwidth); obwidth = obwidth - tmpwidth;}
                        else
                        {
                            if (j==iter-1) {Hmet.push_back(Horis[i+1][4] /*- (obwidth */-tmpwidth/*)*/);obwidth = obwidth - tmpwidth;}
                            else {Hmet.push_back(Hmet[Hmet.size()-1]+2*tmpwidth); obwidth = obwidth - 2*tmpwidth;}
                        }
                    }
                }
            }
            else
            {
                if ((Horis[i+1][4]-Horis[i][4])/1.5 < AverageDist)
                {
                    Hmet.push_back(fcRoundValue((Horis[i+1][4]+Horis[i][4])/2));
                }
                else
                {
                    iter = fcRoundValue((Horis[i+1][4]-Horis[i][4])/AverageDist);
                    tmpwidth = fcRoundValue((Horis[i+1][4]-Horis[i][4])/(2*iter));
                    obwidth = (Horis[i+1][4]-Horis[i][4]);
                    for (int j=0;j<iter;j++)
                    {
                        if (j==0) {Hmet.push_back(Horis[i][4]+ tmpwidth); obwidth = obwidth - tmpwidth;}
                        else
                        {
                            if (j==iter-1) {Hmet.push_back(Horis[i+1][4] - /*(obwidth -*/tmpwidth/*)*/);obwidth = obwidth - tmpwidth;}
                            else {Hmet.push_back(Hmet[Hmet.size()-1]+2*tmpwidth); obwidth = obwidth - 2*tmpwidth;}
                        }
                    }
                }
            }
        }
        else
        {
            tmpwidth = Horis[i][4] - Hmet[Hmet.size()-1];

            Hmet.push_back(Horis[i][4]+tmpwidth);
        }
    }

    for (int i=0;i<NV;i++)
    {
        if (i!=NV-1)
        {
            if (i==0)
            {
                if ((Vert[i+1][4]-Vert[i][4])/1.5 < AverageDist)
                {
                    if ((Vert[i][4]-((Vert[i+1][4]-Vert[i][4])/2)) < 0 ) Vmet.push_back(0);
                    else Vmet.push_back((Vert[i][4]-fcRoundValue((Vert[i+1][4]-Vert[i][4])/2)));
                    Vmet.push_back(fcRoundValue((Vert[i+1][4]+Vert[i][4])/2));
                }
                else
                {
                    iter = fcRoundValue((Vert[i+1][4]-Vert[i][4])/AverageDist);
                    tmpwidth = fcRoundValue((Vert[i+1][4]-Vert[i][4])/(2*iter));
                    obwidth = (Vert[i+1][4]-Vert[i][4]);
                    if ((Vert[i][4] - tmpwidth) < 0) Vmet.push_back(0);
                    else Vmet.push_back((Vert[i][4] - tmpwidth));
                    for (int j=0;j<iter;j++)
                    {
                        if (j==0) {Vmet.push_back(Vert[i][4]+ tmpwidth); obwidth = obwidth - tmpwidth;}
                        else
                        {
                            if (j==iter-1) {Vmet.push_back(Vert[i+1][4] -/* (obwidth -*/tmpwidth/*)*/);obwidth = obwidth - tmpwidth;}
                            else {Vmet.push_back(Vmet[Vmet.size()-1]+2*tmpwidth); obwidth = obwidth - 2*tmpwidth;}
                        }
                    }
                }
            }
            else
            {
                if ((Vert[i+1][4]-Vert[i][4])/1.5 < AverageDist)
                {
                    Vmet.push_back(fcRoundValue((Vert[i+1][4]+Vert[i][4])/2));
                }
                else
                {
                    iter = fcRoundValue((Vert[i+1][4]-Vert[i][4])/AverageDist);
                    tmpwidth = fcRoundValue((Vert[i+1][4]-Vert[i][4])/(2*iter));
                    obwidth = (Vert[i+1][4]-Vert[i][4]);
                    for (int j=0;j<iter;j++)
                    {
                        if (j==0) {Vmet.push_back(Vert[i][4]+ tmpwidth); obwidth = obwidth - tmpwidth;}
                        else
                        {
                            if (j==iter-1) {Vmet.push_back(Vert[i+1][4] - /*(obwidth -*/tmpwidth/*)*/);obwidth = obwidth - tmpwidth;}
                            else {Vmet.push_back(Vmet[Vmet.size()-1]+2*tmpwidth); obwidth = obwidth - 2*tmpwidth;}
                        }
                    }
                }
            }
        }
        else
        {
            tmpwidth = Vert[i][4] - Vmet[Vmet.size()-1];

            Vmet.push_back(Vert[i][4]+tmpwidth);
        }
    }

    ofstream yu("grids.out");
    for (int i=0;i<(Hmet.size()-1);i++)
    {
        tempvec[0]=Hmet[i];
        tempvec[1]=Hmet[i+1]-Hmet[i];
        if (tempvec[0]+tempvec[1]>sizeofpict[1]) tempvec[1] = sizeofpict[1] - tempvec[0];
        gridy.push_back(tempvec);
        yu << "Y" << i << "  " << gridy[i][0] << "   " << (gridy[i][0] + gridy[i][1]) <<  " height " << gridy[i][1] << endl;
    }
#ifdef DEBUG
    cout << endl;
#endif
    for (int i=0;i<(Vmet.size()-1);i++)
    {
        tempvec[0]=Vmet[i];
        tempvec[1]=Vmet[i+1]-Vmet[i];
        if (tempvec[0]+tempvec[1]>sizeofpict[0]) tempvec[1] = sizeofpict[0] - tempvec[0];
        gridx.push_back(tempvec);
        yu << "X" << i << "  " << gridx[i][0] << "   " << (gridx[i][0] + gridx[i][1]) <<  " width " << gridx[i][1] << endl;
    }
    yu.close();
}

Vec4i FoundCircles::dsCornerDetect(const vector<Vec3f>& circles, Vec2i SizePict)
{
    // найденные контуры на входе должны быть желательно с уже повёрнутых картинок
    // по часовой стрелке North East South West
    vector<Vec2d> tmpvector = SortForDistance(circles);
    Vec2d AverageDistAng = dsMeanForVector2d(tmpvector);
    tmpvector.clear();
    int NorBor,SouthBor,WestBor,EastBor;
    NorBor = SouthBor = WestBor = EastBor = 0;

    vector<Vec6i> Vert;
    vector<Vec6i> Horis;

    dsSearchBorders(circles,Vert,Horis,AverageDistAng,0);

    int NH = Horis.size() - 1;
    int NV = Vert.size() - 1;

    if (Horis[0][4] > FactorOfBorder*AverageDistAng[0]) NorBor = 1;
    if (Horis[NH][4] < (SizePict[1]-FactorOfBorder*AverageDistAng[0])) SouthBor = 1;
    if (Vert[0][4] > FactorOfBorder*AverageDistAng[0]) WestBor = 1;
    if (Vert[NV][4] < (SizePict[0]-FactorOfBorder*AverageDistAng[0])) EastBor = 1;
#ifdef DEBUG
    cout << " FactorOfBorder " << FactorOfBorder << endl;
#endif

    return Vec4i(NorBor,EastBor,SouthBor,WestBor);
}

double FoundCircles::dsAngleForRotateImageDegrees(const vector<cv::Vec3f>& vec,int Method,int direct, bool CoutComments)
{
    vector<Vec2d> vec1 = SortForDistance(vec);
    Vec2d distangle;
    double angleofrotate;

    distangle = dsMeanForVector2d(vec1);
#ifdef DEBUG
    cout << " ||||| " << distangle[0] << " ||||| " << distangle[1] << endl;
#endif
    //vector<Vec3f> vecSORT = SortPointsOfCirclesOnY(vec,distangle);

    angleofrotate = (360/(2*pi))*dsAngleOfRotationRadian(vec,distangle,Method,direct,CoutComments);
    //angleofrotate = 360*MaskParameter.angle/(2*pi);
#ifdef DEBUG
    cout << " ||||| ->>>>> " << angleofrotate << endl;
#endif
    return angleofrotate;
}

double FoundCircles::dsAngleOfRotationRadian(const vector<cv::Vec3f>& circles, cv::Vec2d AverageDistAngle, int Method, int direct, bool CoutComments)
{
    // Method = METHODOFFINDOFANGLE {SIMPLE = 1, ROOTFIT = 2, SPFIT = 3}
    // direct = DIRECTION { HORISONT = 1, VERTICAL = 2 };
    vector<Vec3f> tempsort;

    double MeanDistance = AverageDistAngle[0];
    double MeanAngle = AverageDistAngle[1]*2*pi/360;
    Vec3f tempbegin;

    double AvAngle = 0; // calculate of the average angle
    int I=0;  // iterations

    Vec3f xminymin;
    int iterations; // = circles.size(); // number of circles
    int niter;
    vector<cv::Vec3f> tempcircles = circles;

    while(tempcircles.size()>0)
    {
        niter = 0;
        iterations = tempcircles.size();
        for (int i=0; i<iterations; i++)
        {
            if (i == 0)
            {
                xminymin[0] = tempcircles[i][0]; xminymin[1] = tempcircles[i][1]; xminymin[2]=tempcircles[i][2];
                tempcircles.erase(tempcircles.begin() + i + niter);
                niter--;
                tempsort.push_back(xminymin);
            }
            else
            {
                tempbegin[0] = tempcircles[i+niter][0]; tempbegin[1] = tempcircles[i+niter][1]; tempbegin[2] = tempcircles[i+niter][2];
                if (direct == HORISONT)
                {
                    if ((xminymin[1] - tempbegin[1])<((xminymin[0] - tempbegin[0])*tan(MeanAngle)+MeanDistance*cos(MeanAngle)/2) && (xminymin[1] - tempbegin[1])>((xminymin[0] - tempbegin[0])*tan(MeanAngle)-MeanDistance*cos(MeanAngle)/2))
                    {
                        tempsort.push_back(tempbegin);
                        tempcircles.erase(tempcircles.begin() + i + niter);
                        niter--;
                    }
                }
                if (direct == VERTICAL)
                {
                    if ((xminymin[0] - tempbegin[0])<((tempbegin[1] - xminymin[1] )*tan(MeanAngle)+MeanDistance*cos(MeanAngle)/2) && (xminymin[0] - tempbegin[0])>((tempbegin[1] - xminymin[1])*tan(MeanAngle)-MeanDistance*cos(MeanAngle)/2))
                    {
                        tempsort.push_back(tempbegin);
                        tempcircles.erase(tempcircles.begin() + i + niter);
                        niter--;
                    }
                }
            }
        }
        if (tempsort.size()>1)
        {
            Vec2d tmp;// = new double[2];
            switch(Method)
            {
            case SIMPLE :
                AvAngle = AvAngle + dsAngleBetweenBegEndRadian(tempsort,CoutComments); I++;
                break;
            case LSPFIT :
                tmp = LSPFitLineRadian(tempsort,direct,CoutComments);
                AvAngle = AvAngle + atan(tmp[1]); I++;
                break;
            default :
                AvAngle = AvAngle + dsAngleBetweenBegEndRadian(tempsort); I++;
            }

            //AvAngle = AvAngle + dsAngleBetweenBegEndRadian(tempsort); I++;
            //AvAngle = AvAngle + (2*pi)*dsLinFitForCellLineDegree(tempsort,1)/360; I++;
        }
        tempsort.clear();
    }

    AvAngle = AvAngle/I;
#ifdef DEBUG
    cout << " Angle  |||||||||  " <<  AvAngle << endl;
#endif
    return AvAngle;
}

double FoundCircles::dsAngleBetweenBegEndRadian(const vector<cv::Vec3f>& tempcircles, bool CoutComments)
{
    //static int r = 0; r++;
    //if (r == 4) dsLinFitForCellLineDegree(tempcircles);
    int iterations = tempcircles.size();
    Point2f Beg,End;
    float min,max;
    int B,E; // numbers of iteration
    double Angle;

    for (int j=0; j<iterations;j++)
    {
        if ( j == 0 ) {min = tempcircles[j][0]; max = tempcircles[j][0]; B = E = j;}
        else
        {
            if (tempcircles[j][0] < min ) {min = tempcircles[j][0]; B = j;}
            if (tempcircles[j][0] > max ) {max = tempcircles[j][0]; E = j;}
        }
    }
    Beg.x = tempcircles[B][0]; Beg.y = tempcircles[B][1];
    End.x = tempcircles[E][0]; End.y = tempcircles[E][1];
    Angle = 2*pi*AngleBetLinesDeg(Beg,End)/360;
#ifdef DEBUG
    if (CoutComments == 1)
    {
        cout << " Angle " << AngleBetLinesDeg(Beg,End) << " Dist " << DistBet2points(Beg,End)
                << " Begin [" << Beg.x << ";" << Beg.y << "] "
                << " End [" << End.x << ";" << End.y << "]" << endl;
    }
#endif
    return Angle;
}

Vec2d FoundCircles::LSPFitLineRadian(const vector<cv::Vec3f> &circles, int direct, bool CoutComments)
{    
    // direct = DIRECTION { HORISONT = 1, VERTICAL = 2 };
    int N = circles.size();
    int negkoeff;
    double X,Y;
    double koeffa,koeffb;
    double sumx,sumy,sumxy,sumx2;
    sumx = sumy = sumxy = sumx2 = 0;
    for (int i=0;i<N;i++)
    {
        if ( direct == HORISONT) {X = circles[i][0]; Y = circles[i][1]; negkoeff = 1;}
        if ( direct == VERTICAL) {Y = circles[i][0]; X = circles[i][1]; negkoeff = -1;}
        sumx = sumx + X/*/N*/;
        sumy = sumy + Y/*/N*/;
        sumxy = sumxy + X*Y/*/N*/;
        sumx2 = sumx2 + X*X/*/N*/;
    }

    koeffb = negkoeff*(N*sumxy - sumx*sumy)/(N*sumx2 - sumx*sumx);
    koeffa = (sumy - koeffa*sumx)/N;
#ifdef DEBUG
    if (CoutComments == 1)
    {
        cout << " coefficient a = " << koeffa << " coefficient b  " << koeffb << endl;
    }
#endif
    return Vec2d(koeffa,koeffb);
}

vector<Vec6i> FoundCircles::dsSearchBoundaryPoints(const vector<Vec3f>& circles, Vec2d AverageDistAngle, int DIRECT, bool CoutComments)
{
    vector<cv::Vec6i> TempBorders; TempBorders.clear();
    vector<cv::Vec6i> Borders; Borders.clear();
    Vec6i Border;

    int I=0;
    int niter;
    vector<Vec3f> tempsort;
    Vec3f tempbegin;
    Vec3f xminymin;
    float AvAngle = 0;
    float Angle[1];

    double MeanDistance = AverageDistAngle[0];
    double MeanAngle = AverageDistAngle[1]*2*pi/360;


    int iterations;
    vector<Vec3f> tempcircles = circles;

    while(tempcircles.size()>0)
    {
        niter = 0;
        iterations = tempcircles.size();
        for (int i=0; i<iterations; i++)
        {
            if (i == 0)
            {
                xminymin[0] = tempcircles[i][0]; xminymin[1] = tempcircles[i][1]; xminymin[2]=tempcircles[i][2];
                tempcircles.erase(tempcircles.begin() + i + niter);
                niter--;
                tempsort.push_back(xminymin);
            }
            else
            {
                tempbegin[0] = tempcircles[i+niter][0]; tempbegin[1] = tempcircles[i+niter][1]; tempbegin[2] = tempcircles[i+niter][2];
                if (DIRECT == HORISONT)
                {
                    if ((xminymin[1] - tempbegin[1])<((xminymin[0] - tempbegin[0])*tan(MeanAngle)+MeanDistance*cos(MeanAngle)/2) && (xminymin[1] - tempbegin[1])>((xminymin[0] - tempbegin[0])*tan(MeanAngle)-MeanDistance*cos(MeanAngle)/2))
                    {
                        tempsort.push_back(tempbegin);
                        tempcircles.erase(tempcircles.begin() + i + niter);
                        niter--;
                    }
                }
                if (DIRECT == VERTICAL)
                {
                    if ((xminymin[0] - tempbegin[0])<((tempbegin[1] - xminymin[1])*tan(MeanAngle)+MeanDistance*cos(MeanAngle)/2) && (xminymin[0] - tempbegin[0])>((tempbegin[1] - xminymin[1])*tan(MeanAngle)-MeanDistance*cos(MeanAngle)/2))
                    {
                        tempsort.push_back(tempbegin);
                        tempcircles.erase(tempcircles.begin() + i + niter);
                        niter--;
                    }
                }
            }
        }
        if (tempsort.size()>0)
        {
            if(tempsort.size()>1)
            {
                Border = dsTempBoundaryPoints(tempsort,DIRECT,CoutComments); I++;
                TempBorders.push_back(Border);
            }
            else
            {
                Border[0] = Border[2] = (int)tempsort[0][0];
                Border[1] = Border[3] = (int)tempsort[0][1];
                if (DIRECT == HORISONT) Border[4] = (int)tempsort[0][1];
                else Border[4] = (int)tempsort[0][0];

                Border[5] = 0;
                TempBorders.push_back(Border);
                I++;
#ifdef DEBUG
                if (CoutComments == 1)
                {
                    cout << " 1 cell on row [" <<  tempsort[0][0] << ";" << tempsort[0][1] << "] " << endl;
                }
#endif
            }
        }
        tempsort.clear();
    }

    iterations = TempBorders.size();
    niter = 0;
    Vec6i Temp6i;

    for (int j=0;j<iterations;j++)
    {
        for (int i=0;i<TempBorders.size();i++)
        {
            if(i==0)
            {
                Temp6i = TempBorders[i]; niter = i;
            }
            else
            {
                if (DIRECT == VERTICAL)
                {
                    if (TempBorders[i][0]< Temp6i[0]) {Temp6i = TempBorders[i]; niter = i;}
                }
                if (DIRECT == HORISONT)
                {
                    if (TempBorders[i][1]< Temp6i[1]) {Temp6i = TempBorders[i]; niter = i;}
                }
            }
        }
        Borders.push_back(Temp6i);
        TempBorders.erase(TempBorders.begin()+niter);
    }

    return Borders;
}

Vec6i FoundCircles::dsTempBoundaryPoints(const vector<Vec3f> &circles, int DIRECT, bool CoutComments)
{
    Vec6i BoundaryPoints;
    Vec3f xminymin;

    int niter;
    Vec2d LSP;

    LSP = LSPFitLineRadian(circles,DIRECT,CoutComments);

    vector<Vec3f> tempcircles = circles;
    int iterations = tempcircles.size();
    for (int j=0; j<iterations;j++)
    {
        for (int i=0; i<tempcircles.size();i++)
        {
            if (i==0) {xminymin[0] = tempcircles[i][0]; xminymin[1] = tempcircles[i][1];xminymin[2] = tempcircles[i][2]; niter = i;}

            if (DIRECT == HORISONT)
            {
                if ((tempcircles[i][1]*1e2+tempcircles[i][0]*1e6)<xminymin[1]*1e2+xminymin[0]*1e6)
                {
                    xminymin[0]=tempcircles[i][0];
                    xminymin[1]=tempcircles[i][1];
                    xminymin[2]=tempcircles[i][2];
                    niter = i;
                }
            }
            if (DIRECT == VERTICAL)
            {
                if ((tempcircles[i][0]*1e2+tempcircles[i][1]*1e7)<xminymin[0]*1e2+xminymin[1]*1e7)
                {
                    xminymin[0]=tempcircles[i][0];
                    xminymin[1]=tempcircles[i][1];
                    xminymin[2]=tempcircles[i][2];
                    niter = i;
                }
            }

        }

        if (j==0)
        {
            BoundaryPoints[0]  = xminymin[0];
            BoundaryPoints[1]  = xminymin[1];
        }
        if (j==iterations-1)
        {
            BoundaryPoints[2]  = xminymin[0];
            BoundaryPoints[3]  = xminymin[1];
        }
        BoundaryPoints[4] = (int)round(LSP[0]);
        BoundaryPoints[5] = (int)round((1e3*atan(LSP[1])));


        tempcircles.erase(tempcircles.begin()+niter);
    }
#ifdef DEBUG
    if (CoutComments == 1)
    {
        //Point2f Beg,End;
        Point2f Beg((float)(BoundaryPoints[0]),(float)(BoundaryPoints[1]));
        Point2f End((float)(BoundaryPoints[2]),(float)(BoundaryPoints[3]));


        cout << " Angle " << AngleBetLinesDeg(Beg,End) << " Dist " << DistBet2points(Beg,End)
            << " Begin [" << Beg.x << ";" << Beg.y << "] "
            << " End [" << End.x << ";" << End.y << "]" << endl;
    }
#endif
    tempcircles.clear();

    return BoundaryPoints;
}
