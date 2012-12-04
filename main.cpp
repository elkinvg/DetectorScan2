#include <iostream>
#include <time.h>
#include "dsvisualmethofopencv.h"
#include "runforinspection.h"

using namespace std;
using namespace cv;

int visscan(int argc, char** argv);
int batscan(int argc, char** argv);
int batscanwithshablon(int argc, char** argv);

int main(int argc, char** argv)
{
    switch (argc)
    {
    case 1:
        cout << " No parameters " << endl;
        return 1;
    case 2:
        visscan(argc,argv);
        break;
    case 11:
        batscan(argc,argv);
        break;
    case 12:
        batscanwithshablon(argc,argv);
        break;
    default:
        cerr << " argc " << argc;
        cerr << " Error Parameters " << endl;
        break;
    }

    return 0;
}

int visscan(int argc, char** argv)
{
    char* filename = argv[1];
    Mat matscan;
    if (cvLoadImage(filename,1) == NULL)
    {
        cout << endl;
        cout << " " << argv[1] << " is not image " << endl;
        cout << endl;
        return 1;
    }
    matscan = imread(filename);

    if(! matscan.data )   // Check for invalid input
        {
            cerr <<  "Could not open or find the image" << std::endl ;
            return -1;
        }

    dsVisualMethOfOpenCV scan(matscan);
    scan.temprun(matscan);
    return 0;
}

int batscan(int argc, char** argv)
{    
    char* filename = argv[1];
    Mat matscan = imread(filename);
    if(! matscan.data )   // Check for invalid input
        {
            cerr <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
//    DetectorScan ds(matscan,atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),atoi(argv[8]),atoi(argv[9]),(double)(atof(argv[10])));
//    ds.run(matscan);
    clock_t time1 = clock();
        runforinspection ds(matscan,atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),atoi(argv[8]),atoi(argv[9]),(double)(atof(argv[10])));
        ds.run(matscan);
        clock_t time2 = clock();
        cout << "\n" << " time: " << (double)(time2-time1)/CLOCKS_PER_SEC << " sek " << endl;
}

int batscanwithshablon(int argc, char** argv)
{
    char* filename = argv[1];
    Mat matscan = imread(filename);
    if(! matscan.data )   // Check for invalid input
        {
            cerr <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
//    DetectorScan ds(matscan,atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),atoi(argv[8]),atoi(argv[9]),(double)(atof(argv[10])));
//    ds.dsShablonContourFile(argv[11]);
//    ds.run(matscan);

       runforinspection  ds(matscan,atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),atoi(argv[6]),atoi(argv[7]),atoi(argv[8]),atoi(argv[9]),(double)(atof(argv[10])));
        ds.dsShablonContourFile(argv[11]);
        ds.run(matscan);
}
