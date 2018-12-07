/* This is a standalone program. Pass an image name as the first parameter
of the program.  Switch between standard and probabilistic Hough transform
by changing "#if 1" to "#if 0" and back */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <iostream>


//file io
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>

#define PIXELS_PER_METER 3779.5275593333;

using namespace cv;
using namespace std;

static string fileName;


void writeToFile(vector<Vec4i> points, string cmdCode){
  double valx, valy;
  bool contiguous = false;
  std::string fileDir = fileName + ".cdcode";
    ofstream filestr(fileDir.c_str());
    for(int r = 0; r < points.size(); r++){
	if(!contiguous)
		filestr << cmdCode;
      for(int g = 0; g < 4; g+=2){
         valx = points[r][g] / PIXELS_PER_METER;
         valy = points[r][g+1] / PIXELS_PER_METER;
        filestr << " X:" << valx << " Y:" << valy;
      }
      filestr << endl;
   }
   filestr.close();
}
//stuffed animals made from kid's drawings 
int main(int argc, char** argv)
{
    Mat src, dst, color_dst, thresh_dst;
    if( argc < 2 || !(src=imread(argv[1], 0)).data)
        return -1;
    if(argc = 3)
		fileName = argv[2];
	else
		fileName = "sampleOutput";

    Canny( src, dst, 200, 300, 5, true);
    cvtColor( dst, color_dst, CV_GRAY2BGR );
    //threshold( src, thresh_dst, 0, 255, 1 );
    //bitwise_not ( dst, dst );

#if 0
    /*vector<Vec2f> lines;
    HoughLines( dst, lines, 1, CV_PI/180, 100 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
    }*/
#else
    vector<Vec4i> lines;
    vector<Vec4i> meterLines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 5, 60, 40 );
    for(size_t i = 0; i < lines.size(); i++ )
    {
    	std::cout << "x1: " << lines[i][0] << " y1: " << lines[i][1] << " x2: " << lines[i][2] << " y2: " << lines[i][3] << std::endl;
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
    }
    writeToFile(lines, "P2P");
    std::cout << "Size of lines vector: " << lines.size() << std::endl;
#endif
    //namedWindow( "Source", 1 );
    //imshow( "Source", src );
    namedWindow( "Detected Lines", 1 );
    //imshow( "Detected Lines", color_dst );
    //threshold( dst, color_dst, 0, 255,3 );
    imshow( "Detected Lines", color_dst );

    waitKey(0);
    return 0;
}
