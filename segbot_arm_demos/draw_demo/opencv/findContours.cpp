#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 


//file io
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>

using namespace cv;
using namespace std;

#define PIXELS_PER_METER 3779.5275593333

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

std::string fileName;
/// Function header
void thresh_callback(int, void* );

/** @function main */
//cmd arguments: include filename without extension.
int main( int argc, char** argv )
{
	    Mat src, dst, color_dst, thresh_dst, in, scaled;
    if( argc == 1 || !(src=imread(argv[1], 0)).data)
        return -1;
    else if(argc >= 3)
		fileName = argv[2];
	else
		fileName = "sampleOutput";
	cout << endl << "Please note that the input image is scaled to a square." << endl;
	cout << "In order to preserve prespective, use a 1:1 aspect ratio." << endl;
  /// Load source image and convert it to gray
	in = imread( argv[1], 1 );
	if(argc > 3){
		Size size(375,375);//the dst image size,e.g.100x100
		resize(in,src,size);//resize image
	}
	else
		src = in;

  	flip(src, src, 1);

  //Size size(400,400);
  //Mat dst

  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  //char* source_window = "Source";
  //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  //imshow( source_window, src );

  thresh_callback( 0, 0 );

  waitKey(0);
  return(0);
}

void writeToFile(  vector<vector<Point> > points, string cmdCode){
  double valx, valy;
  double delta_x, delta_y, distance;
  int lastIndexDrawn = 0;
  bool printedCMD = false; //used to track if the CMD code has been printed - only gets printed if there are contour points that haven't been filtered out
  std::string fileDir = "drawing_files/" + fileName + ".cdcode";
    ofstream filestr(fileDir.c_str());
     for(int r = 0; r < points.size(); r++){
      for(int g = 0; g < points[r].size(); g++){
		  if(g != 0 || g < points[r].size() - 1){
			  	delta_x = points[r][g].x - points[r][g - lastIndexDrawn].x;
				delta_y = points[r][g].y - points[r][g - lastIndexDrawn].y;
				distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));
				if(distance >= (.008 * PIXELS_PER_METER)){
          if(!printedCMD){
            filestr << cmdCode;
            printedCMD = true;
          }
					lastIndexDrawn = 1;
					valx = points[r][g].x / PIXELS_PER_METER;
					valy = points[r][g].y / PIXELS_PER_METER;
					filestr << " X:" << valx << " Y:" << valy;
				}
				else
					lastIndexDrawn++;
		  }
		  else{
        //cout << "r size: " << points.size();
        //cout << " g size: " << points[r].size();
         valx = points[r][g].x / PIXELS_PER_METER;
         valy = points[r][g].y / PIXELS_PER_METER;
         filestr << " X:" << valx << " Y:" << valy;
		 }
      }
      if(printedCMD) //only produce CRLF if a CMDcode was printed 
        filestr << endl;
      printedCMD = false;
    }
   filestr.close();
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<vector<Point> > meterContours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, 100, 200, 3, true );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL,  CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0) );
  meterContours = contours;
  for(int r = 0; r < contours.size(); r++){
    //cout << "Next contour point: ";
    for(int g = 0; g < contours[r].size(); g++){
      double valx = contours[r][g].x / PIXELS_PER_METER;
      double valy = contours[r][g].y / PIXELS_PER_METER;
      //out << "pt: " << valx << " " << valy;
    }
    //cout << endl;
  }
  writeToFile(meterContours, "P2P");
  cout << endl << "Total size of contors: " << contours.size() << endl;
  cout << endl << "File created successfully!" << endl;
  cout << "To exit, press any key in the popup window." << endl;

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}
