#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <time.h>
#include <stdio.h>

struct timespec rectStart = {0, 0};
struct timespec rectEnd = {0, 0};
struct timespec rectExec = {0, 0};
struct timespec rectWorse = {0, 0};

struct timespec cirStart = {0, 0};
struct timespec cirEnd = {0, 0};
struct timespec cirExec = {0, 0};
struct timespec cirWorse = {0, 0};

struct timespec frameStart = {0, 0};
struct timespec frameEnd = {0, 0};
struct timespec frameExec = {0, 0};
struct timespec frameWorse = {0, 0};

using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 170;
RNG rng(12345);
void thresh_callback(int, void * );

VideoCapture cap;
#define HRES 640
#define VRES 480

int main(int argc, char ** argv) {
  CvCapture * capture;
  IplImage * frame;
  capture = (CvCapture * ) cvCreateCameraCapture(0);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);
  //if(!cap.open(0))
  //    return 0;
  // CommandLineParser parser( argc, argv, "{@input | stuff.jpg | input image}" );
  // Mat src = imread( samples::findFile( parser.get<String>( "@input" ) ) );
  // Mat src;
  while (1) {
    // cap >> src;
    frame = cvQueryFrame(capture);
    Mat src(cvarrToMat(frame));

    if (src.empty()) {
      cout << "Could not open or find the image!\n" << endl;
      cout << "usage: " << argv[0] << " <Input image>" << endl;
      return -1;
    }
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));
    const char * source_window = "Source";
    namedWindow(source_window);

    clock_gettime(CLOCK_REALTIME, &rectStart);
    rectangle(src, Point2f(50, 50), Point2f(300, 300), Scalar(0, 0, 0));
    clock_gettime(CLOCK_REALTIME, &rectEnd);

    rectExec.tv_sec = rectEnd.tv_sec - rectStart.tv_sec;
    rectExec.tv_nsec = rectEnd.tv_nsec - rectStart.tv_nsec;

    // printf("RECTANGLE: Worse case: %ld sec %ld msec\n", rectWorse.tv_sec, rectWorse.tv_nsec/1000000);
    
    if(rectExec.tv_sec > rectWorse.tv_sec || rectExec.tv_nsec > rectWorse.tv_nsec)
    {
        rectWorse = rectExec;    
        printf("RECTANGLE: Worse case: %ld sec %ld usec\n", rectWorse.tv_sec, rectWorse.tv_nsec/1000);
    }

    clock_gettime(CLOCK_REALTIME, &cirStart);
    circle(src, Point(200, 200), 50, Scalar(255, 255, 255), 0, 8, 0);
    clock_gettime(CLOCK_REALTIME, &cirEnd);

    cirExec.tv_sec = cirEnd.tv_sec - cirStart.tv_sec;
    cirExec.tv_nsec = cirEnd.tv_nsec - cirStart.tv_nsec;
    
    if(cirExec.tv_sec > cirWorse.tv_sec || cirExec.tv_nsec > cirWorse.tv_nsec)
    {
        cirWorse = cirExec;
        printf("CIRCLE: Worse case: %ld sec %ld usec\n", cirWorse.tv_sec, cirWorse.tv_nsec/1000);
    }

    imshow(source_window, src);
    const int max_thresh = 255;
    createTrackbar("Canny thresh:", source_window, & thresh, max_thresh, thresh_callback);
    thresh_callback(0, 0);
    waitKey(10);
  }
  return 0;
}


void thresh_callback(int, void * ) {

    clock_gettime(CLOCK_REALTIME, &frameStart);
  Mat canny_output;
  Canny(src_gray, canny_output, thresh, thresh * 2);
  vector <vector<Point>> contours;
  findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
  vector <vector<Point>> contours_poly(contours.size());
  vector <Rect> boundRect(contours.size());
  vector <Point2f> centers(contours.size());
  vector <float> radius(contours.size());

  for (size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(contours[i], contours_poly[i], 3, true);
    boundRect[i] = boundingRect(contours_poly[i]);
    minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
  }
  Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, contours_poly, (int) i, color);
    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
    circle(drawing, centers[i], (int) radius[i], color, 2);
  }
  imshow("Contours", drawing);

  clock_gettime(CLOCK_REALTIME, &frameEnd);

    frameExec.tv_sec = frameEnd.tv_sec - frameStart.tv_sec;
    frameExec.tv_nsec = frameEnd.tv_nsec - frameStart.tv_nsec;
    
    if(frameExec.tv_sec > frameWorse.tv_sec || frameExec.tv_nsec > frameWorse.tv_nsec)
    {
        frameWorse = frameExec;
        printf("FRAME: Worse case: %ld sec %ld usec\n", frameWorse.tv_sec, frameWorse.tv_nsec/1000);
    }
}