/*
 *
 *  Example by Sam Siewert 
 *
 *  Updated 10/29/16 for OpenCV 3.1
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <stdbool.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NUMFRAMES 500
#define MATHFRAMES 500

using namespace cv;
using namespace std;

#define HRES 640
#define VRES 480

#define BILLION  1000000000.0

bool threadEnd = false;


// Transform display window
char timg_window_name[] = "Edge Detector Transform";

int lowThreshold=0;
int const max_lowThreshold = 100;
int kernel_size = 3;
int edgeThresh = 1;
int ratio = 3;
Mat canny_frame, cdst, timg_gray, timg_grad;

IplImage* frame;
time_t startTime, endTime;
CvCapture* capture;

vector<double> fps;


void CannyThreshold(int, void*)
{
    //Mat mat_frame(frame);
    Mat mat_frame(cvarrToMat(frame));

    cvtColor(mat_frame, timg_gray, CV_RGB2GRAY);

    /// Reduce noise with a kernel 3x3
    blur( timg_gray, canny_frame, Size(3,3) );

    /// Canny detector
    Canny( canny_frame, canny_frame, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    timg_grad = Scalar::all(0);

    mat_frame.copyTo( timg_grad, canny_frame);

    imshow( timg_window_name, timg_grad );

}

void *cannyTransform(void* in)
{
    time(&startTime);
    // for(int i = 0; i < NUMFRAMES; i++)
    while(1)
    {
        struct timespec start, end;

        clock_gettime(CLOCK_REALTIME, &start);
        frame=cvQueryFrame(capture);
        if(!frame) break;

        CannyThreshold(0, 0);
        char q = cvWaitKey(33);
        if( q == 'q' )
        {
            printf("got quit\n");
            threadEnd = true;
            break;
        }
        clock_gettime(CLOCK_REALTIME, &end);
        double time_spent = (end.tv_sec - start.tv_sec) +
						(end.tv_nsec - start.tv_nsec) / BILLION;

        double inverse = (double)1.00 / time_spent;
        // printf("From func: %lf\n", inverse);

        if(fps.size() < MATHFRAMES)
            fps.push_back(inverse);
        else
        {
            fps.erase(fps.begin());
            fps.push_back(inverse);
        }
    }
    threadEnd = true;
    time(&endTime);
    // break;
}

void *calcThread(void* in)
{
    while(!threadEnd)
    {
        vector<double> jitter;
        printf("\rAverage fps for the last %d frames: ", MATHFRAMES);
        if(fps.size() < MATHFRAMES)
        {
            printf("Waiting to reach %d frames", MATHFRAMES);
            fflush(stdout);
        }
        else
        {
            double avg = 0;
            for(int i = 0; i < MATHFRAMES; i++)
            {
                avg += fps[i];

                if(i != MATHFRAMES - 1)
                {
                    // jitter[i] = abs(fps[i] - fps[i + 1]);
                    jitter.push_back(abs(fps[i] - fps[i + 1]));
                }
            }
            double jitterSum = 0;
            for(int i = 0; i < MATHFRAMES - 1; i++)
            {
                jitterSum += jitter[i];
                // printf("Jitter: %lf\n", jitter[i]);
            }
            avg /= fps.size();
            // printf("Jitter: %lf\n", jitterSum);
            jitterSum /= jitter.size();
            printf("\rAverage FPS for %d frames: %lf         Jitter: %lf                  ", MATHFRAMES, avg, jitterSum);
            fflush(stdout);
        }
    }
    printf("\n");
}


int main( int argc, char** argv )
{
    int dev=0;
    pthread_t thread, thread1;

    if(argc > 1)
    {
        sscanf(argv[1], "%d", &dev);
        printf("using %s\n", argv[1]);
    }
    else if(argc == 1)
        printf("using default\n");

    else
    {
        printf("usage: capture [dev]\n");
        exit(-1);
    }

    namedWindow( timg_window_name, CV_WINDOW_AUTOSIZE );
    // Create a Trackbar for user to enter threshold
    createTrackbar( "Min Threshold:", timg_window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    capture = (CvCapture *)cvCreateCameraCapture(dev);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, HRES);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, VRES);

    // while(1)
    // {
    // cannyTransform();
    printf("\n");
    pthread_create(&thread, NULL, cannyTransform, NULL);
    pthread_create(&thread1, NULL, calcThread, NULL);
    pthread_join(thread, NULL);
    pthread_join(thread1, NULL);
    // }

    // Time elapsed
    // double seconds = difftime (endTime, startTime);
    // cout << "Time taken : " << seconds << " seconds" << endl;
    
    // Calculate frames per second
    // double fps;
    // fps  = NUMFRAMES / seconds;
    // cout << "Estimated frames per second : " << fps << endl;

    cvReleaseCapture(&capture);
    
}
