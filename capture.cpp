// Code written by Prayag Desai for RTES exercise 4
// Based on code written by Sam Siewert

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

#define NUMFRAMES 100
#define MATHFRAMES 100

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
Mat gray;
vector<Vec4i> lines;
vector<Vec3f> circles;

IplImage* frame;
time_t startTime, endTime;
CvCapture* capture;

struct execTime{
    struct timespec t1;
    struct timespec t2;
};

vector<double> fps;

static struct execTime exec;
pthread_mutex_t lock; 
pthread_mutex_t lock1; 
bool newVal = false;

long circleSize = 0;

int transfromSel = 0;

//15.45 fps
//0.0647
double cannyDeadline = 0.075;

//9 fps
//0.11
double houghDeadline = 0.2;

//14 fps
//0.0714
double houghEllipDeadline = 0.085;

int deadlineCrossed = 0;

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

void *cannyTransform(void *in)
{
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

        pthread_mutex_lock(&lock); 
        exec.t1 = end;
        exec.t2 = start;
        newVal = true;
        pthread_mutex_unlock(&lock); 
    }
    threadEnd = true;
}

void *houghThread(void *in)
{
    while(1)
    {
        struct timespec start, end;

        clock_gettime(CLOCK_REALTIME, &start);
        frame=cvQueryFrame(capture);

        Mat mat_frame(cvarrToMat(frame));
        Canny(mat_frame, canny_frame, 50, 200, 3);

        cvtColor(canny_frame, cdst, CV_GRAY2BGR);
        cvtColor(mat_frame, gray, CV_BGR2GRAY);

        HoughLinesP(canny_frame, lines, 1, CV_PI/180, 50, 50, 10);

        for( size_t i = 0; i < lines.size(); i++ )
        {
          Vec4i l = lines[i];
          line(mat_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        }

     
        if(!frame) break;

        // cvShowImage seems to be a problem in 3.1
        //cvShowImage("Capture Example", frame);

        imshow("Capture Example", mat_frame);

        char c = cvWaitKey(10);
        if( c == 'q' ) break;
        clock_gettime(CLOCK_REALTIME, &end);

        pthread_mutex_lock(&lock); 
        exec.t1 = end;
        exec.t2 = start;
        newVal = true;
        pthread_mutex_unlock(&lock); 
    }

    threadEnd = true;
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");

}

void *houghEllipThread(void *in)
{
    while(1)
    {
        struct timespec start, end;

        clock_gettime(CLOCK_REALTIME, &start);
        frame=cvQueryFrame(capture);
        if(!frame) break;

        Mat mat_frame(cvarrToMat(frame));
        
        // Does not work in OpenCV 3.1
        //cvtColor(mat_frame, gray, CV_BGR2GRAY);
        cvtColor(mat_frame, gray, COLOR_BGR2GRAY);

        GaussianBlur(gray, gray, Size(9,9), 2, 2);

        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 0, 0);

        // printf("circles.size = %ld\n", circles.size());
        circleSize = circles.size();

        for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( mat_frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( mat_frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        // Does not work in OpenCV 3.1
        //cvShowImage("Capture Example", frame);

        imshow("Capture Example", mat_frame);

        char c = cvWaitKey(10);
        if( c == 'q' ) break;
        clock_gettime(CLOCK_REALTIME, &end);

        pthread_mutex_lock(&lock); 
        exec.t1 = end;
        exec.t2 = start;
        newVal = true;
        pthread_mutex_unlock(&lock); 
    }

    threadEnd = true;
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");
}

void *logThread(void *in)
{
    while(!threadEnd)
    {
        struct timespec tempEnd;
        struct timespec tempStart;
        pthread_mutex_lock(&lock);
        if(newVal)
        {
            newVal = false;
            tempEnd = exec.t1;
            tempStart = exec.t2;
        }
        else
        {
            pthread_mutex_unlock(&lock);
            continue;
        }
        pthread_mutex_unlock(&lock);

        double time_spent = (tempEnd.tv_sec - tempStart.tv_sec) +
						    (tempEnd.tv_nsec - tempStart.tv_nsec) / BILLION;

        double inverse = (double)1.00 / time_spent;

        if(fps.size() < MATHFRAMES)
            fps.push_back(inverse);
        else
        {
            fps.erase(fps.begin());
            fps.push_back(inverse);
        }
    }
}

void *calcThread(void *in)
{
    while(!threadEnd)
    {
        vector<double> jitter;
        printf("\rAverage fps for the last %d frames: ", MATHFRAMES);
        pthread_mutex_lock(&lock1); 
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

            double temp = (double)1.00 / avg;
            double jitFPS = 0;
            
            if(transfromSel == 0)
            {
                jitFPS = (double)1.00 / cannyDeadline;
                if(temp > cannyDeadline)
                {
                    deadlineCrossed++;
                }
            }
            else if(transfromSel == 1)
            {
                jitFPS = (double)1.00 / houghDeadline;
                if(temp > houghDeadline)
                {
                    deadlineCrossed++;
                }
            }
            else if(transfromSel == 2)
            {
                jitFPS = (double)1.00 / houghEllipDeadline;
                if(temp > houghEllipDeadline)
                {
                    deadlineCrossed++;
                }
            }

            double tempJit = abs(jitFPS - avg);

            if(transfromSel != 2)
                printf("\rAverage FPS frames: %0.2lf  Jitter: %0.2lf  Rel Jitter: %0.2lf  Deadline cross: %d", avg, jitterSum, tempJit, deadlineCrossed);
            else
                printf("\rAverage FPS frames: %0.2lf Jitter: %0.2lf circles(): %lu  Rel Jitter: %0.2lf  Deadline cross: %d", avg, jitterSum, circleSize, tempJit, deadlineCrossed);
            
            fflush(stdout);
        }
        pthread_mutex_unlock(&lock1); 
    }
    printf("\n");
}

void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_OTHER\n");
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}
int main( int argc, char** argv )
{
    int xSize = 640;
    int ySize = 480;
    struct sched_param main_param;
    int rc=sched_getparam(getpid(), &main_param);
    main_param.sched_priority=sched_get_priority_max(SCHED_FIFO);
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    print_scheduler();
    int dev=0;
    pthread_t thread, thread1, thread2;
    pthread_attr_t thr, thr1,thr2;
    int thr_pr = sched_get_priority_max(SCHED_FIFO) - 1;
    struct sched_param txP;
    struct sched_param txP1;
    struct sched_param txP2;
    pthread_mutex_init(&lock, NULL);
    pthread_mutex_init(&lock1, NULL);
    pthread_attr_init (&thr);
    pthread_attr_init (&thr1);
    pthread_attr_init (&thr2);
    pthread_attr_getschedparam (&thr, &txP);
    pthread_attr_getschedparam (&thr1, &txP1);
    pthread_attr_getschedparam (&thr2, &txP2);
    txP.sched_priority = thr_pr;
    txP1.sched_priority = thr_pr;
    txP2.sched_priority = thr_pr;
    pthread_attr_setschedparam (&thr, &txP);
    pthread_attr_setschedparam (&thr1, &txP1);
    pthread_attr_setschedparam (&thr2, &txP2);
    pthread_attr_setschedpolicy(&thr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&thr1, SCHED_FIFO);
    pthread_attr_setschedpolicy(&thr1, SCHED_FIFO);   
  if (argc == 1)
    {
        printf("\nUsing default: Canny transform, X Res %d, Y res %d\n", xSize, ySize);
    }
    else if (argc == 2)
    {
        sscanf(argv[1], "%d", &transfromSel);
        printf("\nUsing transformation: ");
        if(transfromSel == 0)
        {
            printf("Canny\n");
        }
        else if(transfromSel == 1)
        {
            printf("Hough\n");
        }
        else if(transfromSel == 2)
        {
            printf("Hough Elliptical\n");
        }
        else
        {
            printf("\nINVALID TRANSFORM SELECTED, CHOOSE 0 - 2\n");
            exit(-1);
        }
    }
    else if (argc == 4)
    {
        sscanf(argv[1], "%d", &transfromSel);
        printf("\nUsing transformation: ");
        if(transfromSel == 0)
        {
            printf("Canny\n");
        }
        else if(transfromSel == 1)
        {
            printf("Hough\n");
        }
        else if(transfromSel == 2)
        {
            printf("Hough Elliptical\n");
        }
        else
        {
            printf("\nINVALID TRANSFORM SELECTED, CHOOSE 0 - 2\n");
            exit(-1);
        }

        sscanf(argv[2], "%d", &xSize);
        sscanf(argv[3], "%d", &ySize);

        printf("Using resolution: X:%d Y:%d\n", xSize, ySize);
    }
    else
    {
        printf("\nInvalid number of arguments, usage:\n");
        printf("[TRANSFORMATION] [X-RES] [Y-RES]\n");
        printf("[TRANSFORMATION]:\n");
        printf("0: Canny\n1: Hough\n2: Hough Elliptical\n");
        exit(-1);
    }

    printf("Deadlines chosen:\nCanny: %0.2lf sec\nHough: %0.2lf sec\nElliptical Hough: %0.2lf sec\n\n", cannyDeadline, \
    houghDeadline, houghEllipDeadline);

    printf("Calculating in real time for %d frames.\n\n", MATHFRAMES);

    namedWindow( timg_window_name, CV_WINDOW_AUTOSIZE );
    // Create a Trackbar for user to enter threshold
    createTrackbar( "Min Threshold:", timg_window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    capture = (CvCapture *)cvCreateCameraCapture(dev);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, xSize);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, ySize);

    printf("\n");

    if(transfromSel == 0)
        pthread_create(&thread, NULL, cannyTransform, NULL);
    else if(transfromSel == 1)
        pthread_create(&thread, NULL, houghThread, NULL);
    else if(transfromSel == 2)
        pthread_create(&thread, NULL, houghEllipThread, NULL);
    else
    {
        printf("Invalid transformation select, exiting\n");
        exit(-1);
    }

    pthread_create(&thread1, NULL, calcThread, NULL);
    pthread_create(&thread2, NULL, logThread, NULL);

    pthread_join(thread, NULL);
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);

    cvReleaseCapture(&capture);
    
}
