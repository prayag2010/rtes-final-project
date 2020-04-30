#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

#include <arpa/inet.h>
#include <syslog.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include "thread_details.h"
#include <time.h>
#include <string.h>
#include <semaphore.h>
#include <stdbool.h>
#include <syslog.h>
#include <sys/time.h>
#include <errno.h>
#define PORTNO 9000
#define DISTANCECHECK 8100
using namespace cv;

using namespace std;
pthread_mutex_t image_access;
Mat frame, blurFrame;
int score;
//--- INITIALIZE VIDEOCAPTURE
VideoCapture cap;
Mat fullImageHSV;
Mat frame_threshold;
int ran_x = 320;
int ran_y = 240;

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
int socket_client;
vector<Point> contours_poly;
int largest_area = 0;
int largest_contour_index = 0;
Rect bounding_rect, boundRect;
float radius;
Point2f center;
Mat drawing;

RNG rng(12345);
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

sem_t sem[MAX_THREADS];
bool completed[MAX_THREADS];
int sequencer_on = 1;
void *Sequencer(void *args);
void *imageacq(void *args);

void *generatebound(void *args);
void *checker(void *args);
void *fetchinput(void *args);

void print_scheduler(void) {
  int schedType;

  schedType = sched_getscheduler(getpid());

  switch (schedType) {
  case SCHED_FIFO:
    printf("Pthread Policy is SCHED_FIFO\n");
    break;
  case SCHED_OTHER:
    printf("Pthread Policy is SCHED_OTHER\n");
    exit(-1);
    break;
  case SCHED_RR:
    printf("Pthread Policy is SCHED_RR\n");
    exit(-1);
    break;
  default:
    printf("Pthread Policy is UNKNOWN\n");
    exit(-1);
  }
}

int getMaxAreaContourId(vector<vector<cv::Point>> contours) {
  double maxArea = 0;
  int maxAreaContourId = -1;
  for (int j = 0; j < contours.size(); j++) {
    double newArea = cv::contourArea(contours.at(j));
    if (newArea > maxArea) {
      maxArea = newArea;
      maxAreaContourId = j;
    } // End if
  }   // End for
  return maxAreaContourId;
} // End function

void acquireFrame(void) {
  // wait for a new frame from camera and store it into 'frame'
  cap.read(frame);
  // check if we succeeded
  if (frame.empty()) {
    cerr << "ERROR! blank frame grabbed\n";
  }
  // show live and wait for a key with timeout long enough to show images
  waitKey(5);
}

void applyBlur(void) { GaussianBlur(frame, blurFrame, Size(11, 11), 0, 0); }

void applyMask(void) {
  cvtColor(blurFrame, fullImageHSV, CV_BGR2HSV);
  inRange(fullImageHSV, Scalar(36, 25, 25), Scalar(70, 255, 255),
          frame_threshold);
  erode(frame_threshold, frame_threshold, 0);
  erode(frame_threshold, frame_threshold, 0);
  dilate(frame_threshold, frame_threshold, 0);
  dilate(frame_threshold, frame_threshold, 0);
}

void acqLargestContours(void) {
  // was  CV_RETR_TREE

  findContours(frame_threshold, contours, hierarchy, RETR_EXTERNAL,
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  largest_area = 0;
  largest_contour_index = 0;

  for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
  {
    double area = contourArea(contours[i]); //  Find the area of contour

    if (area > largest_area) {
      largest_area = area;
      largest_contour_index = i; // Store the index of largest contour
      bounding_rect = boundingRect(
          contours[i]); // Find the bounding rectangle for biggest contour
      approxPolyDP(contours[largest_contour_index], contours_poly, 3, true);
      boundRect = boundingRect(contours_poly);
      minEnclosingCircle(contours_poly, center, radius);
    }
  }
}

void drawOverlay(void) {
  // drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
  // Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256),
  // rng.uniform(0,256) );
  // rectangle( frame, boundRect.tl(), boundRect.br(), Scalar( 0, 255, 0 ), 2 );
  circle(frame, center, (int)radius, Scalar(0, 255, 0), 2);
  // drawContours( drawing, contours,largest_contour_index, Scalar( 0, 255, 0 ),
  // 2 ); // Draw the largest contour using previously stored index.
}

int main(int argc, char *argv[]) {
  srand(time(NULL));
  // open the default camera using default API
  cap.open(0);
  // OR advance usage: select any API backend
  int deviceID = 0;        // 0 = open default camera
  int apiID = cv::CAP_ANY; // 0 = autodetect default API
  // open selected camera using selected API
  socket_client = socket(PF_INET, SOCK_STREAM, 0);
  if (socket_client < 0)
    perror("Socket setup");

  // Socket parameters
  struct sockaddr_in server_details;
  socklen_t addr_size;
  socket_client = socket(AF_INET, SOCK_STREAM, 0);
  server_details.sin_family = AF_INET;
  server_details.sin_port = htons(PORTNO);
  server_details.sin_addr.s_addr = inet_addr(argv[1]);
  memset(server_details.sin_zero, '0', sizeof(server_details.sin_zero));
  addr_size = sizeof(server_details);

  if (connect(socket_client, (struct sockaddr *)&server_details, addr_size) ==
      -1) {
    printf("Connect to server failed\n");
  }
  cap.open(deviceID + apiID);
  // check if we succeeded
  if (!cap.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }

  if (pthread_mutex_init(&image_access, NULL) != 0) {
    printf("\n mutex init failed\n");
    return 1;
  }

  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  cpu_set_t threadcpu;
  int i, rc, scope;
  pthread_t threads[MAX_THREADS];
  pthread_attr_t rt_sched_attr[MAX_THREADS];
  int rt_max_prio, rt_min_prio;
  struct sched_param rt_param[MAX_THREADS];
  struct sched_param main_param;
  pthread_attr_t main_attr;
  pid_t mainpid;
  cpu_set_t allcpuset;

  for (int x = 0; x < MAX_THREADS; x++)
    completed[x] = 1;

  CPU_ZERO(&allcpuset);

  for (int i = 0; i < 1; i++)
    CPU_SET(i, &allcpuset);

  if (sem_init(&sem[sequencer], 0, 0)) {
    printf("Failed to initialize S1 semaphore\n");
    exit(-1);
  }
  if (sem_init(&sem[IMAGE_ACQ], 0, 0)) {
    printf("Failed to initialize S2 semaphore\n");
    exit(-1);
  }
  if (sem_init(&sem[IMAGE_DRAW], 0, 0)) {
    printf("Failed to initialize S3 semaphore\n");
    exit(-1);
  }
  if (sem_init(&sem[GENERATE_RECTANGLE], 0, 0)) {
    printf("Failed to initialize S4 semaphore\n");
    exit(-1);
  }
  if (sem_init(&sem[LOCATION_CHECKER], 0, 0)) {
    printf("Failed to initialize S5 semaphore\n");
    exit(-1);
  }
  if (sem_init(&sem[USER_INPUT], 0, 0)) {
    printf("Failed to initialize S6 semaphore\n");
    exit(-1);
  }

  mainpid = getpid();

  rt_max_prio = sched_get_priority_max(SCHED_FIFO);
  rt_min_prio = sched_get_priority_min(SCHED_FIFO);

  rc = sched_getparam(mainpid, &main_param);
  main_param.sched_priority = rt_max_prio;
  rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
  if (rc < 0)
    perror("main_param");
  print_scheduler();

  pthread_attr_getscope(&main_attr, &scope);

  if (scope == PTHREAD_SCOPE_SYSTEM)
    printf("PTHREAD SCOPE SYSTEM\n");
  else if (scope == PTHREAD_SCOPE_PROCESS)
    printf("PTHREAD SCOPE PROCESS\n");
  else
    printf("PTHREAD SCOPE UNKNOWN\n");

  printf("rt_max_prio=%d\n", rt_max_prio);
  printf("rt_min_prio=%d\n", rt_min_prio);

  for (i = 0; i < MAX_THREADS; i++) {

    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);

    rc = pthread_attr_init(&rt_sched_attr[i]);
    rc =
        pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
    rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);

    rt_param[i].sched_priority = rt_max_prio - i;
    pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
  }

  // Image tracker
  rt_param[IMAGE_ACQ].sched_priority = rt_max_prio - 1;
  pthread_attr_setschedparam(&rt_sched_attr[IMAGE_ACQ], &rt_param[IMAGE_ACQ]);
  rc = pthread_create(&threads[IMAGE_ACQ],       // pointer to thread descriptor
                      &rt_sched_attr[IMAGE_ACQ], // use specific attributes
                      //(void *)0,               // default attributes
                      imageacq, // thread function entry point
                      NULL      // parameters to pass in
                      );

  if (rc < 0)
    perror("pthread_create for service 1");
  else
    printf("pthread_create successful for service 1\n");

  rt_param[LOCATION_CHECKER].sched_priority = rt_max_prio - 2;
  pthread_attr_setschedparam(&rt_sched_attr[LOCATION_CHECKER],
                             &rt_param[LOCATION_CHECKER]);
  rc = pthread_create(&threads[LOCATION_CHECKER],
                      &rt_sched_attr[LOCATION_CHECKER], checker, NULL);
  if (rc < 0)
    perror("pthread_create for service 3");
  else
    printf("pthread_create successful for service 3\n");

  rt_param[USER_INPUT].sched_priority = rt_max_prio - 3;
  pthread_attr_setschedparam(&rt_sched_attr[USER_INPUT], &rt_param[USER_INPUT]);
  rc = pthread_create(&threads[USER_INPUT], &rt_sched_attr[USER_INPUT],
                      fetchinput, NULL);
  if (rc < 0)
    perror("pthread_create for service 4");
  else
    printf("pthread_create successful for service 4\n");

  rt_param[GENERATE_RECTANGLE].sched_priority = rt_max_prio - 4;
  pthread_attr_setschedparam(&rt_sched_attr[GENERATE_RECTANGLE],
                             &rt_param[GENERATE_RECTANGLE]);
  rc = pthread_create(&threads[GENERATE_RECTANGLE],
                      &rt_sched_attr[GENERATE_RECTANGLE], generatebound, NULL);

  if (rc < 0)
    perror("pthread_create for service 5");
  else
    printf("pthread_create successful for service 5\n");

  rt_param[sequencer].sched_priority = rt_max_prio;
  pthread_attr_setschedparam(&rt_sched_attr[sequencer], &rt_param[sequencer]);
  rc = pthread_create(&threads[sequencer], &rt_sched_attr[sequencer], Sequencer,
                      NULL);
  if (rc < 0)
    perror("pthread_create for sequencer service 0");
  else
    printf("pthread_create successful for sequeencer service 0\n");

  while (1) {
    ;
  }

  return 0;
}

void *Sequencer(void *args) {
  struct timeval current_time_val;
  struct timespec delay_time = {0, 1000000}; // delay for 1 msec,
  struct timespec remaining_time;
  double current_time;
  double residual;
  int rc, delay_cnt = 0;
  unsigned long long seqCnt = 0;

  gettimeofday(&current_time_val, (struct timezone *)0);

  do {
    delay_cnt = 0;
    residual = 0.0;

    do {
      rc = nanosleep(&delay_time, &remaining_time);

      if (rc == EINTR) {
        residual = remaining_time.tv_sec +
                   ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

        if (residual > 0.0)
          printf("residual=%lf, sec=%d, nsec=%d\n", residual,
                 (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);

        delay_cnt++;
      }

    } while ((residual > 0.0) && (delay_cnt < 100));

    seqCnt++;
    gettimeofday(&current_time_val, (struct timezone *)0);

    if (delay_cnt > 1)
      printf("Sequencer looping delay %d\n", delay_cnt);

    // Release each service at a sub-rate of the generic sequencer rate

    if ((seqCnt % 40) == 0)
      sem_post(&sem[IMAGE_ACQ]);

    if ((seqCnt % 100) == 0)
      sem_post(&sem[LOCATION_CHECKER]);

    if ((seqCnt % 3000) == 0)
      sem_post(&sem[USER_INPUT]);

    if ((seqCnt % 10000) == 0)
      sem_post(&sem[GENERATE_RECTANGLE]);

    if (seqCnt > 60000)
      break;
  } while (completed[sequencer]);

  completed[sequencer] = 0;
  sem_post(&sem[IMAGE_ACQ]);
  sem_post(&sem[IMAGE_DRAW]);
  sem_post(&sem[LOCATION_CHECKER]);
  sem_post(&sem[USER_INPUT]);
  sem_post(&sem[GENERATE_RECTANGLE]);

  pthread_exit((void *)0);
}

void *imageacq(void *args) {
  int times = 0;
  struct timeval start, end, difference;
  static int wcet;
  while (completed[sequencer] == 1) {
    sem_wait(&sem[IMAGE_ACQ]);
    gettimeofday(&start, (struct timezone *)0);
    acquireFrame();
    applyBlur();
    applyMask();
    acqLargestContours();
    completed[IMAGE_ACQ] = 1;
    drawOverlay();
    circle(frame, Point(ran_x, ran_y), 50, Scalar(255, 255, 255), 0, 8, 0);
    char scorestr[50];
    sprintf(scorestr, "Current score is %d", score);
    putText(frame, scorestr, Point(10, 30), FONT_HERSHEY_TRIPLEX, 1,
            Scalar(200, 100, 34), 1);
    imshow("Live", frame);
    gettimeofday(&end, (struct timezone *)0);
    // wcet_measure[MAX_THREADS-1];
    difference.tv_sec = end.tv_sec - start.tv_sec;
    difference.tv_usec = end.tv_usec - start.tv_usec;
    int timediff = 1000 * (difference.tv_sec) + (difference.tv_usec / 1000);
    if (timediff > wcet) {
      wcet = timediff;
    }
  }
  printf("WCET IMAGE ACQ:%d\n", wcet);
}

void *generatebound(void *args) {
  int times = 0;
  struct timeval start, end, difference;
  static int wcet;
  while (completed[sequencer] == 1) {
    sem_wait(&sem[GENERATE_RECTANGLE]);
    gettimeofday(&start, (struct timezone *)0);
    times++;
    ran_x = rand() % 400 + 50;
    ran_y = rand() % 250 + 20;
    gettimeofday(&end, (struct timezone *)0);
    // wcet_measure[MAX_THREADS-1];
    difference.tv_sec = end.tv_sec - start.tv_sec;
    difference.tv_usec = end.tv_usec - start.tv_usec;
    int timediff = 1000 * (difference.tv_sec) + (difference.tv_usec / 1000);
    if (timediff > wcet) {
      wcet = timediff;
    }
  }
  printf("Boundary generation WCET:%d\n", wcet);
}

void *checker(void *args) {
  int times;
  struct timeval start, end, difference;
  static int wcet;
  while (completed[sequencer] == 1) {
    sem_wait(&sem[LOCATION_CHECKER]);
    gettimeofday(&start, (struct timezone *)0);
    times++;
    int distance =
        abs(pow(ran_x - (int)center.x, 2)) + abs(pow(ran_y - (int)center.y, 2));

    if (distance < DISTANCECHECK) {
      score++;
    }
    gettimeofday(&end, (struct timezone *)0);
    // wcet_measure[MAX_THREADS-1];
    difference.tv_sec = end.tv_sec - start.tv_sec;
    difference.tv_usec = end.tv_usec - start.tv_usec;
    int timediff = 1000 * (difference.tv_sec) + (difference.tv_usec / 1000);
    if (timediff > wcet) {
      wcet = timediff;
    }
  }
  printf("Checker:%d\n", wcet);
}

void *fetchinput(void *args) {
  struct timeval start, end, difference;
  static int wcet;
  while (completed[sequencer] == 1) {
    sem_wait(&sem[USER_INPUT]);
    gettimeofday(&start, (struct timezone *)0);
    char ch[2] = {0};
    printf("Enter cmd\n");
    cin >> ch;
    write(socket_client, &ch, 1);
    gettimeofday(&end, (struct timezone *)0);
    // wcet_measure[MAX_THREADS-1];
    difference.tv_sec = end.tv_sec - start.tv_sec;
    difference.tv_usec = end.tv_usec - start.tv_usec;
    int timediff = 1000 * (difference.tv_sec) + (difference.tv_usec / 1000);
    if (timediff > wcet) {
      wcet = timediff;
    }
  }
  printf("WCET USER IP :%d\n", wcet);
}
