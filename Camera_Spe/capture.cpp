

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <python2.7/Python.h>
#include <iostream>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <pthread.h>//added
#include <sched.h>
#include <time.h>


#define NUM_THREADS 2//number of threads competing for the camera resource

using namespace cv;
using namespace std;
pthread_mutex_t mutexcam;
Mat framec;
//************declaration of parameters of face detection function*****************//
volatile double fps;
volatile double t;
void detection( Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale, bool tryflip );
//*********************************************************************************//


//***************************Facial detection Thread*******************************//
void *faceDetection(void *)
{   
    //Open the default camera
    VideoCapture capture(0);
    if(!capture.isOpened())
    {
       //return -1;
    }
    //set frame configuration, it seems the max frame size is 1280x720
    double Vwidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    double Vheight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    //cout << "Video Size: " << Vwidth << " x " << Vheight << endl;

        //capture video frames
	Mat frame_face;
	capture >> frame_face;
	VideoWriter writer("Face Recognition.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(Vwidth, Vheight),1);

	//set Haar classfier
	CascadeClassifier cascade, nestedCascade;
	cascade.load("haarcascade_frontalface_alt.xml");

	//count time
	time_t start,stop;
	start = time(NULL);


	  // while(1)
	  // {
                //if(!capture.read(frame_face))
			//break;
		pthread_mutex_lock(&mutexcam);  
		capture>>frame_face;
		detection( frame_face, cascade, nestedCascade,2,0 );
                /*t=(double)cv::getTickCount();
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		fps = 1.0 / t;
		printf("FPS = %.2f         Execution costs: %.2f \n", fps, t);
		*/
        imshow("Face Recognition",frame_face);
		writer << frame_face;	//write to video
		
		//Manual Exit
		//if(waitKey(1)==27)
			//break;

		//Time limit 2 minutes -> Exit
		stop = time(NULL);
		//if((stop-start) > 100)
			//break;
        capture.release();
		pthread_mutex_unlock(&mutexcam);
	   //}
           //capture.release();
           //cvDestroyWindow("Face Recognition");
}
//*****************************Color Detection Thread******************************//
void *colorDetection(void *)
{   
    Mat frame;
    Mat colorimg;
    VideoCapture capture(0);
	capture >> frame;
	double Vwidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    double Vheight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    double prev_frame_time;
    double curr_frame_time;
    struct timespec frame_time;
    //CvCapture* capture = cvCreateCameraCapture(0);
    int dev=0;
	unsigned prev_locx=0;
	unsigned prev_locy=0;
	unsigned curr_locx;
	unsigned curr_locy;
	unsigned move_h;	
	unsigned move_v;
	int dir=0;	
	int flag=0;
    
		 
		int iLowH=156;
		int iHighH=180;
		
		int iLowS=90;
		int iHighS=255;
		
		int iLowV=90;
		int iHighV=255;
		
		
	//	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	//	cvCreateTrackbar("LowH","Control",&iLowH,179);
	//	cvCreateTrackbar("HighH","Control",&iHighH,179);
		
	//	cvCreateTrackbar("LowS","Control",&iLowS,255);
	//	cvCreateTrackbar("HighS","Control",&iHighS,255);
		
	//	cvCreateTrackbar("LowV","Control",&iLowV,255);
	//	cvCreateTrackbar("HighV","Control",&iHighV,255);
   // namedWindow( timg_window_name, CV_WINDOW_AUTOSIZE );
    // Create a Trackbar for user to enter threshold
    //createTrackbar( "Min Threshold:", timg_window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    //capture = (CvCapture *)cvCreateCameraCapture(dev);
    //CvVideoWriter* video=NULL;
    //cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
    //IplImage *frame = 0;
	double ave_framedt=0.0, ave_frame_rate=0.0, fc=0.0, framedt=0.0;
        unsigned int frame_count=0;
		pthread_mutex_lock(&mutexcam);
        capture>>frame;
        imwrite("color.jpg",frame);
        colorimg=imread("color.jpg");
    
		Mat imgHSV;
		vector<Mat> hsvSplit;
		cvtColor(colorimg,imgHSV,COLOR_BGR2HSV);
		
		split(imgHSV,hsvSplit);
		equalizeHist(hsvSplit[2],hsvSplit[2]);
		merge(hsvSplit,imgHSV);
		Mat imgThresholded;
		inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
		imwrite("color2.jpg",imgThresholded);
		Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
		morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);
		
		morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);
		imwrite("color1.jpg",imgThresholded);
		imshow("Thresholded Image",imgThresholded);
		imshow("original",colorimg);

		waitKey(1);
		int nr=imgThresholded.rows;
		int nc=imgThresholded.cols;
		//printf("ROw is %d, Coloumn is %d\n",nr,nc);
		unsigned isum=0;unsigned jsum=0;unsigned counter=1;
		int total=0;
		//size_t intensity=0;
		for (int i=0;i<nr;i++)
		{  uchar* ptr=(uchar*)imgThresholded.ptr<uchar>(i);
			
			for (int j=0;j<nc;j++)
			{	//printf("value :%d\n",ptr[j]);
				if (ptr[j]==255)
				{	isum+=i;
					jsum+=j;
					counter++;
					}
			}
		}

		curr_locx=isum/counter;
		curr_locy=jsum/counter;
		
		//printf("%d\n",curr_locx);
		//printf("%d\n",curr_locy);
		/*
		if (curr_locy>prev_locy)
			{move_h=curr_locy-prev_locy;
				dir=1;
				printf("%d\n",dir);}
		if (curr_locy<prev_locy)
			{move_h=prev_locy-curr_locy;
				dir=2;
				printf("%d\n",dir);}
		
		if (curr_locx>prev_locx)
			{move_v=curr_locx-prev_locx;
				dir=3;
				printf("%d\n",dir);}
		if (curr_locx<prev_locx)
			{move_v=prev_locx-curr_locx;
				dir=4;
				printf("%d\n",dir);}		
		if (move_h>50 && dir==2 && move_v<150)
			{//printf("it's a left move!\n");
				flag=1;}
			else if (move_h>50 && dir==1 && move_v<150)
			{//printf("it's a right move!\n");
				flag=2;}
			else if (move_v>150 && dir==4 && move_h<50)
			{//printf("it's a left move!\n");
				flag=3;}
			else if (move_v>150 && dir==3 && move_h<50)
			{//printf("it's a right move!\n");
				flag=4;}	
		
		
			if (flag==1)
			{printf("Left option: Today's weather: Sunny!\n");}
			if (flag==2)
			{printf("Right option: Tomorrow's weather: Cloudy!\n");}
			if (flag==3)
			{printf("Up option: Note: Remember to submit final report before 11:55 pm Dect 4th\n");}
			if (flag==4)
			{printf("Down option: The alarm is : 11:30 pm Dect 4th\n");}
			
			
		prev_locx=curr_locx;
		prev_locy=curr_locy;
		*/
		if (curr_locy>540 && curr_locx>100 && curr_locx<380)
			{printf("Left option: I'm hungry\n");}
			else if (curr_locy<100 && curr_locx>100 && curr_locx<380)
			printf("Right option: I'm thirsty\n");
			else if (curr_locx<100 && curr_locy>100 && curr_locy<540)
			printf("Up option: I'm sad\n");
			else if (curr_locx>380 && curr_locy>100 && curr_locy<540)
			printf("Down option: Just check if you are there\n");
  	//printf("Frame @ %u sec, %lu nsec, dt=%5.2lf msec, avedt=%5.2lf msec, rate=%5.2lf fps\n", 
    //              (unsigned)frame_time.tv_sec, 
    //              (unsigned long)frame_time.tv_nsec,
     //             framedt, ave_framedt, ave_frame_rate);
	 //   framedt=curr_frame_time - prev_frame_time;
     //       prev_frame_time=curr_frame_time;

    //	usleep(100000);
		capture.release();
		pthread_mutex_unlock(&mutexcam);
    
     //}  
    //cvReleaseCapture(&capture);
    //cvDestroyWindow("Color Recognition");
}
int main( int argc, char** argv )
{


    //cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);//pop up user interface
    //CvCapture* capture = cvCreateCameraCapture(0);//cv capture from wabcam
    //IplImage* frame;
    system("./webcam");
    system("./alert");
    usleep(2000000);
    pthread_mutex_init(&mutexcam,NULL);
  
        
	pthread_t threads[NUM_THREADS];
	int rc,ret;

//faceDetectionThread
/*pthread_attr_t Canny_sched_attr;
int rt_max_prio, rt_min_prio;
struct sched_param Canny_param;

ret=pthread_attr_init(&Canny_sched_attr);
ret=pthread_attr_setinheritsched(&Canny_sched_attr, PTHREAD_EXPLICIT_SCHED);
ret=pthread_attr_setschedpolicy(&Canny_sched_attr, SCHED_FIFO);

//rt_max_prio = sched_get_priority_max(SCHED_FIFO);
//rt_min_prio = sched_get_priority_min(SCHED_FIFO);
int priority1=30;
Canny_param.sched_priority = priority1;//rt_max_prio;
pthread_attr_setschedparam(&Canny_sched_attr, &Canny_param);*/
		while(1){
       rc = pthread_create(&threads[0], NULL, 
                          faceDetection, NULL);
       if (rc){
         //cout << "Error:unable to create thread," << rc << endl;
         printf("Unable to create thread,%d\n", rc);
         exit(-1);
      }
      if(pthread_join(threads[0], NULL) == 0)
      //printf("Thread 1: %d done\n", threads[0]);
      ;
      else
      perror("Thread 1");

      rc = pthread_create(&threads[1], NULL, 
                          colorDetection, NULL);
       if (rc){
         //cout << "Error:unable to create thread," << rc << endl;
         printf("Unable to create thread,%d\n", rc);
         exit(-1);
      }
      if(pthread_join(threads[1], NULL) == 0)
      //printf("Thread 2: %d done\n", threads[1]);
      ;
      else
      perror("Thread 2");
      //Manual Exit
      if(waitKey(1)==27)
      break;
	
		}	
	pthread_mutex_destroy(&mutexcam);

};




void detection( Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale, bool tryflip )
{	
	int i = 0;

	vector<Rect> faces, faces2;	    //vector to store faces info
	Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );
	
	//Since Haar Classifier is trained with gray scale image, turn img into gray scale
	cvtColor( img, gray, CV_BGR2GRAY );

	//enhance images brightness and contrast
	resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
	equalizeHist( smallImg, smallImg );	

	cascade.detectMultiScale( smallImg, faces, 1.1, 2, 0
				//|CV_HAAR_FIND_BIGGEST_OBJECT
				//|CV_HAAR_DO_ROUGH_SEARCH
				|CV_HAAR_SCALE_IMAGE,
				Size(30, 30));
		
    //detect again to enhance accurancy
    if( tryflip )
    {
        flip(smallImg, smallImg, 1);
        cascade.detectMultiScale( smallImg, faces2, 1.1, 2, 0
                                 //|CV_HAAR_FIND_BIGGEST_OBJECT
                                 //|CV_HAAR_DO_ROUGH_SEARCH
                                 |CV_HAAR_SCALE_IMAGE,
                                 Size(30, 30) );
        for( vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++ )
        {
            faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
        }
    }

 
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++ )
    {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
		Scalar color = CV_RGB(0,0,255);
        int radius;

        double aspect_ratio = (double)r->width/r->height;
        if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
        {
            //resize image
            center.x = cvRound((r->x + r->width*0.5)*scale);
            center.y = cvRound((r->y + r->height*0.5)*scale);
            radius = cvRound((r->width + r->height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );
        }
        else
            rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
                       cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
                       color, 3, 8, 0);
        if( nestedCascade.empty() )
            continue;
        smallImgROI = smallImg(*r);
    }

}

