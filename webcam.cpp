    #include <opencv2/core/core.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include <iostream>
    #include <stdio.h>
    #include <stdlib.h>

    int main(int argc,char **argv)
    {
    /* init camera */
      CvCapture* pCapture = cvCreateCameraCapture(0);
      cvSetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH, 800);
      cvSetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT, 480);

      IplImage *pFrame = 0;

      if(NULL == pCapture)
      {
        fprintf(stderr,"Can't initialize webcam!\n");
        return 1;
      }

      pFrame = cvQueryFrame(pCapture); // query a frame

      if(NULL == pFrame)
      {
        fprintf(stderr,"Can't get a frame!\n");
        return 1;
      }

      const char *pImageFileName ="webcam.jpg";
      cvSaveImage(pImageFileName, pFrame);

      cvReleaseCapture(&pCapture); // free memory

      return 0;
    }

