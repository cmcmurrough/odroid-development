//
//    Copyright 2013 Christopher D. McMurrough
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

/***********************************************************************************************************************
FILENAME:   cv_usb_example.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
Code example for using USB camera with OpenCV

REVISION HISTORY:
11.12.2012   CDM     original file creation
09.01.2013   CDM     published under GPL
***********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <opencv/highgui.h>

// camera parameters
#define CAMERA_FRAME_WIDTH 640
#define CAMERA_FRAME_HEIGHT 480

/***********************************************************************************************************************
CvCapture* initializeCamera(int cameraIndex)
initialize the camera capture, cameraIndex is the USB camera number
***********************************************************************************************************************/
CvCapture* initializeCamera(int cameraIndex)
{
    CvCapture* capture = cvCaptureFromCAM(cameraIndex);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT);

    if (!capture)
    {
        printf("ERROR: unable to initialize camera %d \n", cameraIndex);
        return NULL;
    }

    printf("Camera %d opened successfully \n", cameraIndex);
    return capture;
}

/***********************************************************************************************************************
void closeCamera(CvCapture* capture)
close a camera capture
***********************************************************************************************************************/
void closeCamera(CvCapture* capture)
{
    cvReleaseCapture(&capture);
}

/***********************************************************************************************************************
bool getImageFrame(CvCapture* pupilCapture, IplImage*& rawImage)
get the pupil image from the capture object
***********************************************************************************************************************/
bool getImageFrame(CvCapture* camera, IplImage*& rawImage)
{
    // get the raw image
    IplImage* frame = cvQueryFrame(camera);

    // return false if no image was acquired
    if(!frame)
    {
        return false;
    }

    rawImage = frame;

    return true;
}

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, expects the camera index as command line arguments
***********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store video capture parameters
    int cameraIndex = 0;

    // define image pointers
    IplImage *img = NULL;

    // initialize the camera
    CvCapture* capture = initializeCamera(cameraIndex);
    if(!capture)
    {
        printf("Unable to initialize camera %u! \n", cameraIndex);
        return 0;
    }

    // create the image window
    cvNamedWindow("image");

    // process frames until program termination
    while(true)
    {
        // attempt to acquire an image frame
        if(getImageFrame(capture, img))
        {
            cvShowImage("image", img);
            cvWaitKey(1);
        }
        else
        {
            printf("Unable to get image! \n");
        }
    }

    // release program resources before returning
    closeCamera(capture);
    cvDestroyWindow("image");
}

