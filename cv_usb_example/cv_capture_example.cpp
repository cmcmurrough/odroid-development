//
//    Copyright 2014 Christopher D. McMurrough
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

/*******************************************************************************************************************//**
 * @FILE cv_capture_example.cpp
 * @BRIEF C++ example for acquiring and processing image frames with OpenCV
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/

// include necessary dependencies
#include <iostream>
#include <stdio.h>
#include <opencv/highgui.h>

// configuration parameters
#define NUM_COMNMAND_LINE_ARGUMENTS 2
#define DISPLAY_WINDOW_NAME "Camera Image"

/*******************************************************************************************************************//**
 * @BRIEF Process a single image frame
 * @PARAM[in] imageIn the input image frame
 * @RETURN true if frame was processed successfully
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
bool processFrame(const cv::Mat &imageIn)
{
	// process the image frame
	return true;
}

/***********************************************************************************************************************
 * @BRIEF program entry point
 * @PARAM[in] argc number of command line arguments
 * @PARAM[in] argv string array of command line arguments
 * @RETURN return code (0 for normal termination)
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
int main(int argc, char **argv)
{
    // store video capture parameters
    int cameraIndex = 0;
    bool showFrames = false;

    // validate and parse the command line arguments
    if(argc != NUM_COMNMAND_LINE_ARGUMENTS + 1)
    {
        std::printf("USAGE: %s <camera_index> <display_mode> \n", argv[0]);
		std::printf("WARNING: Proceeding with default execution parameters... \n");
		cameraIndex = 0;
		showFrames = true;
    }
	else
	{
		cameraIndex = atoi(argv[1]);
		showFrames = atoi(argv[2]) > 0;
	}

    // initialize the camera capture
	cv::VideoCapture capture(cameraIndex);
	if(!capture.isOpened())
    {
        std::printf("Unable to open video source, terminating program! \n");
        return 0;
    }
	
	// get the video source paramters
	int captureWidth = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH));
	int captureHeight = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	std::printf("Video source opened successfully (width=%d height=%d)! \n", captureWidth, captureHeight);

    // create the debug image windows
    if(showFrames)
    {
		cv::namedWindow(DISPLAY_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    }

    // process data until program termination
	bool doCapture = true;
	int frameCount = 0;
    while(doCapture)
    {
        // get the start time
		double startTicks = static_cast<double>(cv::getTickCount());

        // attempt to acquire an image frame
		cv::Mat captureFrame;
        if(capture.read(captureFrame))
        {
            // process the image frame
            processFrame(captureFrame);

            // increment the frame counter
            frameCount++;
        }
        else
        {
            std::printf("Unable to acquire image frame! \n");
        }

        // update the GUI window if necessary
        if(showFrames)
        {
			cv::imshow(DISPLAY_WINDOW_NAME, captureFrame);
            
			// check for program termination
			if(cv::waitKey(1) == 'q')
			{
				doCapture = false;
			}
        }

        // compute the frame processing time
		double endTicks = static_cast<double>(cv::getTickCount());
		double elapsedTime = (endTicks - startTicks) / cv::getTickFrequency();
        std::printf("Frame processing time: %f \n", elapsedTime);
    }

    // release program resources before returning
	capture.release();
	cv::destroyAllWindows();
}
