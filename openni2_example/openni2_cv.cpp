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
FILENAME:   openni2_cv.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
Code example for using the OpenNI2Grabber class with OpenCV

REVISION HISTORY:
08.12.2013  CDM     original file creation
09.01.2013  CDM     published under GPL
***********************************************************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "OpenNI2Grabber.h"

using namespace cv;

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
***********************************************************************************************************************/
int main(int argc, char** argv)
{
    OpenNI2Grabber grabber;
    bool isRunning = false;
    const int timeoutMs = 1000;

    // attempt to start the grabber
    if(grabber.initialize())
    {
        if(grabber.start())
        {
            isRunning = grabber.isRunning();
        }
    }
    else
    {
        std::cout << "Unable to initialize OpenNI2Grabber, program terminating!" << std::endl;
        return 0;
    }

    // acquire frames until program termination
    std::cout << "Press 'p' to render point cloud, 'q' to halt acquisition..." << std::endl;
    Mat depthImage, colorImage;
    Mat depthImageDraw;
    while(isRunning)
    {
        // wait for a new image frame
        if(grabber.waitForFrame(timeoutMs))
        {
            // update the display for both frames
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                imshow("depth", depthImageDraw);
            }
            if(grabber.getColorFrame(colorImage))
            {
                imshow("rgb", colorImage);
            }
        }
        else
        {
            std::cout << "No new frames received in " << timeoutMs << " ms..." << std::endl;
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q')
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }
    }

    // stop the acquisition
    grabber.stop();

    return 0;
}
