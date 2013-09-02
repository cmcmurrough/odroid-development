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
FILENAME:   openni2_pcl.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
Provides an example of using OpenNI2Grabber to assemble point clouds from depth and rgb images

REVISION HISTORY:
08.12.2013  CDM     original file creation
09.01.2013  CDM     published under GPL
***********************************************************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "OpenNI2Grabber.h"

using namespace cv;

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
***********************************************************************************************************************/
int main(int argc, char** argv)
{
    // create the cloud viewer object
    pcl::visualization::CloudViewer viewer("Point Cloud");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);




    bool isRunning = false;
    OpenNI2Grabber grabber;

    // attempt to start the grabber
    if(grabber.initialize(true, true, true, true))
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
    std::cout << "Press 'q' to halt acquisition..." << std::endl;
    Mat depthImage, colorImage;
    Mat depthImageDraw;
    Mat disparityImage;
    Mat depthImageMeters;
    while(isRunning)
    {
        // acquire an image frame
        if(grabber.waitForFrame(1000))
        {
            // display the acquired images
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                cv::imshow("depth", depthImageDraw);

            }
            if(grabber.getColorFrame(colorImage))
            {
                cv::imshow("rgb", colorImage);
            }
        }
        else
        {
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q' || key == 27)
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }

        // render the point cloud
        if(key == 'p' || key == 'P')
        {
            //grabber.makeCloud(depthImageMeters, colorImage, cloud);
            grabber.makeCloud(depthImage, colorImage, cloud);
            std::cout << "rendering point cloud with " << cloud->size() << " points..." << std::endl;
            viewer.showCloud(cloud);
        }
    }

    grabber.stop();

    return 0;
}
