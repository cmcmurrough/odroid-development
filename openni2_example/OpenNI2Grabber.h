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
FILENAME:   OpenNI2Grabber.h
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
This file provides convenient access to an OpenNI2 device for OpenCV and PCL

REVISION HISTORY:
08.12.2013  CDM     original file creation
09.01.2013  CDM     published under GPL
***********************************************************************************************************************/

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//using namespace pcl;

#define RESOLUTION_X_DEPTH 640
#define RESOLUTION_Y_DEPTH 480
#define FPS_DEPTH 30

#define RESOLUTION_X_COLOR 640
#define RESOLUTION_Y_COLOR 480
#define FPS_COLOR 30

#define DEPTH_CX 319.5
#define DEPTH_CY 239.5
#define DEPTH_FX 525.0
#define DEPTH_FY 525.0

/***********************************************************************************************************************
class OpenNI2Grabber
This class contains conventient wrapper functions for accessing an OpenNI2 device
***********************************************************************************************************************/
class OpenNI2Grabber
{
private:

    openni::Device myDevice;

    openni::VideoStream myDepthStream;
    openni::VideoStream myColorStream;

    std::vector <openni::VideoStream*> myStreams;

    openni::VideoMode myDepthVideoMode;
    openni::VideoMode myColorVideoMode;

    openni::VideoFrameRef myDepthFrame;
    openni::VideoFrameRef myColorFrame;

    bool myGetDepth;
    bool myGetColor;
    bool myImageRegistrationEnabled;
    bool myIsDepthColorSyncEnabled;
    bool myIsInitialized;
    bool myIsRunning;

    int myNumStreams;
    int myDepthStreamIndex;
    int myColorStreamIndex;

public:

    OpenNI2Grabber() {};
    ~OpenNI2Grabber()
    {
        myDepthStream.stop();
        myColorStream.stop();
        myDepthStream.destroy();
        myColorStream.destroy();
        myStreams.clear();
        myDevice.close();

        // delete myDepthFrame and myColorFrame?
        openni::OpenNI::shutdown();
    };

    bool initialize(bool getDepth, bool getColor, bool setRegistration, bool setDepthColorSync);
    bool isInitialized();
    bool isRunning();
    bool isImageRegistrationEnabled();
    bool isDepthStreamEnabled();
    bool isColorStreamEnabled();
    bool IsDepthColorSyncEnabled();
    bool waitForFrame(int wait_ms);
    bool getDepthFrame(cv::Mat& depthImage);
    bool getColorFrame(cv::Mat& colorImage);

    bool start();
    void stop();

    void depthToDisparity(cv::Mat& depthImage, cv::Mat& disparityImage);
    void depthToMeters(cv::Mat& depthMM, cv::Mat& depthM);
    void makeCloud(cv::Mat& depthImage, cv::Mat& colorImage, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut);
};

/***********************************************************************************************************************
bool initialize(bool getDepth = true, bool getColor = true, bool setRegistration = true, bool setDepthColorSync = true)
initialize the grabber for depth and/or color acquisition
***********************************************************************************************************************/
bool OpenNI2Grabber::initialize(bool getDepth = true, bool getColor = true, bool setRegistration = true, bool setDepthColorSync = true)
{
    myImageRegistrationEnabled = false;
    myGetDepth = false;
    myGetColor = false;
    myIsInitialized = false;
    myIsRunning = false;
    myNumStreams = 0;
    myDepthStreamIndex = 0;
    myColorStreamIndex = 0;

    // return if no stream is specified
    if(!getDepth && !getColor)
    {
        std::cout << "No acquisition streams specified! " << std::endl;
        return false;
    }

    // initialize the OpenNI API
    openni::OpenNI::initialize();

    // return false if unable to initialize the device
    if (myDevice.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
        std::cout << "Unable to initialize OpenNI device!" << std::endl;
        openni::OpenNI::shutdown();
        return false;
    }

    // initialize the stream pointer array
    myGetDepth = getDepth;
    myGetColor = getColor;
    if(myGetDepth && myGetColor)
    {
        myNumStreams = 2;
        myDepthStreamIndex = 0;
        myColorStreamIndex = 1;
    }
    else if(myGetDepth && !myGetColor)
    {
        myNumStreams = 1;
        myDepthStreamIndex = 0;
    }
    else if(!myGetDepth && myGetColor)
    {
        myNumStreams = 1;
        myColorStreamIndex = 0;
    }

    // attempt to start stream acquisition
    try
    {
        // initialize the depth stream if necessary
        if(myGetDepth)
        {
            myDepthStream.create(myDevice, openni::SENSOR_DEPTH);
            myDepthVideoMode = myDepthStream.getVideoMode();
            myDepthVideoMode.setResolution(RESOLUTION_X_DEPTH, RESOLUTION_Y_DEPTH);
            myDepthVideoMode.setFps(FPS_DEPTH);
            myDepthStream.setVideoMode(myDepthVideoMode);
            myDepthStream.setMirroringEnabled(false);

            // update the pointer array
            myStreams.push_back(&myDepthStream);
        }

        // initialize the color stream if necessary
        if(myGetColor)
        {
            myColorStream.create(myDevice, openni::SENSOR_COLOR);
            openni::VideoMode myColorVideoMode = myColorStream.getVideoMode();
            myColorVideoMode.setResolution(RESOLUTION_X_COLOR, RESOLUTION_Y_COLOR);
            myColorVideoMode.setFps(FPS_COLOR);
            myColorStream.setVideoMode(myColorVideoMode);
            myColorStream.setMirroringEnabled(false);

            // update the pointer array
            myStreams.push_back(&myColorStream);
        }

        // set depth color synchronization if necessary
        if(setDepthColorSync & myGetDepth && myGetColor)
        {
            int status = myDevice.setDepthColorSyncEnabled(true);
            if (status != openni::STATUS_OK)
            {
                std::cout << "Unable to set depth/color synchronization: " << openni::OpenNI::getExtendedError() << std::endl;
                myIsDepthColorSyncEnabled = false;
            }
            else
            {
                myIsDepthColorSyncEnabled = true;
                std::cout << "Depth/color synchronization enabled!" << std::endl;
            }
        }

        // set depth to color registration mode
        if(setRegistration && myDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        {
            int status = myDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            if (status != openni::STATUS_OK)
            {
                std::cout << "Unable to set registration mode: " << openni::OpenNI::getExtendedError() << std::endl;
                myImageRegistrationEnabled = false;
            }
            else
            {
                myImageRegistrationEnabled = true;
                std::cout << "Registration mode enabled!" << std::endl;
            }
        }
        else
        {
            std::cout << "Registration not supported by this device!" << std::endl;
            myImageRegistrationEnabled = false;
        }

        // streams opened with no error, return true
        myIsInitialized = true;

        return true;
    }
    catch(char* error)
    {
        // we encountered an error, return false
        std::cout << "Exception occured while opening stream: " << error << std::endl;
        myImageRegistrationEnabled = false;
        myGetDepth = false;
        myGetColor = false;
        myIsInitialized = false;
        myIsRunning = false;
        myNumStreams = 0;
        myDepthStreamIndex = 0;
        myColorStreamIndex = 0;
        myDevice.close();
        openni::OpenNI::shutdown();

        return false;
    }
}

/***********************************************************************************************************************
bool isInitialized()
return the initialization state of the grabber
***********************************************************************************************************************/
bool OpenNI2Grabber::isInitialized()
{
    return myIsInitialized;
}

/***********************************************************************************************************************
bool isRunning()
return the execution state of the grabber
***********************************************************************************************************************/
bool OpenNI2Grabber::isRunning()
{
    return myIsRunning;
}

/***********************************************************************************************************************
bool isImageRegistrationEnabled()
return true if registration is enabled on the device
***********************************************************************************************************************/
bool OpenNI2Grabber::isImageRegistrationEnabled()
{
    return myImageRegistrationEnabled;
}

/***********************************************************************************************************************
bool isDepthStreamEnabled()
return true if depth acquisition is enabled
***********************************************************************************************************************/
bool OpenNI2Grabber::isDepthStreamEnabled()
{
    return myGetDepth;
}

/***********************************************************************************************************************
bool isColorStreamEnabled()
return true if color acquisition is enabled
***********************************************************************************************************************/
bool OpenNI2Grabber::isColorStreamEnabled()
{
    return myGetColor;
}

/***********************************************************************************************************************
bool IsDepthColorSyncEnabled()
return true if depth and color streams are synchronized
***********************************************************************************************************************/
bool OpenNI2Grabber::IsDepthColorSyncEnabled()
{
    return myIsDepthColorSyncEnabled;
}

/***********************************************************************************************************************
bool waitForFrame(int wait_ms)
waits up to the given duration in milliseconds for a frame update, returning true if an update was successful
***********************************************************************************************************************/
bool OpenNI2Grabber::waitForFrame(int wait_ms)
{
    int changedIndex;

    // wait for an update
    int status = openni::OpenNI::waitForAnyStream(&myStreams[0], myNumStreams, &changedIndex, wait_ms);

    // check the result of the wait
    if (status != openni::STATUS_OK)
    {
        std::cout << "No frame updates processed in " << wait_ms << " ms: " << openni::OpenNI::getExtendedError() << std::endl;
        return false;
    }

    // check the updated stream
    if(myGetDepth && changedIndex == myDepthStreamIndex)
    {
        return true;
    }
    else if(myGetColor && changedIndex == myColorStreamIndex)
    {
        return true;
    }
    else
    {
        std::cout << "Unexpected stream received! " << std::endl;
        return false;
    }
}

/***********************************************************************************************************************
bool getDepthFrame(cv::Mat& depthImage)
reads a depth frame from the stream
***********************************************************************************************************************/
bool OpenNI2Grabber::getDepthFrame(cv::Mat& depthImage)
{
    // read the frame from the depth stream
    myDepthStream.readFrame(&myDepthFrame);

    // check to see if the frame is valid
    if (myDepthFrame.isValid())
    {
        depthImage.create(myDepthFrame.getHeight(), myDepthFrame.getWidth(), CV_16UC1);
        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*) myDepthFrame.getData();
        memcpy(depthImage.data, pDepthRow, myDepthFrame.getStrideInBytes() * myDepthFrame.getHeight());

        return true;
    }
    else
    {
        return false;
    }
}

/***********************************************************************************************************************
bool getColorFrame(cv::Mat& colorImage)
reads a color frame from the stream, returning true if the conversion was successful
***********************************************************************************************************************/
bool OpenNI2Grabber::getColorFrame(cv::Mat& colorImage)
{
    // read the frame from the color stream
    myColorStream.readFrame(&myColorFrame);

    // check to see if the frame is valid
    if (myColorFrame.isValid())
    {
        colorImage.create(myColorFrame.getHeight(), myColorFrame.getWidth(), CV_8UC3);
        const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*) myColorFrame.getData();
        memcpy(colorImage.data, pImageRow, myColorFrame.getStrideInBytes() * myColorFrame.getHeight());
        cvtColor(colorImage, colorImage, CV_RGB2BGR);

        return true;
    }
    else
    {
        return false;
    }
}

/***********************************************************************************************************************
bool start()
start data acquisition
***********************************************************************************************************************/
bool OpenNI2Grabber::start()
{
    if(!myIsInitialized)
    {
        std::cout << "Grabber is not initialized, unable to start acquisition!" << std::endl;
        return false;
    }
    else
    {
        // start the depth stream if necessary
        if(myGetDepth)
        {
            if(myDepthStream.start() != openni::STATUS_OK)
            {
                std::cout << "Unable to start depth stream!" << std::endl;
                return false;
            }
        }

        // start the color stream if necessary
        if(myGetColor)
        {
            if (myColorStream.start() != openni::STATUS_OK)
            {
                std::cout << "Unable to start color stream!" << std::endl;
                return false;
            }
        }

        // the streams were started successfully
        myIsRunning = true;
        return true;
    }
}

/***********************************************************************************************************************
void stop()
halt data acquisition
***********************************************************************************************************************/
void OpenNI2Grabber::stop()
{
    // stop the depth stream if necessary
    if(myGetDepth)
    {
        myDepthStream.stop();
    }

    // stop the color stream if necessary
    if(myGetColor)
    {
        myColorStream.stop();
    }
}

/***********************************************************************************************************************
void depthToMeters(cv::Mat& depthMM, cv::Mat& depthM)
convert the integer depth image in mm to a float image in meters
***********************************************************************************************************************/
void OpenNI2Grabber::depthToMeters(cv::Mat& depthMM, cv::Mat& depthM)
{
    // convert image type to float while also scaling by 0.001
    depthMM.convertTo(depthM, CV_32FC1, 0.001f, 0);
}

/***********************************************************************************************************************
void makeCloud(cv::Mat& depthImage, cv::Mat& colorImage, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
create a point cloud structure from a given depth and color image
***********************************************************************************************************************/
void OpenNI2Grabber::makeCloud(cv::Mat& depthImage, cv::Mat& colorImage, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut)
{
    // define camera parameter constants
    const float cx = DEPTH_CX;
    const float cy = DEPTH_CY;
    const float fx = DEPTH_FX;
    const float fy = DEPTH_FY;
    const float fx_inv = 1.0f/fx;
    const float fy_inv = 1.0f/fy;

    // reset the point cloud
    cloudOut->clear();
    cloudOut->width = colorImage.cols;
    cloudOut->height = colorImage.rows;
    cloudOut->points.resize(cloudOut->width * cloudOut->height);

    uint16_t d;
    float px, py, pz;
    uchar pb, pg, pr;
    int index = 0;

    // iterate through the points
    for(int i = 0; i < colorImage.rows; i++)
    {
        for (int j = 0; j < colorImage.cols; j++)
        {
            // get the depth value
            d = depthImage.at<uint16_t>(i,j);

            /// TODO: Check for bad depth values and assign NaN

            // compute the point data
            pz = ((float) d) * 0.001f;
            px = (j - cx) * pz * fx_inv;
            py = (i - cy) * pz * fy_inv;
            pb = colorImage.at<cv::Vec3b>(i,j)[0];
            pg = colorImage.at<cv::Vec3b>(i,j)[1];
            pr = colorImage.at<cv::Vec3b>(i,j)[2];

            //cloudOut->points.push_back (point);
            cloudOut->points[index].x = px;
            cloudOut->points[index].y = py;
            cloudOut->points[index].z = pz;
            cloudOut->points[index].r = pr;
            cloudOut->points[index].g = pg;
            cloudOut->points[index].b = pb;

            index ++;
        }
    }
}
