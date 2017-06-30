/*
Copyright (c) 2014, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "VideoInput.hpp"

#ifdef _MSC_VER
#   include <Dshow.h>
#	include "FlyCapture2.h"
using namespace FlyCapture2;
#endif

#ifdef Q_OS_MAC
#   include "VideoInput_QTkit.hpp"
#endif

#ifdef Q_OS_LINUX
#   include <unistd.h>
#   include <fcntl.h>
#   include <linux/videodev2.h>
#   include <sys/ioctl.h>
#   include <errno.h>
#   define V4L2_MAX_DEVICE_DRIVER_NAME 80
#   define V4L2_MAX_CAMERAS 8
#endif

#include <QApplication>
#include <QMetaType>
#include <QTime>
#include <QMap>

#include <stdio.h>
#include <iomanip>
#include <iostream>

//bool VideoInput::_cam_webcam = false;
//bool VideoInput::_cam_PG = false;

VideoInput::VideoInput(QObject  * parent): 
    QThread(parent),
    _camera_index(-1),
    _video_capture(NULL),
    _init(false),
    _stop(false)
{
    qRegisterMetaType<cv::Mat>("cv::Mat");
}

VideoInput::~VideoInput()
{
    //stop_camera(true);
}

void VideoInput::run()
{
    _init = false;
    _stop = false;
 
	Error error;
	BusManager busMgr;
	PGRGuid guid;
	Camera cam;
	unsigned int numCameras;

	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}
	if (numCameras < 1)
	{
		std::cout << "No camera detected." << std::endl;
		return;
	}
	else
	{
		std::cout << "Number of cameras detected: " << numCameras << std::endl;
	}

	error = busMgr.GetCameraFromIndex(0, &guid);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}

	error = cam.Connect(&guid);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}    

	_init = true;
	
	// Properties Set Up
	// Check if the camera supports the FRAME_RATE property
	std::cout << "Detecting frame rate from camera... " << std::endl;
	PropertyInfo propInfo;
	propInfo.type = FRAME_RATE;
	error = cam.GetPropertyInfo(&propInfo);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}
	float frameRateToUse = 60.0f;
	if (propInfo.present == true)
	{
		// Get the frame rate
		Property prop;
		prop.type = FRAME_RATE;
		error = cam.GetProperty(&prop);
		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
		}
		else
		{
			// Set the frame rate.
			// Note that the actual recording frame rate may be slower,
			// depending on the bus speed and disk writing speed.
			prop.absValue = frameRateToUse;
		}
	}
	std::cout << "Using frame rate of " << std::fixed << std::setprecision(1) << frameRateToUse << std::endl;

	// Setting Shutter using the FlyCapture API
	// Declare a property struct.
	Property prop;
	// Define the property to adjust
	prop.type = SHUTTER;
	// Ensure the property is on
	prop.onOff = true;
	// Ensure auto-adjust mode is off
	prop.autoManualMode = false;
	// Ensure the property is set up to use absolute value control
	/*prop.absControl = true;
	// Set the absolute value of shutter to 17ms
	prop.absValue = 17;
	// Set the property
	error = cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}*/

	// Setting Gain Using the FlyCapture API
	prop.type = GAIN;
	prop.onOff = true;
	prop.autoManualMode = false;
	/*prop.absControl = true;
	prop.absValue = 17;
	error = cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}*/

	// Setting Auto Exposure Using the FlyCapture API
	prop.type = AUTO_EXPOSURE;
	prop.onOff = true;
	prop.autoManualMode = false;
	/*prop.absControl = true;
	prop.absValue = 0.92;
	error = cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}*/

	// Setting White Balance Using the FlyCapture API
	prop.type = WHITE_BALANCE;
	prop.onOff = true;
	prop.autoManualMode = false;
	/*// Set the white balance red channel to 595
	prop.valueA = 595;
	// Set the white balance blue channel to 965
	//prop.valueB = 875;
	//error = cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}*/

	error = cam.StartCapture();
	if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
	{
		std::cout << "Bandwidth exceeded" << std::endl;
		return;
	}
	else if (error != PGRERROR_OK)
	{
		std::cout << "Failed to start image capture" << std::endl;
		return;
	}

	int error_frame = 0;
	int max_error = 10;
	int warmup = 10000;
	QTime timer;
	timer.start();
	while ( !_stop && error_frame < max_error)
	{
		Image rawImage;
		error = cam.RetrieveBuffer(&rawImage);
		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
			if (timer.elapsed() > warmup) { error_frame++; }
			//return;
		}
		error_frame = 0;
		// convert to rgb
		Image rgbImage;
		rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
		cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
        // 180 rotation
        cv::transpose( image, image );
        cv::flip( image, image, 0 );
        cv::transpose( image, image );
        cv::flip( image, image, 0 );

		emit new_image(image);
		//sleep(5);
	}

	error = cam.StopCapture();
	if (error != PGRERROR_OK)
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}
    //clean up
    QApplication::processEvents();
}

void VideoInput::waitForStart(void)
{
    while (isRunning() && !_init)
    {
        QApplication::processEvents();
    }
}

void VideoInput::setImageSize(size_t width, size_t height)
{
  if (_video_capture)
  {
    cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_WIDTH, width);
    cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_HEIGHT, height);
    std::cerr << "setImageSize: " << width << "x" << height << std::endl;
  }
}

QStringList VideoInput::list_devices(void)
{
    QStringList list;
    bool silent = false;
#ifdef _MSC_VER
    list = list_devices_dshow(silent);
#endif
    return list;
}

QStringList VideoInput::list_device_resolutions(int index)
{
    QStringList list;
    bool silent = true;
#ifdef _MSC_VER
    list = list_device_resolutions_dshow(index, silent);
#endif
    return list;
}

QStringList VideoInput::list_devices_dshow(bool silent)
{
    if (!silent) { printf("\nVIDEOINPUT SPY MODE!\n\n"); }

    QStringList list;

#ifdef _MSC_VER
	// Looking for Point Grey Cameras
	if (!silent) { printf("Looking for a Point Grey Camera\n\n"); }
	Error error;
	BusManager busMgr;
	unsigned int numCameras;
	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
	}
	else
	{
		if (numCameras < 1)
		{
			std::cout << "No camera detected." << std::endl;
		}
		else
		{
			for (int i = 0; i < numCameras; i++)
			{
				PGRGuid guid;
				error = busMgr.GetCameraFromIndex(i, &guid);
				if (error != PGRERROR_OK)
				{
					error.PrintErrorTrace();
				}
				else
				{
					Camera cam;
					error = cam.Connect(&guid);
					if (error != PGRERROR_OK)
					{
						error.PrintErrorTrace();
					}
					else
					{
						// Get the camera information
						CameraInfo camInfo;
						error = cam.GetCameraInfo(&camInfo);
						if (error != PGRERROR_OK)
						{
							error.PrintErrorTrace();
						}
						else
						{
							list.append(camInfo.modelName);
						}
					}
				}
			}
		}
		if (!silent) { printf("SETUP: %i Point Grey Camera(s) found\n", numCameras); }
	}

#endif //_MSC_VER
	/*for (int j = 0; j < list.size(); ++j)
		std::cout << list.at(j).toLocal8Bit().constData() << std::endl;*/
    return list;
}

void VideoInput::configure_dshow(int index, bool silent)
{
    QStringList resList = list_device_resolutions_dshow(index, silent);
    unsigned int pixCount = 0, cols = 0, rows = 0;
    foreach (auto resString, resList)
    {
        QStringList res = resString.split('x');
        if (res.length()<2) { continue; }

        unsigned int curCols = res.at(0).toUInt();
        unsigned int curRows = res.at(1).toUInt();
        unsigned int curPixCount = curCols*curRows;
        if (curPixCount>pixCount)
        {
            pixCount = curPixCount;
            cols = curCols;
            rows = curRows;
        }
    }

    if (pixCount)
    {
        cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_WIDTH, cols);
        cvSetCaptureProperty(_video_capture, CV_CAP_PROP_FRAME_HEIGHT, rows);
    }
}

QStringList VideoInput::list_device_resolutions_dshow(int index, bool silent)
{
    QStringList list;

#ifdef _MSC_VER
		Error error;
		BusManager busMgr;
		unsigned int numCameras;
		error = busMgr.GetNumOfCameras(&numCameras);
		if (error != PGRERROR_OK)
		{
			error.PrintErrorTrace();
		}
		else
		{
			if (numCameras < 1)
			{
				std::cout << "No camera detected." << std::endl;
			}
			else
			{
				for (int i = 0; i < numCameras; i++)
				{
					PGRGuid guid;
					error = busMgr.GetCameraFromIndex(i, &guid);
					if (error != PGRERROR_OK)
					{
						error.PrintErrorTrace();
					}
					else
					{
						Camera cam;
						error = cam.Connect(&guid);
						if (error != PGRERROR_OK)
						{
							error.PrintErrorTrace();
						}
						else
						{
							// Get the camera information
							CameraInfo camInfo;
							error = cam.GetCameraInfo(&camInfo);
							if (error != PGRERROR_OK)
							{
								error.PrintErrorTrace();
							}
							else
							{
								list.append(camInfo.sensorResolution);
							}
						}
					}
				}
			}
		}
#endif //_MSC_VER

    return list;
}

