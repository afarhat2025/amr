/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#ifdef RTABMAP_DEPTHAI
#ifndef DEPTHAI_OPENCV_SUPPORT
#define DEPTHAI_OPENCV_SUPPORT
#endif
#include <depthai/depthai.hpp>
#endif

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraDepthAI :
	public Camera
{
public:
	static bool available();

public:
	CameraDepthAI(
			const std::string & mxidOrName = "",
			int imageWidth = 1280, // 640 or 1280
			float imageRate = 0.0f,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraDepthAI();

	void setOutputMode(int outputMode = 0);
	void setDepthProfile(int confThreshold = 200, int lrcThreshold = 5);
	void setExtendedDisparity(bool extendedDisparity = false, bool enableCompanding = false);
	void setSubpixelMode(bool enabled = false, int fractionalBits = 3);
	void setDisparityWidthAndFilter(int disparityWidth = 96, int medianFilter = 5);
	void setRectification(bool useSpecTranslation = false, float alphaScaling = 0.0f, bool enabled = true);
	void setIMU(bool imuPublished, bool publishInterIMU);
	void setIrIntensity(float dotIntensity = 0.0f, float floodIntensity = 0.0f);
	void setDetectFeatures(int detectFeatures = 0, const std::string & blobPath = "");
	void setGFTTDetector(bool useHarrisDetector = false, float minDistance = 7.0f, int numTargetFeatures = 1000);
	void setSuperPointDetector(float threshold = 0.01f, bool nms = true, int nmsRadius = 4);

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0);

private:
#ifdef RTABMAP_DEPTHAI
	StereoCameraModel stereoModel_;
	cv::Size targetSize_;
	Transform imuLocalTransform_;
	std::string mxidOrName_;
	int outputMode_;
	int confThreshold_;
	int lrcThreshold_;
	int imageWidth_;
	bool extendedDisparity_;
	bool enableCompanding_;
	int subpixelFractionalBits_;
	int disparityWidth_;
	int medianFilter_;
	bool useSpecTranslation_;
	float alphaScaling_;
	bool imagesRectified_;
	bool imuPublished_;
	bool publishInterIMU_;
	float dotIntensity_;
	float floodIntensity_;
	int detectFeatures_;
	bool useHarrisDetector_;
	float minDistance_;
	int numTargetFeatures_;
	float threshold_;
	bool nms_;
	int nmsRadius_;
	std::string blobPath_;
	std::unique_ptr<dai::Device> device_;
	std::shared_ptr<dai::DataOutputQueue> cameraQueue_;
	std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
	UMutex imuMutex_;
#endif
};


} // namespace rtabmap
