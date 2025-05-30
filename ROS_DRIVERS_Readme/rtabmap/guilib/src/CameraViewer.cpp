/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/SensorEvent.h>
#include "rtabmap/gui/CameraViewer.h"

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/MarkerDetector.h>
#include <rtabmap/gui/ImageView.h>
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/ULogger.h>
#include <QtCore/QMetaType>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QPushButton>

namespace rtabmap {


CameraViewer::CameraViewer(QWidget * parent, const ParametersMap & parameters) :
		QDialog(parent),
	imageView_(new ImageView(this)),
	cloudView_(new CloudViewer(this)),
	processingImages_(false),
	parameters_(parameters),
	markerDetector_(0)
{
	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");

	imageView_->setImageDepthShown(true);
	imageView_->setMinimumSize(320, 240);
	imageView_->setVisible(false);
	QHBoxLayout * layout = new QHBoxLayout();
	layout->setContentsMargins(0,0,0,0);
	layout->addWidget(imageView_,1);
	layout->addWidget(cloudView_,1);

	QLabel * decimationLabel = new QLabel("Decimation", this);
	decimationSpin_ = new QSpinBox(this);
	decimationSpin_->setMinimum(-16);
	decimationSpin_->setMaximum(16);
	decimationSpin_->setValue(2);

	pause_ = new QPushButton("Pause", this);
	pause_->setCheckable(true);
	showCloudCheckbox_ = new QCheckBox("Show RGB-D cloud", this);
	showCloudCheckbox_->setEnabled(false);
	showCloudCheckbox_->setChecked(true);
	showScanCheckbox_ = new QCheckBox("Show scan", this);
	showScanCheckbox_->setEnabled(false);
	showScanCheckbox_->setChecked(true);

	markerCheckbox_ = new QCheckBox("Detect markers", this);
#ifdef HAVE_OPENCV_ARUCO
	markerCheckbox_->setEnabled(true);
	markerDetector_ = new MarkerDetector(parameters);
#else
	markerCheckbox_->setEnabled(false);
	markerCheckbox_->setToolTip("Disabled: RTAB-Map is not built with OpenCV's aruco module.");
#endif
	markerCheckbox_->setChecked(false);

	imageSizeLabel_ = new QLabel(this);

	QDialogButtonBox * buttonBox = new QDialogButtonBox(this);
	buttonBox->setStandardButtons(QDialogButtonBox::Close);
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

	QHBoxLayout * layout2 = new QHBoxLayout();
	layout2->addWidget(pause_);
	layout2->addWidget(decimationLabel);
	layout2->addWidget(decimationSpin_);
	layout2->addWidget(showCloudCheckbox_);
	layout2->addWidget(showScanCheckbox_);
	layout2->addWidget(markerCheckbox_);
	layout2->addWidget(imageSizeLabel_);
	layout2->addStretch(1);
	layout2->addWidget(buttonBox);

	QVBoxLayout * vlayout = new QVBoxLayout(this);
	vlayout->setContentsMargins(0,0,0,0);
	vlayout->setSpacing(0);
	vlayout->addLayout(layout, 1);
	vlayout->addLayout(layout2);

	this->setLayout(vlayout);
}

CameraViewer::~CameraViewer()
{
	this->unregisterFromEventsManager();
	delete markerDetector_;
}

void CameraViewer::setDecimation(int value)
{
	decimationSpin_->setValue(value);
}

void CameraViewer::showImage(const rtabmap::SensorData & data)
{
	processingImages_ = true;
	QString sizes;

	cv::Mat left;
	cv::Mat depthOrRight;
	LaserScan scan;
	if( !data.imageRaw().empty() || !data.imageCompressed().empty() ||
		!data.depthOrRightRaw().empty() || !data.depthOrRightCompressed().empty() ||
		!data.laserScanRaw().empty() || !data.laserScanCompressed().empty())
	{
		data.uncompressDataConst(
			!data.imageRaw().empty() || !data.imageCompressed().empty()?&left:0, 
			!data.depthOrRightRaw().empty() || !data.depthOrRightCompressed().empty()?&depthOrRight:0,
			!data.laserScanRaw().empty() || !data.laserScanCompressed().empty()?&scan:0);
	}

	imageView_->setVisible(!left.empty() || !left.empty());
	std::map<int, MarkerInfo> detections;
	if(!left.empty())
	{
		std::vector<CameraModel> models;
		if(markerCheckbox_->isEnabled() && markerCheckbox_->isChecked())
		{
			models = data.cameraModels();
			if(models.empty())
			{
				for(size_t i=0; i<data.stereoCameraModels().size(); ++i)
				{
					models.push_back(data.stereoCameraModels()[i].left());
				}
			}
		}
			
		if(!models.empty() && models[0].isValidForProjection())
		{
			cv::Mat imageWithDetections;
			detections = markerDetector_->detect(left, models, depthOrRight, std::map<int, float>(), &imageWithDetections);
			imageView_->setImage(uCvMat2QImage(imageWithDetections));
		}
		else
		{
			imageView_->setImage(uCvMat2QImage(left));
		}
		sizes.append(QString("Color=%1x%2").arg(left.cols).arg(left.rows));
	}
	if(!depthOrRight.empty())
	{
		imageView_->setImageDepth(depthOrRight);
		sizes.append(QString(" Depth=%1x%2").arg(depthOrRight.cols).arg(depthOrRight.rows));
	}
	imageSizeLabel_->setText(sizes);

	if(!depthOrRight.empty() &&
	   ((data.stereoCameraModels().size() && data.stereoCameraModels()[0].isValidForProjection()) || (data.cameraModels().size() && data.cameraModels().at(0).isValidForProjection())))
	{
		if(showCloudCheckbox_->isChecked())
		{
			if(!left.empty() && !depthOrRight.empty())
			{
				showCloudCheckbox_->setEnabled(true);
				if(data.imageRaw().empty())
				{
					if(!data.stereoCameraModels().empty())
					{
						cloudView_->addCloud("cloud", util3d::cloudRGBFromSensorData(SensorData(left, depthOrRight, data.stereoCameraModels()), decimationSpin_->value()!=0?decimationSpin_->value():1, 0, 0, 0, parameters_));
					}
					else
					{
						cloudView_->addCloud("cloud", util3d::cloudRGBFromSensorData(SensorData(left, depthOrRight, data.cameraModels()), decimationSpin_->value()!=0?decimationSpin_->value():1, 0, 0, 0, parameters_));
					}
					
				}
				else
				{
					cloudView_->addCloud("cloud", util3d::cloudRGBFromSensorData(data, decimationSpin_->value()!=0?decimationSpin_->value():1, 0, 0, 0, parameters_));
				}
			}
			else if(!depthOrRight.empty())
			{
				showCloudCheckbox_->setEnabled(true);
				if(data.depthOrRightRaw().empty())
				{
					cloudView_->addCloud("cloud", util3d::cloudFromSensorData(SensorData(cv::Mat(), depthOrRight, data.cameraModels()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1, 0, 0, 0, parameters_));
				}
				else
				{
					cloudView_->addCloud("cloud", util3d::cloudFromSensorData(data, decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1, 0, 0, 0, parameters_));
				}
			}

			// Add landmarks to 3D Map view
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
			cloudView_->removeAllCoordinates("landmark_");
#endif
			cloudView_->removeAllTexts();
			if(!detections.empty())
			{
				for(std::map<int, MarkerInfo>::const_iterator iter=detections.begin(); iter!=detections.end(); ++iter)
				{
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
					cloudView_->addOrUpdateCoordinate(uFormat("landmark_%d", iter->first), iter->second.pose(), iter->second.length(), false);
#endif
					std::string num = uNumber2Str(iter->first);
					cloudView_->addOrUpdateText(
							std::string("landmark_str_") + num,
							num,
							iter->second.pose(),
							0.05,
							Qt::yellow);
				}
			}
		}
	}

	if(!scan.isEmpty())
	{
		showScanCheckbox_->setEnabled(true);
		if(showScanCheckbox_->isChecked())
		{
			if(scan.hasNormals())
			{
				if(scan.hasIntensity())
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudINormal(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
				}
				else if(scan.hasRGB())
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudRGBNormal(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
				}
				else
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudNormal(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
				}
			}
			else if(scan.hasIntensity())
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudI(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
			}
			else if(scan.hasRGB())
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudRGB(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
			}
			else
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloud(scan), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), scan.localTransform(), Qt::yellow);
			}
		}
	}

	cloudView_->setVisible((showCloudCheckbox_->isEnabled() && showCloudCheckbox_->isChecked()) ||
						   (showScanCheckbox_->isEnabled() && showScanCheckbox_->isChecked()));
	if(cloudView_->isVisible())
	{
		cloudView_->refreshView();
	}
	if(cloudView_->getAddedClouds().contains("cloud"))
	{
		cloudView_->setCloudVisibility("cloud", showCloudCheckbox_->isChecked());
	}
	if(cloudView_->getAddedClouds().contains("scan"))
	{
		cloudView_->setCloudVisibility("scan", showScanCheckbox_->isChecked());
	}

	processingImages_ = false;
}

bool CameraViewer::handleEvent(UEvent * event)
{
	if(!pause_->isChecked())
	{
		if(event->getClassName().compare("SensorEvent") == 0)
		{
			SensorEvent * camEvent = (SensorEvent*)event;
			if(camEvent->getCode() == SensorEvent::kCodeData)
			{
				if(camEvent->data().isValid())
				{
					if(!processingImages_ && this->isVisible() && camEvent->data().isValid())
					{
						processingImages_ = true;
						QMetaObject::invokeMethod(this, "showImage",
								Q_ARG(rtabmap::SensorData, camEvent->data()));
					}
				}
			}
		}
	}
	return false;
}

} /* namespace rtabmap */
