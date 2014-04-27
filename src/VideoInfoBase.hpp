#pragma once

#include <opencv2/core/core.hpp>

class VideoInfoInterface
{
public:
	virtual cv::Mat* GetCornerPoints() = 0;
	virtual std::string GetVideoPath() = 0;
	virtual std::string GetCalibDataPath() = 0;

	virtual ~VideoInfoInterface() {}
};