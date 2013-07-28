#ifndef __VIDEO_INFO_BASE_H
#define __VIDEO_INFO_BASE_H

#include <opencv2/core/core.hpp>

class VideoInfoBase
{
public:
	virtual cv::Mat* GetCornerPoints() = 0;
	virtual ~VideoInfoBase() {}
};

#endif