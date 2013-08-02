#ifndef __VIDEO_INFO_BASE_H
#define __VIDEO_INFO_BASE_H

#include <opencv2/core/core.hpp>

class VideoInfoInterface
{
public:
	virtual cv::Mat* GetCornerPoints() = 0;
	virtual ~VideoInfoInterface() {}
};

#endif