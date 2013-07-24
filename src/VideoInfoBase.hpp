#ifndef __VIDEO_INFO_BASE_H
#define __VIDEO_INFO_BASE_H

class VideoInfoBase
{
public:
	virtual cv:Mat[] GetCornerPoints() = 0;
};

#endif