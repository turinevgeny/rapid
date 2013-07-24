#ifndef __VIDEO_INFO_BASE_H
#define __VIDEO_INFO_BASE_H

class VideoInfoBase
{
private:
	const int NumberOfCorners = 8;
public:
	cv:Mat[] GetCornerPoints() = 0;
	int GetNumberOfCorners() {return NumberOfCorners;}
};

#endif