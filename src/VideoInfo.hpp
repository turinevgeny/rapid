#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

class VideoInfo : public VideoInfoBase
{
public:
	VideoInfo(string fileName,
			  cv::Mat T,
			  double rotationAngle,
			  cv::Mat rotationMatrix,
			  cv:Mat[] cornerPoints);

	~VideoInfo();

	int GetNumberOfCorners() {return NumberOfCorners;}
private:
	cv:Mat[] cornerPoints;
	const int NumberOfCorners = 8;
};

#endif