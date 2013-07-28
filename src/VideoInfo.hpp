#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

#include <opencv2/core/core.hpp>

#include "VideoInfoBase.hpp"

class VideoInfo : VideoInfoBase
{
public:
	//VideoInfo(string fileName,
	//		  cv::Mat T,
	//		  double rotationAngle,
	//		  cv::Mat rotationMatrix,
	//		  cv:Mat[] cornerPoints);

	VideoInfo() : NumberOfCorners(8), filename("../../BoxVideo2/new1.MOV") {}
	virtual ~VideoInfo() {}

	int GetNumberOfCorners() {return NumberOfCorners;}

public:
	//Serialization for the class
	friend cv::FileStorage &operator<<(cv::FileStorage &out, const VideoInfo &c);
	//Deserialization for the class
	friend cv::FileStorage &operator>>(cv::FileStorage &in, VideoInfo &c);

	virtual cv::Mat* GetCornerPoints();

private:
	std::string filename;
	//cv::Mat T;
	//double rotationAngle;
	//cv::Mat rotationMatrix;
	//cv::Mat* cornerPointsInModelCoords;

	const int NumberOfCorners;
};

#endif