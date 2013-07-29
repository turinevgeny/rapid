#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

#include <opencv2/core/core.hpp>

#include "VideoInfoBase.hpp"

class VideoInfo : VideoInfoInterface
{
public:
	//VideoInfo(string fileName,
	//		  cv::Mat T,
	//		  double rotationAngle,
	//		  cv::Mat rotationMatrix,
	//		  cv:Mat[] cornerPoints);

	VideoInfo();
	virtual ~VideoInfo();

	int GetNumberOfCorners() {return NumberOfCorners;}

	//Serialization for the class
	friend cv::FileStorage &operator<<(cv::FileStorage &out, const VideoInfo &c);
	//Deserialization for the class
	friend cv::FileStorage &operator>>(cv::FileStorage &in, VideoInfo &c);

	virtual cv::Mat* GetCornerPoints();
private:
	// Serializable fields
	std::string filename;
	cv::Mat T;
	cv::Mat rotationMatrix;
	cv::Mat* cornerPointsInModelCoords;

	const int NumberOfCorners;
	cv::Mat* cornerPoints;
private:
	static std::string IntToString(int i);
};

#endif
