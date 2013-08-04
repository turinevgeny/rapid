#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

#include <opencv2/core/core.hpp>

#include "VideoInfoBase.hpp"

class VideoInfo : VideoInfoInterface
{
public:
	VideoInfo() : cornerPointsInModelCoords(NULL) {}

	virtual cv::Mat* GetCornerPoints();
	int GetNumberOfCorners() {return NumberOfCorners;}
	bool IsInitialized() const {return !(cornerPointsInModelCoords == NULL);}
	// Fills the class fields basing on /new1.MOV on Dropbox
	void MockUp();

	//Serialization for the class
	friend cv::FileStorage &operator<<(cv::FileStorage &out, const VideoInfo &c);
	//Deserialization for the class
	friend cv::FileStorage &operator>>(cv::FileStorage &in, VideoInfo &c);

	virtual ~VideoInfo();
private:
	// Serializable fields
	std::string filename;
	cv::Mat T;
	cv::Mat rotationMatrix;
	cv::Mat* cornerPointsInModelCoords;
	int NumberOfCorners;

	cv::Mat* cornerPoints;
private:
	static std::string IntToString(int i);
};

#endif
