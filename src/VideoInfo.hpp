#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

#include <opencv2/core/core.hpp>

#include "VideoInfoBase.hpp"

class VideoInfo : VideoInfoInterface
{
public:
	VideoInfo() : cornerPointsInModelCoords(NULL), cornerPoints(NULL) {}

	// VideoInfoInterface implementation
	virtual cv::Mat* GetCornerPoints();
	virtual std::string GetVideoPath() { return videoPath; }
	virtual std::string GetCalibDataPath() { return calibDataPath; }

	// getters
	int GetNumberOfCorners() const { return numberOfCorners; }
	//cv::Mat GetModelCoordsOrigin() const { return T; }

	bool IsInitialized() const { return !(cornerPointsInModelCoords == NULL); }
	// Fills the class fields basing on /new1.MOV on Dropbox
	void MockUp();

	//Serialization for the class
	friend cv::FileStorage& operator<<(cv::FileStorage& out, const VideoInfo& c);
	//Deserialization for the class
	friend cv::FileStorage& operator>>(cv::FileStorage& in, VideoInfo& c);

	virtual ~VideoInfo();
private:
	// Serializable fields
	std::string videoPath;
	std::string calibDataPath;
	//cv::Mat T;
	cv::Mat rotationMatrix;
	cv::Mat* cornerPointsInModelCoords;
	int numberOfCorners;

	cv::Mat* cornerPoints;
private:
	static std::string IntToString(int i);
};

#endif
