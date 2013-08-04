#include <iostream>

#include <opencv2/core/core.hpp>

#include "VideoInfo.hpp"

std::string VideoInfo::IntToString(int i)
{
	std::stringstream number;
	number << i;
	return number.str();
}

cv::FileStorage& operator<<(cv::FileStorage& out, const VideoInfo& c) 
{
	if (!c.IsInitialized())
	{
		std::cerr << "VideoInfo Error: Instance hasn't been initialized" << std::endl;
		return out;
	}

	out << "VideoInfo";
	out << "{"
			<< "numberOfCorners" << c.numberOfCorners
			<< "videoPath" << c.videoPath
			<< "calibDataPath" << c.calibDataPath
			<< "T" << c.T
			<< "rotationMatrix" << c.rotationMatrix
			<< "cornerPointsInModelCoords"
			<< "{";
					for(int i = 0; i < c.numberOfCorners; i++)
					{
						std::string nodeName =  "cornerPointsInModelCoords" + c.IntToString(i);
						out << nodeName<< c.cornerPointsInModelCoords[i];
					}
		 out << "}"
		  << "}";

	return out;
}

cv::FileStorage& operator>>(cv::FileStorage& in, VideoInfo& c)
{
	cv::FileNode node = in["VideoInfo"];
	c.numberOfCorners = (int) node["numberOfCorners"];
	c.videoPath = (std::string) node["videoPath"];
	c.calibDataPath = (std::string) node["calibDataPath"];
	node["T"] >> c.T;
	node["rotationMatrix"] >> c.rotationMatrix;

	delete[] c.cornerPointsInModelCoords;
	c.cornerPointsInModelCoords = new cv::Mat[c.numberOfCorners];

	cv::FileNode cornerPointsInModelCoordsNode = node["cornerPointsInModelCoords"];
	for (int i = 0; i < c.numberOfCorners; i++)
	{
		std::string nodeName =  "cornerPointsInModelCoords" + c.IntToString(i);
		cornerPointsInModelCoordsNode[nodeName] >> c.cornerPointsInModelCoords[i];
	}

	return in;
}

cv::Mat* VideoInfo::GetCornerPoints()
{
	cornerPoints = new cv::Mat[numberOfCorners];

	for (int i = 0; i < numberOfCorners; ++i)
	{
		cornerPoints[i] = cornerPointsInModelCoords[i] * rotationMatrix;
	}

	return cornerPoints;
}

void VideoInfo::MockUp()
{

	numberOfCorners = 8;

	videoPath = "../../video/test.MOV";
	calibDataPath = "../../camera.xml";
	T = (cv::Mat_<double>(1,3) << -15, 120, 352);

	const double a = 145.0;
	const double b = 65.0;
	const double c = 45.0;

	const double alpha = CV_PI/2 - acos(100/a);

	rotationMatrix = (cv::Mat_<double>(3,3) <<  cos(alpha),  0, sin(alpha),
												0,           1, 0,
												-sin(alpha), 0, cos(alpha));

	cornerPointsInModelCoords = new cv::Mat[numberOfCorners];

	cornerPointsInModelCoords[0] = (cv::Mat_<double>(1,3) << 0,  0, 0);
	cornerPointsInModelCoords[1] = (cv::Mat_<double>(1,3) << b,  0, 0);
	cornerPointsInModelCoords[2] = (cv::Mat_<double>(1,3) << b,  0, a);
	cornerPointsInModelCoords[3] = (cv::Mat_<double>(1,3) << 0,  0, a);
	cornerPointsInModelCoords[4] = (cv::Mat_<double>(1,3) << 0, -c, 0);
	cornerPointsInModelCoords[5] = (cv::Mat_<double>(1,3) << b, -c, 0);
	cornerPointsInModelCoords[6] = (cv::Mat_<double>(1,3) << b, -c, a);
	cornerPointsInModelCoords[7] = (cv::Mat_<double>(1,3) << 0, -c, a);
}

VideoInfo::~VideoInfo()
{
	delete[] cornerPointsInModelCoords;
	delete[] cornerPoints;
}
