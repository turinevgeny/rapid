#include <iostream>

#include <opencv2/core/core.hpp>

#include "VideoInfo.hpp"

using namespace cv;

std::string VideoInfo::IntToString(int i)
{
	std::stringstream number;
	number << i;
	return number.str();
}

FileStorage& operator<<(FileStorage& out, const VideoInfo& c)
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

FileStorage& operator>>(FileStorage& in, VideoInfo& c)
{
	FileNode node = in["VideoInfo"];
	c.numberOfCorners = (int) node["numberOfCorners"];
	c.videoPath = (std::string) node["videoPath"];
	c.calibDataPath = (std::string) node["calibDataPath"];
	node["rotationMatrix"] >> c.rotationMatrix;

	delete[] c.cornerPointsInModelCoords;
	c.cornerPointsInModelCoords = new Mat[c.numberOfCorners];

	FileNode cornerPointsInModelCoordsNode = node["cornerPointsInModelCoords"];
	for (int i = 0; i < c.numberOfCorners; i++)
	{
		std::string nodeName =  "cornerPointsInModelCoords" + c.IntToString(i);
		cornerPointsInModelCoordsNode[nodeName] >> c.cornerPointsInModelCoords[i];
	}

	return in;
}

Mat* VideoInfo::GetCornerPoints()
{
	cornerPoints = new Mat[numberOfCorners];

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

	const double a = 145.0;
	const double b = 65.0;
	const double c = 45.0;

	const double alpha = CV_PI/2 - acos(100/a);

	rotationMatrix = (Mat_<double>(3,3) <<  cos(alpha),  0, sin(alpha),
												0,           1, 0,
												-sin(alpha), 0, cos(alpha));

	cornerPointsInModelCoords = new Mat[numberOfCorners];

	cornerPointsInModelCoords[0] = (Mat_<double>(1,3) << 0,  0, 0);
	cornerPointsInModelCoords[1] = (Mat_<double>(1,3) << b,  0, 0);
	cornerPointsInModelCoords[2] = (Mat_<double>(1,3) << b,  0, a);
	cornerPointsInModelCoords[3] = (Mat_<double>(1,3) << 0,  0, a);
	cornerPointsInModelCoords[4] = (Mat_<double>(1,3) << 0, -c, 0);
	cornerPointsInModelCoords[5] = (Mat_<double>(1,3) << b, -c, 0);
	cornerPointsInModelCoords[6] = (Mat_<double>(1,3) << b, -c, a);
	cornerPointsInModelCoords[7] = (Mat_<double>(1,3) << 0, -c, a);
}

VideoInfo::~VideoInfo()
{
	delete[] cornerPointsInModelCoords;
	delete[] cornerPoints;
}
