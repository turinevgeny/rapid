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
	out << "VideoInfo";
	out << "{"
			<< "filename" << c.filename
			<< "T" << c.T
			<< "rotationMatrix" << c.rotationMatrix
			<< "cornerPointsInModelCoords"
			<< "{";
					for(int i = 0; i < c.NumberOfCorners; i++)
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
	c.filename = (std::string) node["filename"];
	node["T"] >> c.T;
	node["rotationMatrix"] >> c.rotationMatrix;

	cv::FileNode cornerPointsInModelCoordsNode = node["cornerPointsInModelCoords"];
	for (int i = 0; i < c.NumberOfCorners; i++)
	{
		std::string nodeName =  "cornerPointsInModelCoords" + c.IntToString(i);
		cornerPointsInModelCoordsNode[nodeName] >> c.cornerPointsInModelCoords[i];
	}

	return in;
}

cv::Mat* VideoInfo::GetCornerPoints()
{
	return cornerPoints;
}

VideoInfo::VideoInfo() : NumberOfCorners(8) 
{
	filename = "../../BoxVideo2/new1.MOV";
	T = (cv::Mat_<double>(1,3) << -15, 120, 352);

	const double a = 145.0;
	const double b = 65.0;
	const double c = 45.0;

	const double alpha = CV_PI/2 - acos(100/a);

	rotationMatrix = (cv::Mat_<double>(3,3) <<  cos(alpha), 0, sin(alpha),
												0,          1, 0,
												-sin(alpha), 0, cos(alpha));

	cornerPointsInModelCoords = new cv::Mat[NumberOfCorners];

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
}
