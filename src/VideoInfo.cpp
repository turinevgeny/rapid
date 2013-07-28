#include "VideoInfo.hpp"

#include <opencv2/core/core.hpp>

cv::FileStorage &operator<<(cv::FileStorage &out, const VideoInfo &c) 
{
	out << "VideoInfo";
	out << "{" 
		<< "filename" << c.filename
		<< "}";

	return out;
}

cv::FileStorage &operator>>(cv::FileStorage &in, VideoInfo &c)
{
	cv::FileNode node = in["VideoInfo"];
	c.filename = (std::string) node["filename"];

	return in;
}

cv::Mat* VideoInfo::GetCornerPoints()
{
	throw std::exception("The method or operation is not implemented.");
}
