#ifndef __RAPIDTRacker_H
#define __RAPIDTRacker_H

#include <opencv2/core/core.hpp>
#include "Model.hpp"

enum Direction{HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNLOAD_DIAGONAL};

class RAPIDTracker
{
private:
	Model model;
	std::string videoFile;
private:
	cv::Mat	ExtractEdges(const cv::Mat &image) const;
	Direction FindSearchDirection(cv::Point2d controlPoint, cv::Point2d companionPoint) const;
	cv::Point2d FindImageEdge(const cv::Mat &edges, const cv::Point2d startPoint, Direction searchDirection);
	double getDisplacement(cv::Point2d imageEdgePoint, cv::Point2d startPoint, Direction direction); // p.74
	// 1 method??
public:
	RAPIDTracker(const std::string videoFile, const Model &model);
	Model ProcessFrame(const cv::Mat &frame);
};

#endif