#ifndef __RAPIDTRacker_H
#define __RAPIDTRacker_H

#include <opencv2/core/core.hpp>
#include "Model.hpp"

// Describes possible search directions among a row of pixels
enum Direction{HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNWARD_DIAGONAL};

class RAPIDTracker
{
private:
	Model		model;
	std::string videoFile;
public:
				RAPIDTracker(const std::string videoFile, const Model &model);
	Model		ProcessFrame(const cv::Mat &frame);
private:
	cv::Mat		ExtractEdges(const cv::Mat &image) const;
	double		GetDisplacement(cv::Point2d controlPoint, cv::Point2d companionPoint, const cv::Mat &edges, cv::Point2d &foundPoint);
};

#endif