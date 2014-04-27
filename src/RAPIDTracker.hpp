#pragma once

#include <opencv2/core/core.hpp>

#include "Model.hpp"

// Describes possible search directions among a row of pixels
enum Direction {HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNWARD_DIAGONAL};

class RAPIDTracker
{
public:
	RAPIDTracker(Model& model);
	virtual Model ProcessFrame(const cv::Mat& frame);

	double GetConvergenceMeasure(const Model& model1, const Model& model2, int normType) const;
private:
	Model		model;
private:
	virtual cv::Mat		ExtractEdges(const cv::Mat& image) const;
	bool		FindPoints(cv::Point2d controlPoint,
                           cv::Point2d companionPoint,
                           const cv::Mat& edges,
                           cv::Point2d& foundPoint,
                           cv::Point2d& foundPoint2);
};