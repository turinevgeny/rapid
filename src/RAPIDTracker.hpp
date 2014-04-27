#ifndef __RAPIDTRacker_H
#define __RAPIDTRacker_H

#include <opencv2/core/core.hpp>

#include "Model.hpp"

// Describes possible search directions among a row of pixels
enum Direction {HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNWARD_DIAGONAL};

class RAPIDTracker
{
public:
	RAPIDTracker(Model& model);
	virtual void GetAndDrawPointsForSolvePnP(
		const cv::Mat& frame,
		std::vector<cv::Point2d>& out_foundBoxPoints2D,
		std::vector<cv::Point3d>& out_modelPoints3D);
	virtual void RunSolvePnP(
		const std::vector<cv::Point2d> foundBoxPoints2D,
		const std::vector<cv::Point3d> modelPoints3D,
		cv::Mat& out_rvec,
		cv::Mat& out_tvec);
	virtual Model ProcessFrame(const cv::Mat& frame);

	double GetConvergenceMeasure(const Model& model1, const Model& model2, int normType);
protected:
	Model		model;
private:
	virtual cv::Mat		ExtractEdges(const cv::Mat& image) const;
	bool		FindPoints(cv::Point2d controlPoint,
                           cv::Point2d companionPoint,
                           const cv::Mat& edges,
                           cv::Point2d& foundPoint,
                           cv::Point2d& foundPoint2);
};

#endif