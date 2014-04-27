#pragma once

#include "Tracker.hpp"

class RAPIDTracker : public Tracker
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
};