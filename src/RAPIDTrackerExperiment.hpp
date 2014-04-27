#ifndef __RAPIDTRacker_EXPERIMENT_H
#define __RAPIDTRacker_EXPERIMENT_H

#include <fstream>

#include <opencv2/core/core.hpp>

#include "Model.hpp"
#include "RAPIDTracker.hpp"

class RAPIDTrackerExperiment : public RAPIDTracker
{
public:
	RAPIDTrackerExperiment::RAPIDTrackerExperiment(Model& _model);
	virtual void RunSolvePnP(
		const std::vector<cv::Point2d> foundBoxPoints2D,
		const std::vector<cv::Point3d> modelPoints3D,
		cv::Mat& out_rvec,
		cv::Mat& out_tvec);
	RAPIDTrackerExperiment::~RAPIDTrackerExperiment();
private:
	std::ofstream file;
private:
	void getSubVectors(
		const std::vector<cv::Point3d> modelPoints3D, 
		const std::vector<cv::Point2d> foundBoxPoints2D, 
		const std::vector<unsigned> subset, 
		std::vector<cv::Point3d> &out_subModelPoints3D,
		std::vector<cv::Point2d> &out_subFoundBoxPoints2D);
};

#endif 