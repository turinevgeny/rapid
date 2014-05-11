#pragma once

#include <fstream>

#include <opencv2/core/core.hpp>

#include "Model.hpp"
#include "RAPIDTracker.hpp"

class RAPIDTrackerExperiment : public RAPIDTracker
{
public:
	RAPIDTrackerExperiment::RAPIDTrackerExperiment(
		Model& _model, 
		bool _isLogsEnabled,
		unsigned int k);
	
	RAPIDTrackerExperiment::~RAPIDTrackerExperiment();
protected:
	virtual bool GenerateNextSubset(
		std::vector<unsigned> &subset,
		const unsigned int n) const = 0;
	virtual void RunSolvePnP(
		const std::vector<cv::Point2f> foundBoxPoints2D,
		const std::vector<cv::Point3f> modelPoints3D,
		cv::Mat& out_rvec,
		cv::Mat& out_tvec) const;
	virtual void getSubVectors(
		const std::vector<cv::Point3f> modelPoints3D, 
		const std::vector<cv::Point2f> foundBoxPoints2D, 
		const std::vector<unsigned> subset, 
		std::vector<cv::Point3f> &out_subModelPoints3D,
		std::vector<cv::Point2f> &out_subFoundBoxPoints2D) const;
	virtual void  OutputRvecAndTvec(
		const cv::Mat out_rvec,
		const cv::Mat out_tvec,
		std::ofstream& file) const;
protected:
	unsigned int k;
};
