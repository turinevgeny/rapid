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
		const std::vector<cv::Point2f> foundBoxPoints2D,
		const std::vector<cv::Point3f> modelPoints3D,
		cv::Mat& out_rvec,
		cv::Mat& out_tvec) const;
	RAPIDTrackerExperiment::~RAPIDTrackerExperiment();
    friend std::ofstream& operator<<(std::ofstream file, const cv::Mat& vector);
private:
    void writeVectors(const cv::Mat& rvec, const cv::Mat& tvec);
	void getSubVectors(
		const std::vector<cv::Point3f> modelPoints3D, 
		const std::vector<cv::Point2f> foundBoxPoints2D, 
		const std::vector<unsigned> subset, 
		std::vector<cv::Point3f> &out_subModelPoints3D,
		std::vector<cv::Point2f> &out_subFoundBoxPoints2D) const;
};

#endif 