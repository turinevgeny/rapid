#pragma once

#include <list>

#include <opencv2/core/core.hpp>

class OutlierDetector
{
public:
	OutlierDetector() { }
	virtual std::list<cv::Point2d> RemoveOutliers(std::list<cv::Point2d>&) = 0;
private:
	OutlierDetector(OutlierDetector const &);
	OutlierDetector & operator=(OutlierDetector const &);
};

/*
	size – Subset size.
	thresh – Maximum re-projection error value to classify as inlier.
	eps – Maximum ratio of incorrect correspondences.
	prob – Required success probability.
*/
class Ransac : public OutlierDetector
{
public:
	Ransac(int size, double thresh, double eps, double prob)
		: _size(size), _thresh(thresh), _eps(eps), _prob(prob)
	{ }
	virtual std::list<cv::Point2d> RemoveOutliers(std::list<cv::Point2d>&);
private:
	int _size;		// subset size
	double _thresh;	// max error to classify as inlier
	double _eps;	// max outliers ratio - ??? is it really necessary?
	double _prob;	// probability of success
};