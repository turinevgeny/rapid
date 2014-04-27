#pragma once

#include "Tracker.hpp"

class RAPIDTracker : public Tracker
{
public:
	RAPIDTracker(Model& model);
	virtual Model ProcessFrame(const cv::Mat& frame);
};