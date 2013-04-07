#ifndef __RAPIDTRacker_H
#define __RAPIDTRacker_H

#include <opencv2/core/core.hpp>
#include "Model.hpp"

class RAPIDTracker
{
private:
	Model model;
private:
	Mat	ExtractEdges(Mat &image);
public:
	RAPIDTracker(const Model &model);
	Model ProcessFrame();
};

#endif