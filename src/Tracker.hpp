#pragma once

#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "Model.hpp"

// Describes possible search directions among a row of pixels
enum Direction {HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNWARD_DIAGONAL};

class Tracker
{
public:
    Tracker(Model _model) : model(_model)
    { }
    double GetConvergenceMeasure(const Model& model1, const Model& model2, int normType) const;
	virtual cv::Mat	ExtractEdges(const cv::Mat& image) const;

protected:
    bool FindPoints(cv::Point2d controlPoint,
        cv::Point2d companionPoint,
        const cv::Mat& edges,
        cv::Point2d& foundPoint,
        cv::Point2d& foundPoint2);
protected:
	Model model;
};