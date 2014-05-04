#pragma once

#include "Tracker.hpp"

class RansacTracker : public Tracker
{
public:
    RansacTracker(
        Model model,
        bool isLogsEnabled,
        int iterationsCount = 100,
        float reprojectionError = 8.0,
        int minInliersCount = 100);

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;
private:
    int _iterationsCount;
    float _reprojectionError;
    int _minInliersCount;
};