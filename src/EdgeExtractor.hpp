#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

class EdgeExtractor
{
public:
    virtual void GetAndDrawCanny(cv::Mat& edges) const
    {
        cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
        cv::Canny(edges, edges, 20, 100, 3);

        cv::namedWindow("canny", CV_WINDOW_AUTOSIZE);
        cv::imshow("canny",edges);
    }
};