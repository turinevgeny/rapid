#include <opencv2/core/core.hpp>

/** lf stands for Line Fitting
 */
namespace lf
{
void FillData(std::vector<cv::Point2d> &allData)
{
    allData.push_back(cv::Point2d(0,0));
    allData.push_back(cv::Point2d(1,2));
    allData.push_back(cv::Point2d(2,2));
    allData.push_back(cv::Point2d(3,2));
    allData.push_back(cv::Point2d(3,3));
    allData.push_back(cv::Point2d(4,4));
    allData.push_back(cv::Point2d(10,2));
}

// y = mx + b
cv::Point2d ComputeLineParams(const cv::Point2d& p1, const cv::Point2d& p2)
{
    if (p1.x == p2.x) return cv::Point2d(0,0);
    // m
    double m = (p1.y - p2.y)/(p1.x - p2.x);
    // b
    double b = p1.y - p1.x * m;
    return cv::Point2d(m,b);
}

// yi - m * xi - b
double ComputeLineDistance(const cv::Point2d& point, const cv::Point2d& model)
{
    return abs(point.y - model.x * point.x - model.y);
}

void LineFitFunctor(const std::vector<cv::Point2d> &allData,
                    const std::vector<unsigned> &useIndices,
                    std::vector<cv::Point2d> &fitModels)
{
    fitModels.push_back(lf::ComputeLineParams(allData[useIndices[0]], allData[useIndices[1]]));
}

void LineDistanceFunctor(const std::vector<cv::Point2d> &allData,
                         const std::vector<cv::Point2d> &testModels,
                         const double distanceThreshold,
                         unsigned &out_bestModelIndex,
                         std::vector<unsigned> &out_inlierIndices)
{
    out_bestModelIndex = 0;
    int bestNumInliers = 0;
    int currNumInliers = 0;

    for(int idxModel = 0; idxModel < testModels.size(); idxModel++)
    {
        currNumInliers = 0;
        for(int idxData=0; idxData < allData.size(); idxData++)
        {
            if ( ComputeLineDistance(allData[idxData], testModels[idxModel]) <= distanceThreshold)
            {
               currNumInliers++;
            }
        }

        if(currNumInliers > bestNumInliers)
        {
            out_bestModelIndex = idxModel;
            bestNumInliers = currNumInliers;
        }
    }

    for(int idxData=0; idxData < allData.size(); idxData++)
    {
        if ( ComputeLineDistance(allData[idxData], testModels[out_bestModelIndex]) <= distanceThreshold)
        {
            out_inlierIndices.push_back(idxData);
        }
    }
}

}