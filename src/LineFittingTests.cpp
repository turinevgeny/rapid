#include <opencv2/core/core.hpp>
#include <gtest/gtest.h>

#include "LineFitting.hpp"
#include "Ransac.hpp"

class LineFittingTest : public ::testing::Test
{
protected:
   void SetUp() {
       lf::FillData(allData);
       const double distanceThreshold = 0.1;
       const size_t minimumSizeSamplesToFit = 2;
       const double prob_good_sample = 0.99;
       const size_t maxIter = 2000;
       const std::vector<cv::Point2d> allData_const(allData);

       /*od::Ransac<cv::Point2d,cv::Point2d>::execute(
           allData_const,
           lf::LineFitFunctor,
           lf::LineDistanceFunctor,
           distanceThreshold,
           minimumSizeSamplesToFit,
           out_best_inliers,
           out_best_model,
           prob_good_sample,
           maxIter);*/
   }


protected:
   std::vector<cv::Point2d> allData;
   std::vector<unsigned> out_best_inliers;
   cv::Point2d out_best_model;
};

TEST_F(LineFittingTest, LineFunctors)
{
    std::vector<cv::Point2d> fitModels;
    std::vector<unsigned> useIndices;
    useIndices.push_back(0); useIndices.push_back(1);

    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(allData[useIndices[0]], cv::Point2d(0,0));
    EXPECT_EQ(allData[useIndices[1]], cv::Point2d(1,2));
    ASSERT_EQ(fitModels.size(), 1);
    EXPECT_EQ(fitModels[0].x, 2);
    EXPECT_EQ(fitModels[0].y, 0);

    useIndices[0] = 4; useIndices[1] = 5;

    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(allData[useIndices[0]], cv::Point2d(3,3));
    EXPECT_EQ(allData[useIndices[1]], cv::Point2d(4,4));
    ASSERT_EQ(fitModels.size(), 2);
    EXPECT_EQ(fitModels[1].x, 1);
    EXPECT_EQ(fitModels[1].y, 0);

    unsigned out_bestModelIndex;
    std::vector<unsigned> out_inlierIndices;

    lf::LineDistanceFunctor(allData, fitModels, 3, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(out_bestModelIndex, 1);
    EXPECT_EQ(out_inlierIndices.size(), 6);

    out_inlierIndices.clear();
    lf::LineDistanceFunctor(allData, fitModels, 0.5, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(out_bestModelIndex, 1);
    EXPECT_EQ(out_inlierIndices.size(), 4);

    useIndices[0] = 6; useIndices[1] = 2;

    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(allData[useIndices[0]], cv::Point2d(10,2));
    EXPECT_EQ(allData[useIndices[1]], cv::Point2d(2,2));
    ASSERT_EQ(fitModels.size(), 3);
    EXPECT_EQ(fitModels[2].x, 0);
    EXPECT_EQ(fitModels[2].y, 2);

    out_inlierIndices.clear();
    lf::LineDistanceFunctor(allData, fitModels, 1, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(out_bestModelIndex, 1);
    EXPECT_EQ(out_inlierIndices.size(), 6);
}

TEST(LineFitting, ComputeLineDistance)
{
    double ld;
    ld = lf::ComputeLineDistance(cv::Point2d(10,2),cv::Point2d(1,0));
    EXPECT_EQ(ld, 8);
    ld = lf::ComputeLineDistance(cv::Point2d(10,2),cv::Point2d(2,0));
    EXPECT_EQ(ld, 18);
    ld = lf::ComputeLineDistance(cv::Point2d(3,3),cv::Point2d(1,0));
    EXPECT_EQ(ld, 0);
    ld = lf::ComputeLineDistance(cv::Point2d(2,2),cv::Point2d(1,1));
    EXPECT_EQ(ld, 1);
}