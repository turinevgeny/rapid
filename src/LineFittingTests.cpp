#include <opencv2/core/core.hpp>
#include <gtest/gtest.h>

#include "LineFitting.hpp"
#include "Ransac.hpp"

//Error messages related to LineFitFunctor
const char pointDiscrepancy [] = {"Point doesn't correspond to the test. Check alldata vector"};
const char pushError[] = {"LineFitFunctor doesn't push fitting Model"};
const char incorrectFitting[] = {"LineFitFunctor fit the model incorrectly"};

//Error messages related to execute method
const char unexpectedBestModel[] = {"Ransac hasn't find expected model"};
const char unexpectedNumInliers[] = {"Ransac return unexpected number of inliers"};
const char unexpectedInliersIndices[] = {"Ransac return unexpected inliers indices"};

//Error messages related to LineDistanceFunctor
const char incorrectBestModelIndex[] = {"LineDistanceFunctor return incorrect best model index"};
const char wrongNumInliers[] = {"LineDistanceFunctor return the wrong number of inliers"};
const char wrongInliersIndices[] = {"LineDistanceFunctor return wrong inlier indices"};

class LineFittingTest : public ::testing::Test
{
protected:
   void SetUp()
   {
       lf::FillData(allData);
   }
protected:
   std::vector<cv::Point2d> allData;

   //for execute
   std::vector<unsigned> out_best_inliers;
   cv::Point2d out_best_model;

   //for LineFitFunctor
   std::vector<cv::Point2d> fitModels;
   std::vector<unsigned> useIndices;

   //for LineDistanceFunctor
   unsigned out_bestModelIndex;
   std::vector<unsigned> out_inlierIndices;
};

TEST_F(LineFittingTest, LineFunctors)
{
    // Checking of LineFitFunctor
    useIndices.push_back(0); useIndices.push_back(1);
    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(cv::Point2d(0,0), allData[useIndices[0]]) << pointDiscrepancy;
    EXPECT_EQ(cv::Point2d(1,2), allData[useIndices[1]]) << pointDiscrepancy;
    ASSERT_EQ(1, fitModels.size())<< pushError;
    EXPECT_EQ(cv::Point2d(2,0), fitModels[0]) << incorrectFitting;

    //Checking of LineFitFunctor with another points
    useIndices[0] = 6; useIndices[1] = 2;
    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(cv::Point2d(10,2), allData[useIndices[0]]) << pointDiscrepancy;
    EXPECT_EQ(cv::Point2d(2,2), allData[useIndices[1]]) << pointDiscrepancy;
    ASSERT_EQ(2, fitModels.size()) << pushError;
    EXPECT_EQ(cv::Point2d(0,2), fitModels[1]) << incorrectFitting;

    //Checking of LineDistanceFunctor
    lf::LineDistanceFunctor(allData, fitModels, 3, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(1, out_bestModelIndex) << incorrectBestModelIndex;
    EXPECT_EQ(7, out_inlierIndices.size()) << wrongNumInliers;

    std::vector<unsigned> expected;
    for(int i=0; i<7; i++)
        expected.push_back(i);
    EXPECT_EQ (expected, out_inlierIndices) << wrongInliersIndices;

    //Checking of LineDistanceFunctor with another treshold
    out_inlierIndices.clear();
    lf::LineDistanceFunctor(allData, fitModels, 0.5, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(out_bestModelIndex, 1) << incorrectBestModelIndex;
    EXPECT_EQ(out_inlierIndices.size(), 4) << wrongNumInliers;

    // Checking of LineFitFunctor and manipulations with fitModels
    useIndices[0] = 4; useIndices[1] = 5;
    lf::LineFitFunctor(allData, useIndices, fitModels);
    EXPECT_EQ(cv::Point2d(3,3), allData[useIndices[0]]) << pointDiscrepancy;
    EXPECT_EQ(cv::Point2d(4,4), allData[useIndices[1]]) << pointDiscrepancy;
    ASSERT_EQ(3, fitModels.size()) << pushError;
    EXPECT_EQ(cv::Point2d(1,0), fitModels[2]) << incorrectFitting;

    //Checking LineDistanceFunctor with the updated fitModels array and with lower treshold
    out_inlierIndices.clear();
    lf::LineDistanceFunctor(allData, fitModels, 1, out_bestModelIndex, out_inlierIndices);
    EXPECT_EQ(2, out_bestModelIndex) << incorrectBestModelIndex;
    EXPECT_EQ(6, out_inlierIndices.size()) << wrongNumInliers;
    expected.pop_back();
    EXPECT_EQ (expected, out_inlierIndices) << wrongInliersIndices;
}

TEST_F(LineFittingTest, ExecuteQuitContaminated)
{
    const std::vector<cv::Point2d> allData_const(allData);
    const double distanceThreshold = 1;
    const size_t minimumSizeSamplesToFit = 2;
    const double prob_good_sample = 0.99;
    const size_t maxIter = 2000;

    od::Ransac<cv::Point2d,cv::Point2d>::execute(
           allData_const,
           lf::LineFitFunctor,
           lf::LineDistanceFunctor,
           distanceThreshold,
           minimumSizeSamplesToFit,
           out_best_inliers,
           out_best_model,
           prob_good_sample,
           maxIter);

    EXPECT_EQ(cv::Point2d(1,0), out_best_model) << unexpectedBestModel;
    EXPECT_EQ(6, out_best_inliers.size()) << unexpectedNumInliers;

    std::vector<unsigned> expected;
    for(int i=0; i<6; i++)
        expected.push_back(i);
    EXPECT_EQ (expected, out_best_inliers) << unexpectedInliersIndices;
}

TEST_F(LineFittingTest, ExecuteAlmostUncontaminated)
{
    allData.clear();
    lf::FillAlmostUncontaminatedData(allData);

    const std::vector<cv::Point2d> allData_const(allData);
    const double distanceThreshold = 1;
    const size_t minimumSizeSamplesToFit = 2;
    const double prob_good_sample = 0.99;
    const size_t maxIter = 2000;

    od::Ransac<cv::Point2d,cv::Point2d>::execute(
           allData_const,
           lf::LineFitFunctor,
           lf::LineDistanceFunctor,
           distanceThreshold,
           minimumSizeSamplesToFit,
           out_best_inliers,
           out_best_model,
           prob_good_sample,
           maxIter);

    EXPECT_EQ(cv::Point2d(1,0), out_best_model) << unexpectedBestModel;
    EXPECT_EQ(12, out_best_inliers.size()) << unexpectedNumInliers;

    std::vector<unsigned> expected;
    for(int i=0; i<12; i++)
        expected.push_back(i);
    EXPECT_EQ (expected, out_best_inliers) << unexpectedInliersIndices;
}

TEST_F(LineFittingTest, ExecuteContaminated)
{
    allData.clear();
    lf::FillAlmostUncontaminatedData(allData);
    lf::AddOutliers(allData);

    const std::vector<cv::Point2d> allData_const(allData);
    const double distanceThreshold = 0.1;
    const size_t minimumSizeSamplesToFit = 2;
    const double prob_good_sample = 0.99;
    const size_t maxIter = 2000;

    od::Ransac<cv::Point2d,cv::Point2d>::execute(
           allData_const,
           lf::LineFitFunctor,
           lf::LineDistanceFunctor,
           distanceThreshold,
           minimumSizeSamplesToFit,
           out_best_inliers,
           out_best_model,
           prob_good_sample,
           maxIter);

    EXPECT_EQ(cv::Point2d(1,0), out_best_model) << unexpectedBestModel;
    EXPECT_EQ(12, out_best_inliers.size()) << unexpectedNumInliers;

    std::vector<unsigned> expected;
    for(int i=0; i<12; i++)
        expected.push_back(i);
    EXPECT_EQ (expected, out_best_inliers) << unexpectedInliersIndices;
}

TEST(LineFitting, ComputeLineDistance)
{
    double ld;
    ld = lf::ComputeLineDistance(cv::Point2d(10,2),cv::Point2d(1,0));
    EXPECT_EQ(8, ld) << "incorrect distance";
    ld = lf::ComputeLineDistance(cv::Point2d(10,2),cv::Point2d(2,0));
    EXPECT_EQ(18, ld) << "incorrect distance";
    ld = lf::ComputeLineDistance(cv::Point2d(3,3),cv::Point2d(1,0));
    EXPECT_EQ(0, ld) << "incorrect distance. Point lies on the line";
    ld = lf::ComputeLineDistance(cv::Point2d(2,2),cv::Point2d(1,1));
    EXPECT_EQ(1, ld) << "incorrect distance";
}

TEST(LineFitting, EuclideanDistance)
{
    double dist;
    dist = lf::euclideanDistance(cv::Point2d(2,2), cv::Point2d(2,2));
    EXPECT_EQ(0, dist) << "The distance between identical points is not equal to zero";

    dist = lf::euclideanDistance(cv::Point2d(-1,0), cv::Point2d(2,0));
    EXPECT_EQ(3, dist) << "Incorrect distance along the Y axis";

    dist = lf::euclideanDistance(cv::Point2d(0,2), cv::Point2d(0,-1));
    EXPECT_EQ(3, dist) << "Incorrect distance along the X axis";

    dist = lf::euclideanDistance(cv::Point2d(100,100), cv::Point2d(101,101));
    EXPECT_EQ(cv::sqrt(2.0), dist) << "Incorrect distance";
}