#include <iostream>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

namespace Maths
{

int Square(int value)
{
    return value*value;
}

double CrossProduct(const cv::Mat& a, const cv::Mat& b)
{
	return a.dot(b);
}

}

TEST(firstTest, SquareOf10)
{
	EXPECT_EQ(100, Maths::Square(10));
}

TEST(firstTest, SquareOf9)
{
	EXPECT_EQ(81, Maths::Square(9));
}

TEST(secondTest, CrossProductTesting)
{
	cv::Mat A = cv::Mat::zeros(1, 30, CV_32F);
	cv::Mat B = cv::Mat::zeros(1, 30, CV_32F);

	EXPECT_EQ(0.0, Maths::CrossProduct(A, B));
}
