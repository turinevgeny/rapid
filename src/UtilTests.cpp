#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include "Util.hpp"

TEST(RandomGenerator, drawUniformVector)
{
	util::RandomGenerator rng;
	const double min = 0.0;
	const double max = 1.0;

	const size_t size = 5;

	std::vector<double> vec(size);

	rng.drawUniformVector< std::vector<double> >(vec, min, max);

	for(int i = 0; i < size; i++)
	{
		EXPECT_LE(vec[i], max);
		EXPECT_GE(vec[i], min);
	}
}