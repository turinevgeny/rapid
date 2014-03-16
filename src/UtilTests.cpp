#include <iostream>
#include <vector>
#include <algorithm>

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

TEST(RandomGenerator, drawUniformSubset)
{
    util::RandomGenerator rng;

    std::vector<unsigned> subset;

    const size_t cardinality = 20;

    rng.drawUniformSubset(cardinality, subset);

    size_t subset_cardinality = subset.size();

    std::sort ( subset.begin(), subset.end() );
    std::vector<unsigned>::iterator fisrt_duplicate = std::unique ( subset.begin(), subset.end() );

    EXPECT_TRUE(fisrt_duplicate == subset.end()) << "Found duplicates in subset indices";

    for(int i = 0; i < subset.size(); i++)
    {
        EXPECT_LE(subset[i], cardinality) << "Found an index too large";
        EXPECT_GE(subset[i], 0) << "Found a negative index";
    }
}