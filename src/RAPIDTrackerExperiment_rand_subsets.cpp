#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include "RAPIDTrackerExperiment_rand_subsets.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;
using namespace cv;

unsigned int currCount = 0;

RAPIDTrackerExperiment_rand_subsets::RAPIDTrackerExperiment_rand_subsets(
	Model& _model, 
	bool _isLogsEnabled, 
	unsigned int _k,
	unsigned int _count)
	:	RAPIDTrackerExperiment(_model, _isLogsEnabled, _k), count(_count)
{}

RAPIDTrackerExperiment_rand_subsets::~RAPIDTrackerExperiment_rand_subsets()
{}

bool RAPIDTrackerExperiment_rand_subsets::GenerateNextSubset(
	std::vector<unsigned>& out_subset, 
	const unsigned int n) const
{
	util::RandomGenerator rng;
	rng.drawUniformSubset(n, k, out_subset);

	currCount++;
	if (currCount == count) 
	{
		currCount = 0;
		return false;
	}
	else
	{
		return true;
	}
}
