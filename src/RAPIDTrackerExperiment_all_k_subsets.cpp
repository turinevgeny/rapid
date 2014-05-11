#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include "RAPIDTrackerExperiment_all_k_subsets.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTrackerExperiment_all_k_subsets::RAPIDTrackerExperiment_all_k_subsets(
	Model& _model, 
	bool _isLogsEnabled, 
	unsigned int _k)
	:	RAPIDTrackerExperiment(_model, _isLogsEnabled, _k)
{}

RAPIDTrackerExperiment_all_k_subsets::~RAPIDTrackerExperiment_all_k_subsets()
{}

bool RAPIDTrackerExperiment_all_k_subsets::GenerateNextSubset(std::vector<unsigned> &out_subset, const unsigned int n) const
{
	for (int i = k - 1; i >= 0; i--)
		if ( out_subset[i] + 1 <= n - 1) {
			if ( i != k - 1 && out_subset[i + 1] == out_subset[i] + 1 )
				continue;
			++out_subset[i];

			for (int j = i + 1; j < k; j++)
				out_subset[j] = out_subset[j - 1] + 1;
			return true;
		}
		return false;
}
