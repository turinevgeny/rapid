#pragma once

#include <fstream>

#include <opencv2/core/core.hpp>

#include "Model.hpp"
#include "RAPIDTrackerExperiment.hpp"

class RAPIDTrackerExperiment_rand_subsets : public RAPIDTrackerExperiment
{
public:
	RAPIDTrackerExperiment_rand_subsets::RAPIDTrackerExperiment_rand_subsets(
		Model& _model, 
		bool _isLogsEnabled, 
		unsigned int _k,
		unsigned int _count);
	RAPIDTrackerExperiment_rand_subsets::~RAPIDTrackerExperiment_rand_subsets();
protected:
	virtual bool GenerateNextSubset(
		std::vector<unsigned> &subset,
		const unsigned int n) const;
private:
	unsigned int count;
};
