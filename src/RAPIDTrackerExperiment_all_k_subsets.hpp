#pragma once

#include <fstream>

#include <opencv2/core/core.hpp>

#include "Model.hpp"
#include "RAPIDTrackerExperiment.hpp"

class RAPIDTrackerExperiment_all_k_subsets : public RAPIDTrackerExperiment
{
public:
	RAPIDTrackerExperiment_all_k_subsets::RAPIDTrackerExperiment_all_k_subsets(
		Model& _model, 
		bool _isLogsEnabled,
		unsigned int k);
	
	RAPIDTrackerExperiment_all_k_subsets::~RAPIDTrackerExperiment_all_k_subsets();
protected:
	virtual bool GenerateNextSubset(
		std::vector<unsigned> &subset,
		const unsigned int n) const;
};
