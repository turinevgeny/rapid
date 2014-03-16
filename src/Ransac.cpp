#include "Ransac.hpp"

/*---------------------------------------------------------------
			Ransac generic implementation
 ---------------------------------------------------------------*/
template <typename DataType, typename ModelParametersType>
bool od::Ransac<DataType, ModelParametersType>::execute(
    const std::vector<DataType> &data,
    TRansacFitFunctor           fit_func,
    TRansacDistanceFunctor      dist_func,
    TRansacDegenerateFunctor    degen_func,
    const double                distanceThreshold,
    const size_t                minimumSizeSamplesToFit,
    std::vector<unsigned>       &out_best_inliers,
    ModelParametersType         &out_best_model,
    const double                prob_good_sample,
    const size_t                maxIter
    )
{
	CV_Assert(minimumSizeSamplesToFit >= 1);

	const size_t Npts = data.size();

	CV_Assert(Npts > 1);

	const size_t maxDataTrials = 100; // Maximum number of attempts to select a non-degenerate data set

	out_best_inliers.clear();

    size_t trialcount = 0;
    size_t bestscore = std::string::npos; // npos will mean "none"
    size_t N = 1;     // Dummy initialization for number of trials

    std::vector<unsigned> ind(minimumSizeSamplesToFit);

    while (N > trialcount)
    {
        // Select at random s data points to form a trial model, M.
        // In selecting these points we have to check that they are not in
        // a degenerate configuration.
        bool degenerate = true;
        size_t count = 1;
        std::vector<ModelParametersType> models;

        while (degenerate)
        {
            // Generate s random indices in the range 1..npts
            ind.resize(minimumSizeSamplesToFit);

			util::RandomGenerator randomGenerator;
			// The +0.99... is due to the floor rounding afterwards when converting from random double samples to size_t
			randomGenerator.drawUniformVector(ind, 0.0, Npts - 1 + 0.999999);

            // Test that these points are not a degenerate configuration.
            degenerate = degen_func(data, ind);

            if (!degenerate)
            {
                // Fit model to this random selection of data points.
                // Note that M may represent a set of models that fit the data
                fit_func(data, ind, models);

                // Depending on your problem it might be that the only way you
                // can determine whether a data set is degenerate or not is to
				// try to fit a model and see if it succeeds. If it fails we
                // reset degenerate to true.
                degenerate = models.empty();
			}

            // Safeguard against being stuck in this loop forever
            if (++count > maxDataTrials)
            {
                std::cerr << "[RANSAC] Unable to select a non-degenerate data set" << std::endl;
                return false;
            }
        }

        // Once we are out here we should have some kind of model...
        // Evaluate distances between points and model returning the indices
        // of elements in x that are inliers.
        unsigned int bestModelIdx = 1000;
        std::vector<unsigned> inliers;
        if (!degenerate)
        {
			dist_func(data, models, distanceThreshold, bestModelIdx, inliers);
			CV_Assert(bestModelIdx < models.size());
        }

        // Find the number of inliers to this model.
        const size_t ninliers = inliers.size();
        bool update_estim_num_iters = (trialcount==0); // Always update on the first iteration, regardless of the result (even for ninliers=0)

        if (ninliers > bestscore || (bestscore==std::string::npos && ninliers!=0))
        {
            bestscore = ninliers;  // Record data for this model

            out_best_model    = models[bestModelIdx];
            out_best_inliers  = inliers;
            update_estim_num_iters = true;
        }

        if (update_estim_num_iters)
        {
            // Update estimate of N, the number of trials to ensure we pick,
            // with probability p, a data set with no outliers.
            double fracinliers = ninliers/static_cast<double>(Npts);
            double pNoOutliers = 1 - pow(fracinliers, static_cast<double>(minimumSizeSamplesToFit));

            pNoOutliers = std::max( std::numeric_limits<double>::epsilon(), pNoOutliers);  // Avoid division by -Inf
            pNoOutliers = std::min(1.0 - std::numeric_limits<double>::epsilon() , pNoOutliers); // Avoid division by 0
            // Number of
            N = static_cast<size_t>(log(1 - prob_good_sample) / log(pNoOutliers));
        }

        ++trialcount;

        // Safeguard against being stuck in this loop forever
        if (trialcount > maxIter)
        {
            std::cerr << "[RANSAC] Warning: maximum number of trials reached" << std::endl;
            return false;
        }
    }

	return true;
}

template <typename DataType, typename ModelParametersType>
bool od::Ransac<DataType, ModelParametersType>::execute(
    const std::vector<DataType> &data,
    TRansacFitFunctor           fit_func,
    TRansacDistanceFunctor      dist_func,
    const double                distanceThreshold,
    const size_t                minimumSizeSamplesToFit,
    std::vector<unsigned>       &out_best_inliers,
    ModelParametersType         &out_best_model,
    const double                prob_good_sample,
    const size_t                maxIter
    )
{
	return execute(
		data,
		fit_func,
		dist_func,
		TrueDegenerativeFunction,
		distanceThreshold,
		minimumSizeSamplesToFit,
		out_best_inliers,
		out_best_model,
		prob_good_sample,
		maxIter);
}

template <typename DataType, typename ModelParametersType>
bool od::Ransac<DataType, ModelParametersType>::TrueDegenerativeFunction(
	const std::vector<DataType> &allData,
	const std::vector<unsigned> &useIndices )
{
	return true;
}

// Template instantiation:
template class od::Ransac<cv::Point2d, od::DetectionParams>;