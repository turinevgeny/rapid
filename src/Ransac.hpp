#include <vector>

#include <opencv2/core/core.hpp>

/** od stands for Outliers Detection
 */
namespace od
{

struct DetectionParams
{
	cv::Mat rot, trans;
};

template <typename DataType, typename ModelParametersType>
class Ransac
{
public:
	typedef void (*TRansacFitFunctor)(
		const std::vector<DataType> &allData,
		const std::vector<unsigned> &useIndices,
		std::vector<ModelParametersType> &fitModels );

	typedef void (*TRansacDistanceFunctor)(
		const std::vector<DataType> &allData,
		const std::vector<ModelParametersType> &testModels,
		const double distanceThreshold,
		unsigned &out_bestModelIndex,
		std::vector<unsigned> &out_inlierIndices );

	typedef bool (*TRansacDegenerateFunctor)(
		const std::vector<DataType> &allData,
		const std::vector<unsigned> &useIndices );

	/** An implementation of the RANSAC algorithm for robust fitting of models to data.
		*/
	static bool execute(
		const std::vector<DataType> &data,
		TRansacFitFunctor			fit_func,
		TRansacDistanceFunctor  	dist_func,
		TRansacDegenerateFunctor 	degen_func,
		const double   				distanceThreshold,
		const size_t				minimumSizeSamplesToFit,
		std::vector<unsigned>		&out_best_inliers,
		ModelParametersType			&out_best_model,
		const double                prob_good_sample = 0.999,
		const size_t				maxIter = 2000
		);
	static int DummyMethod();
}; // end class

}
