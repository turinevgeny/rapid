#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>

namespace util
{
class RandomGenerator
{
public:
	RandomGenerator()
		: rng()
	{ }
	RandomGenerator(uint64 seed)
		: rng(seed)
	{ }

	/** Fills the given vector with independent, uniformly distributed samples.
		*/
	template <class VecType>
	void drawUniformVector(
		VecType &v,
		const double unif_min = 0,
		const double unif_max = 1 )
	{
		const size_t N = v.size();
		for (size_t c = 0; c < N; c++)
			v[c] = static_cast<typename VecType::value_type>( rng.uniform(unif_min, unif_max) );
	}

private:
	cv::RNG rng;
};
}