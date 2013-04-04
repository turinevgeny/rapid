#ifndef __NEW1_H
#define __NEW1_H

#include <opencv2/core/core.hpp>
#include <math.h>

#include "BoxSizes.hpp"

#define PI 3.1415926535897932

const char videoFile[] = "../video/test.MOV";

// all distances are expressed in millimeters

//const Mat T = (Mat_<double>(1,3) << -15, 120, 352);
const Mat T = (Mat_<double>(1,3) << -15, 120, 352);

const double alpha = PI/2 - acos(100/a);

const Mat rotateMatrix = (Mat_<double>(3,3) <<  cos(alpha), 0, sin(alpha),
											    0,          1, 0,
										       -sin(alpha), 0, cos(alpha));

const Mat p[8] = {
	(Mat_<double>(1,3) << 0, 0, 0)*rotateMatrix,		// bottom anterior point on the left side
	(Mat_<double>(1,3) << b, 0, 0)*rotateMatrix,		// bottom anterior point on the right side
	(Mat_<double>(1,3) << b, 0, a)*rotateMatrix,		// bottom rear point on the right side
	(Mat_<double>(1,3) << 0, 0, a)*rotateMatrix,		// bottom rear point in the left side

	(Mat_<double>(1,3) << 0, -c, 0)*rotateMatrix,		// top anterior point on the left side
	(Mat_<double>(1,3) << b, -c, 0)*rotateMatrix,		// top anterior point on the right side
	(Mat_<double>(1,3) << b, -c, a)*rotateMatrix,		// top rear point on the right side
	(Mat_<double>(1,3) << 0, -c, a)*rotateMatrix		// top rear point in the left side
};

#endif
