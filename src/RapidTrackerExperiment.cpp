#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include "RAPIDTrackerExperiment.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTrackerExperiment::RAPIDTrackerExperiment(Model& _model)
	:RAPIDTracker(_model)
{}

RAPIDTrackerExperiment::~RAPIDTrackerExperiment()
{}

void RAPIDTrackerExperiment::getSubVectors(
	const std::vector<Point3f> modelPoints3D, 
	const std::vector<Point2f> foundBoxPoints2D, 
	const std::vector<unsigned> subset, 
	std::vector<Point3f> &out_subModelPoints3D,
	std::vector<Point2f> &out_subFoundBoxPoints2D) const
{
	for (int i = 0; i < subset.size(); i++)
	{
		out_subModelPoints3D.push_back(modelPoints3D[subset[i]]);
		out_subFoundBoxPoints2D.push_back(foundBoxPoints2D[subset[i]]);
	}
}

void RAPIDTrackerExperiment::RunSolvePnP(
	const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
	Mat& out_rvec,
	Mat& out_tvec) const
{
	int n = model.controlPoints.size();
	int k = n / 2;
	util::RandomGenerator rng;

    std::ofstream file;
    file.open ("../others/matlab_workspace/rvec_and_tvec.txt");

	for (int i = 0; i < n; i++)
	{
		std::vector<unsigned> subset;
		rng.drawUniformSubset(n, k, subset);

		std::vector<Point3f> subModelPoints3D;
		std::vector<Point2f> subFoundBoxPoints2D;

		getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

		solvePnP(Mat(subModelPoints3D), Mat(subFoundBoxPoints2D), model.cameraMatrix,
			model.distortionCoefficients, out_rvec, out_tvec, false);
		//cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
		Mat delta_rvec = abs(out_rvec - model.rotationVector);
		Mat delta_tvec = abs(out_tvec - model.translateVector);
		cout << "---(SolvePnP) delta rotate vector" << endl << delta_rvec<< endl;
		cout << "---(SolvePnP) delta translate vector=" << endl << delta_tvec << endl << endl;

        for(int i=0; i<3; i++)
	        file << delta_rvec.at<double>(i, 0) << ", ";
        for(int i=0; i<2; i++)
    	    file << delta_tvec.at<double>(i, 0) << ", ";
    	file << delta_tvec.at<double>(2, 0) << endl;
		
	}

    file.close();
}
