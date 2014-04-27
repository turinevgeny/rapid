#include "RAPIDTracker.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTracker::RAPIDTracker(Model& _model)
    :Tracker(_model)
{}

Model RAPIDTracker::ProcessFrame(const Mat& frame)
{
	Mat result = frame.clone();
	Mat edges = ExtractEdges(result);

	Point2d foundPoint,foundPoint2;

	std::list<Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<Mat>::iterator companionPointsIter = model.companionPoints.begin();

	std::vector<Point2d> foundBoxPoints2D;
    std::vector<Point3d> modelPoints3D;

	while (controlPointsIter != model.controlPoints.end())
	{
		Point2d r = model.Project(*controlPointsIter);
		Point2d s = model.Project(*companionPointsIter);
        if (FindPoints(r, s, edges, foundPoint, foundPoint2))
        {
            foundBoxPoints2D.push_back(foundPoint);

		    circle(result, foundPoint, 4, Scalar(0,0,255));
            circle(result, foundPoint2, 4, Scalar(255,0,255));
		    line(result, foundPoint, r, Scalar(0,255,0), 1, 8);

		    double Px = (*controlPointsIter).at<double>(0,0);
		    double Py = (*controlPointsIter).at<double>(0,1);
		    double Pz = (*controlPointsIter).at<double>(0,2);

            modelPoints3D.push_back(Point3f(Px, Py, Pz));
        }
        else
        {
            circle(result, foundPoint, 1, Scalar(255,0,255));
            circle(result, foundPoint2, 1, Scalar(255,0,255));
		    line(result, foundPoint, foundPoint2, Scalar(0,255,0), 1, 8);
            line(result, foundPoint, r, Scalar(255,0,0), 1, 8);
        }

		controlPointsIter++;
		companionPointsIter++;
	}

	Mat rvec,tvec;
    solvePnP(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
                 model.distortionCoefficients, rvec, tvec, false);
    //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
    cout << "---(SolvePnP) delta rotate vector" << endl << rvec - model.rotationVector<< endl;
    cout << "---(SolvePnP) delta translate vector=" << endl << tvec - model.translateVector << endl << endl;

    model.updatePose(rvec - model.rotationVector, tvec - model.translateVector);

	namedWindow("Current: foundPoints", CV_WINDOW_AUTOSIZE);
    imshow("Current: foundPoints", result);

 	return model;
}