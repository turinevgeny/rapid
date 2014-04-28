#include "RansacTracker.hpp"

using std::cout;
using std::endl;

using namespace cv;

RansacTracker::RansacTracker(
    Model model,
    int iterationsCount,
    float reprojectionError,
    int minInliersCount)
    :   Tracker(model),
        _iterationsCount(iterationsCount),
        _reprojectionError(reprojectionError),
        _minInliersCount(minInliersCount)
{ }

void RansacTracker::RunSolvePnP(
    const std::vector<Point2d> foundBoxPoints2D,
    const std::vector<Point3d> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
    solvePnPRansac(
        Mat(modelPoints3D),
        Mat(foundBoxPoints2D),
        model.cameraMatrix,
        model.distortionCoefficients,
        out_rvec,
        out_tvec,
        false,
        _iterationsCount,
        _reprojectionError,
        _minInliersCount);


    //solvePnP(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,model.distortionCoefficients, out_rvec, out_tvec, false);

    //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
    cout << "---(SolvePnP) delta rotate vector" << endl << out_rvec - model.rotationVector<< endl;
    cout << "---(SolvePnP) delta translate vector=" << endl << out_tvec - model.translateVector << endl << endl;
}