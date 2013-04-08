#include "RAPIDTracker.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <cmath>
using namespace cv;

RAPIDTracker::RAPIDTracker(const std::string _videoFile, const Model &_model)
{
	videoFile = _videoFile;
	model = _model;
}

Mat RAPIDTracker::ExtractEdges(const Mat &image) const
{
	Mat frame;
	Mat edges;
	cvtColor(frame, edges, CV_BGR2GRAY);
	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	Canny(edges, edges, 20, 100, 3);
	return edges;
}

Direction RAPIDTracker::FindSearchDirection(Point2d controlPoint, Point2d companionPoint) const
{
	double beta = model.cameraMatrix.at<double>(0,0) / model.cameraMatrix.at<double>(1,1);
	double tangentAlpha = (companionPoint.y-controlPoint.y)/(companionPoint.x-controlPoint.x);

	double top = tan(CV_PI/4 + beta/2);
	double bottom = tan(beta/2);

	if( (tangentAlpha > (-1)*top) || (tangentAlpha < (-1)*bottom) )
		return DOWNLOAD_DIAGONAL;
	if( (tangentAlpha < top) || (tangentAlpha > bottom) )
		return UPWARD_DIAGONAL;
	if( (tangentAlpha >= top) || (tangentAlpha <= (-1)*top) )
		return HORIZONTAL;
	if( (tangentAlpha <= bottom) || (tangentAlpha >= (-1)*bottom) )
		return VERTICAL;
}

Model RAPIDTracker::ProcessFrame(const Mat &frame)
{
	std::list<Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<Mat>::iterator companionPointsIter = model.companionPoints.begin();

	while (controlPointsIter != model.controlPoints.end())
	{
		Point2d r = model.Project(*controlPointsIter);
		Point2d s = model.Project(*companionPointsIter);

		Direction direction = FindSearchDirection(r, s);

		Mat edges = ExtractEdges(frame);

		//Point2d imageEdgePoint = FindImageEdge(edges, r, direction);

		controlPointsIter++;
		companionPointsIter++;
	}
}