#include "Model.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
using namespace cv;
// using std::cout;
// using std::endl;

Model::Model()
{

}

Model::Model(const Mat &_T, const Mat *_cornerPoints, int _pointsPerEdge, const Mat &_cameraMatrix, const Mat &_distortionCoefficients)
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge = _pointsPerEdge;
	SetControlPoints();
}

Model::Model(const Mat &_T, const std::vector<Mat> _cornerPoints, int _pointsPerEdge, const Mat &_cameraMatrix, const Mat &_distortionCoefficients)
{
	T = _T.clone();

	for(unsigned int i = 0; i < _cornerPoints.size(); i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge = _pointsPerEdge;
	SetControlPoints();
}

Model::~Model()
{
	std::list<Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		controlPointsIter->release();
		controlPointsIter++;
	}

	std::vector<Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		cornerPointsIter->release();
		cornerPointsIter++;
	}
}

// Projecting points manually. Parameters selection based on luck and attentivness.
Point2d Model::Project(const Mat& _3DPoint, double scaleCoeff, const Point2d &translateVector) const
{
	// projecting
	Point2d point(_3DPoint.at<double>(0,0) / (_3DPoint.at<double>(0,2)),
			      _3DPoint.at<double>(0,1) / (_3DPoint.at<double>(0,2)));

	// scaling
	point *= scaleCoeff;

	// moving
	point += translateVector;

	return point;
}

// Manual projecting based on camera calibrating data.
/*
Point2d Model::Project(const Mat& _3DPoint) const
{
	double x = _3DPoint.at<double>(0,0) / _3DPoint.at<double>(0,2);
	double y = _3DPoint.at<double>(0,1) / _3DPoint.at<double>(0,2);

	double rQuad = x*x+y*y;

	double x1 = x*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);
	double y1 = y*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);

	double u = x1*cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
	double v = y1*cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);

	return Point2d(u,v);
}
*/

// Projecting using OpenCV function
Point2d Model::Project(const Mat& _3DPoint, const Mat &_rotationVector, const Mat &_translateVector) const
{
	std::vector<Point2d> projectedPoints;
	projectPoints(_3DPoint, _rotationVector, _translateVector, cameraMatrix, distortionCoefficients, projectedPoints);
    return projectedPoints[0];
}

Mat Model::Outline(const Mat &source)
{
	Mat result = source.clone();

	Scalar whiteColor = Scalar(Scalar::all(255));

	Mat translateVector = (Mat_<double>(3,1) << -12, 11, 90);
	Mat rotationVector(3, 1, CV_32F, Scalar::all(0));

	// drawing edges
	line(result, Project(T+cornerPoints[0]), Project(T+cornerPoints[1]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[1]), Project(T+cornerPoints[2]), whiteColor, 2, 8);
 	line(result, Project(T+cornerPoints[2]), Project(T+cornerPoints[3]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[0]), Project(T+cornerPoints[4]), whiteColor, 2, 8);
 
 	line(result, Project(T+cornerPoints[4]), Project(T+cornerPoints[5]), whiteColor, 2, 8);
 	line(result, Project(T+cornerPoints[5]), Project(T+cornerPoints[6]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[6]), Project(T+cornerPoints[7]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[7]), Project(T+cornerPoints[4]), whiteColor, 2, 8);

 	line(result, Project(T+cornerPoints[0]), Project(T+cornerPoints[3]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[1]), Project(T+cornerPoints[5]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[2]), Project(T+cornerPoints[6]), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[3]), Project(T+cornerPoints[7]), whiteColor, 2, 8);

	// drawing cotrol points
	std::list<Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		circle(result, Project((*controlPointsIter), rotationVector, translateVector), 5, Scalar(Scalar::all(255)));
		controlPointsIter++;
	}

	// drawing companion points
	std::list<Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		circle(result, Project((*companionPointsIter), rotationVector, translateVector), 5, Scalar(0,255,0) );
		companionPointsIter++;
	}

	return result;
}

// adds control points located on i-j edge
void Model::AddControlPointsFromTheEdge(int i, int j)
{
    const double offset = 0.15;

	const double companionPointsOffset = 0.3;

	const Mat direction = cornerPoints[i] - cornerPoints[j];

    Mat p, s;

    p = T + cornerPoints[i] - direction*offset;
    controlPoints.push_back(p);
	p.release();

	s = T + cornerPoints[i] - direction*companionPointsOffset*2;
	companionPoints.push_back(s);
	s.release();
	

    p = T + cornerPoints[i] - direction*(1 - offset);
    controlPoints.push_back(p);
	p.release();

	s = T + cornerPoints[i] - direction*(1-companionPointsOffset*2);
	companionPoints.push_back(s);
	s.release();


    for (int k = 1; k < pointsPerEdge-1; k++)
	{
		p = T+cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		p.release();

		s = T + cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1) + companionPointsOffset);
		companionPoints.push_back(s);
		s.release();
	}
}

void Model::SetControlPoints()
{
	// pointsPerEdge control point correspond to every edge
	AddControlPointsFromTheEdge(1, 2);
	AddControlPointsFromTheEdge(0, 1);
 	AddControlPointsFromTheEdge(2, 3);
	AddControlPointsFromTheEdge(0, 4);

	AddControlPointsFromTheEdge(4, 5);
	AddControlPointsFromTheEdge(5, 6);
	AddControlPointsFromTheEdge(6, 7);
	AddControlPointsFromTheEdge(7, 4);

	AddControlPointsFromTheEdge(0, 3);
	AddControlPointsFromTheEdge(1, 5);
	AddControlPointsFromTheEdge(2, 6);
	AddControlPointsFromTheEdge(3, 7);
}

void Model::RotateAndTranslate(const Mat &rotationVector, const Mat &translateVector)
{

}