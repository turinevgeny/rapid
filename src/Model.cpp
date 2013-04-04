#include "Model.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
using namespace cv;
// using std::cout;
// using std::endl;

Model::Model(const Mat &_T,
			 const Mat *_cornerPoints,
			 int        _pointsPerEdge,
			 const Mat &_cameraMatrix,
			 const Mat &_distortionCoefficients)
	
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints[i] = _cornerPoints[i].clone();

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge = _pointsPerEdge;
	SetControlPoints();
}

Model::~Model()
{
	std::list<Mat>::iterator iter = controlPoints.begin();
	while (iter != controlPoints.end())
	{
		iter->release();
		iter++;
	}
	std::cout << controlPoints.size();
}

// Projecting points manually. Parameters selection based on luck and attentivness.
Point2d Model::Project(const Mat& _3DPoint, double scaleCoeff, const Point2d &translateVector)
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
Point2d Model::Project(const Mat& _3DPoint)
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

// Projecting using OpenCV function
Point2d Model::Project(const Mat& _3DPoint, const Mat &_rotationVector, const Mat &_translateVector)
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
	line(result, Project(T+cornerPoints[0], rotationVector, translateVector), Project(T+cornerPoints[1], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[1], rotationVector, translateVector), Project(T+cornerPoints[2], rotationVector, translateVector), whiteColor, 2, 8);
 	line(result, Project(T+cornerPoints[2], rotationVector, translateVector), Project(T+cornerPoints[3], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[0], rotationVector, translateVector), Project(T+cornerPoints[4], rotationVector, translateVector), whiteColor, 2, 8);
 
 	line(result, Project(T+cornerPoints[4], rotationVector, translateVector), Project(T+cornerPoints[5], rotationVector, translateVector), whiteColor, 2, 8);
 	line(result, Project(T+cornerPoints[5], rotationVector, translateVector), Project(T+cornerPoints[6], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[6], rotationVector, translateVector), Project(T+cornerPoints[7], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[7], rotationVector, translateVector), Project(T+cornerPoints[4], rotationVector, translateVector), whiteColor, 2, 8);

 	line(result, Project(T+cornerPoints[0], rotationVector, translateVector), Project(T+cornerPoints[3], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[1], rotationVector, translateVector), Project(T+cornerPoints[5], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[2], rotationVector, translateVector), Project(T+cornerPoints[6], rotationVector, translateVector), whiteColor, 2, 8);
	line(result, Project(T+cornerPoints[3], rotationVector, translateVector), Project(T+cornerPoints[7], rotationVector, translateVector), whiteColor, 2, 8);

	// drawing cotrol points
	std::list<Mat>::iterator iter = controlPoints.begin();
	while (iter != controlPoints.end())
	{
		circle(result, Project((*iter), rotationVector, translateVector), 5, CV_RGB(255, 255, 255));
		iter++;
	}

	return result;
}

// adds control points located on i-j edge
void Model::AddControlPointsFromTheEdge(int i, int j)
{
    const double offset = 0.15;

	const Mat direction = cornerPoints[i] - cornerPoints[j];

    Mat p;

    p = T+cornerPoints[i] - direction*offset;
    controlPoints.push_back(p);
	p.release();

    p = T+cornerPoints[i] - direction*(1-offset);
    controlPoints.push_back(p);
	p.release();

    for (int k = 1; k < pointsPerEdge-1; k++)
	{
		p = T+cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		p.release();
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
