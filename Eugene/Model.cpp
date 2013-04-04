#include "Model.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
using namespace cv;
using namespace std;

Model :: Model(const Mat &_T, const Mat *_p, double _scaleCoeff)
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints[i] = _p[i].clone();

	scaleCoeff = _scaleCoeff;
}

Model :: Model(const Mat &_T, const Mat *_cornerPoints, const Mat &_Camera_Matrix, const Mat &_Distortion_Coefficients)
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints[i] = _cornerPoints[i].clone();

	Camera_Matrix = _Camera_Matrix.clone();
	Distortion_Coefficients = _Distortion_Coefficients.clone();
}

Model :: Model(const Mat &_T,
			   const Mat *_cornerPoints,
			   int        _pointsPerEdge,
			   const Mat &_Camera_Matrix,
			   const Mat &_Distortion_Coefficients)
	
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints[i] = _cornerPoints[i].clone();

	Camera_Matrix = _Camera_Matrix.clone();
	Distortion_Coefficients = _Distortion_Coefficients.clone();

	pointsPerEdge = _pointsPerEdge;
	SetControlPoints();
}

Model :: ~Model()
{

}

// Projecting points manually. Parameters selection based on luck and attentivness.
Point2d Model :: Project(const Mat& _3DPoint, double scaleCoeff, const Point2d &translateVector)
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
Point2d Model :: Project(const Mat& _3DPoint)
{
	double x = _3DPoint.at<double>(0,0) / _3DPoint.at<double>(0,2);
	double y = _3DPoint.at<double>(0,1) / _3DPoint.at<double>(0,2);

	double rQuad = x*x+y*y;

	double x1 = x*(1+Distortion_Coefficients.at<double>(0,0)*rQuad+Distortion_Coefficients.at<double>(1,0)*rQuad*rQuad + Distortion_Coefficients.at<double>(4,0)*rQuad*rQuad*rQuad);
	double y1 = y*(1+Distortion_Coefficients.at<double>(0,0)*rQuad+Distortion_Coefficients.at<double>(1,0)*rQuad*rQuad + Distortion_Coefficients.at<double>(4,0)*rQuad*rQuad*rQuad);

	double u = x1*Camera_Matrix.at<double>(0,0) + Camera_Matrix.at<double>(0,2);
	double v = y1*Camera_Matrix.at<double>(1,1) + Camera_Matrix.at<double>(1,2);

	return Point2d(u,v);
}

// Projecting using OpenCV function
Point2d Model :: Project(const Mat& _3DPoint, const Mat &_rotationVector, const Mat &_translateVector)
{
	Mat projectedPoint;

	projectPoints(_3DPoint, _rotationVector, _translateVector, Camera_Matrix, Distortion_Coefficients, projectedPoint);

    Mat_<Vec<float, 2> > I = projectedPoint;

	return Point2f(I(0,0)[0], I(0,0)[1]);
}

Mat Model :: Outline(const Mat &source)
{
	Mat result = source.clone();

	CvScalar whiteColor = CV_RGB(255, 255, 255);

	/*
	Point2d transpose(source.cols / 2, source.rows / 2);

	line(result, Project(T+p[0], scaleCoeff, transpose), Project(T+p[1], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[1], scaleCoeff, transpose), Project(T+p[2], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[0], scaleCoeff, transpose), Project(T+p[4], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[2], scaleCoeff, transpose), Project(T+p[3], scaleCoeff, transpose), whiteColor, 2, 8);

	line(result, Project(T+p[4], scaleCoeff, transpose), Project(T+p[5], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[5], scaleCoeff, transpose), Project(T+p[6], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[6], scaleCoeff, transpose), Project(T+p[7], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[7], scaleCoeff, transpose), Project(T+p[4], scaleCoeff, transpose), whiteColor, 2, 8);

	line(result, Project(T+p[0], scaleCoeff, transpose), Project(T+p[3], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[1], scaleCoeff, transpose), Project(T+p[5], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[2], scaleCoeff, transpose), Project(T+p[6], scaleCoeff, transpose), whiteColor, 2, 8);
	line(result, Project(T+p[3], scaleCoeff, transpose), Project(T+p[7], scaleCoeff, transpose), whiteColor, 2, 8);
	*/

	/*
	Mat projP1, projP2, projP3, projP4, projP5;

	//Mat translateVector = (Mat_<double>(3,1) << -15, 28, 70);
	//Mat translateVector = (Mat_<double>(3,1) << -12, 7, 6);
	Mat translateVector = (Mat_<double>(3,1) << 0, 0, 0);

	projectPoints(T+p[0], Mat(3, 1, CV_32F, Scalar::all(0)), translateVector, Camera_Matrix, Distortion_Coefficients, projP1);
	projectPoints(T+p[1], Mat(3, 1, CV_32F, Scalar::all(0)), translateVector, Camera_Matrix, Distortion_Coefficients, projP2);
	projectPoints(T+p[2], Mat(3, 1, CV_32F, Scalar::all(0)), translateVector, Camera_Matrix, Distortion_Coefficients, projP3);
	projectPoints(T+p[3], Mat(3, 1, CV_32F, Scalar::all(0)), translateVector, Camera_Matrix, Distortion_Coefficients, projP4);
	projectPoints(T+p[4], Mat(3, 1, CV_32F, Scalar::all(0)), translateVector, Camera_Matrix, Distortion_Coefficients, projP5);
	cout << projP1.depth() << " " << projP1.channels() <<  endl;
	cout << projP2.depth() << " " << projP2.channels() <<  endl;
	cout << "projected: " << projP1 << std::endl << projP2 << std::endl;
	*/

	/*
	line(result, Convert2ChannelMatToPoint2f(projP1), Convert2ChannelMatToPoint2f(projP2), CV_RGB(255, 255, 0), 2, 8);
	line(result, Convert2ChannelMatToPoint2f(projP2), Convert2ChannelMatToPoint2f(projP3), CV_RGB(255, 255, 0), 2, 8);
	line(result, Convert2ChannelMatToPoint2f(projP1), Convert2ChannelMatToPoint2f(projP5), CV_RGB(255, 255, 0), 2, 8);
	line(result, Convert2ChannelMatToPoint2f(projP3), Convert2ChannelMatToPoint2f(projP4), CV_RGB(255, 255, 0), 2, 8);

	line(result, Convert2ChannelMatToPoint2f(projP1), Convert2ChannelMatToPoint2f(projP4), CV_RGB(255, 255, 0), 2, 8);
	*/

	Mat translateVector = (Mat_<double>(3,1) << -12, 11, 90);
	Mat rotationVector(3, 1, CV_32F, Scalar::all(0));

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

	/*
	line(result, Project(T+p[0]), Project(T+p[1]), whiteColor, 2, 8);
	line(result, Project(T+p[1]), Project(T+p[2]), whiteColor, 2, 8);
	line(result, Project(T+p[0]), Project(T+p[4]), whiteColor, 2, 8);
	line(result, Project(T+p[2]), Project(T+p[3]), whiteColor, 2, 8);

	line(result, Project(T+p[4]), Project(T+p[5]), whiteColor, 2, 8);
	line(result, Project(T+p[5]), Project(T+p[6]), whiteColor, 2, 8);
	line(result, Project(T+p[6]), Project(T+p[7]), whiteColor, 2, 8);
	line(result, Project(T+p[7]), Project(T+p[4]), whiteColor, 2, 8);

	line(result, Project(T+p[0]), Project(T+p[3]), whiteColor, 2, 8);
	line(result, Project(T+p[1]), Project(T+p[5]), whiteColor, 2, 8);
	line(result, Project(T+p[2]), Project(T+p[6]), whiteColor, 2, 8);
	line(result, Project(T+p[3]), Project(T+p[7]), whiteColor, 2, 8);
	*/

	list<Mat*> :: iterator iter = controlPoints.begin();
	while (iter != controlPoints.end())
	{
		circle(result, Project((**iter), rotationVector, translateVector), 5, CV_RGB(255, 255, 255));
		iter++;

	}

	return result;
}

// adds control points located on i-j edge
void Model :: AddControlPointsFromTheEdge(int i, int j)
{
    const double offset = 0.15;

	Mat direction = cornerPoints[i] - cornerPoints[j];


    Mat *p = new Mat(3, 1, CV_32F);
    *p = T+cornerPoints[i] - direction*offset;
    controlPoints.push_back(p);

    p = new Mat(3, 1, CV_32F);
    *p = T+cornerPoints[i] - direction*(1-offset);
    controlPoints.push_back(p);


    for (int k = 1; k < pointsPerEdge-1; k++)
	{
        p = new Mat(3, 1, CV_32F);
		*p = T+cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		//cout << endl << *p << endl;
	}
}

void Model :: SetControlPoints()
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
