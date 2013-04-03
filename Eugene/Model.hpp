#ifndef __MODEL_H
#define __MODEL_H

#include <opencv2/core/core.hpp>
#include <list>
using namespace std;
using namespace cv;

class Model
{
private:
	Mat				T;					// model coordinate system origin in camera coords
	Mat				cornerPoints[8];	// corner points in model coords
	double			scaleCoeff;			// for explicit projecting. Obsolete.
	Mat				Camera_Matrix, Distortion_Coefficients;
	list<Mat*>		controlPoints;		// list of control points
	int				pointsPerEdge;
public:
					Model(const Mat &T, const Mat *cornerPoints, double scaleCoeff);
					Model(const Mat &T, const Mat *cornerPoints, const Mat &Camera_Matrix, const Mat &Distortion_Coefficients);
					Model(const Mat &T, const Mat *cornerPoints, int pointsPerEdge, const Mat &Camera_Matrix, const Mat &Distortion_Coefficients);
					~Model();
	Mat				Outline(const Mat &source);				// projects the model onto the image
private:
	Point2d			Project(const Mat &_3DPoint, const Mat &rotationVector, const Mat &translateVector);
	Point2d			Project(const Mat &_3DPoint);
	Point2d			Project(const Mat &_3DPoint, double scaleCoeff, const Point2d &translateVector);
	void			SetControlPoints();						// fills control points list with points evenly located on the edges
	void            AddControlPointsFromTheEdge(int i, int j);
};

#endif