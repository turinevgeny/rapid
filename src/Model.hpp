#ifndef __MODEL_H
#define __MODEL_H

#include <opencv2/core/core.hpp>
#include <list>

class Model
{
private:
	cv::Mat					T;					// model coordinate system origin in camera coords
	std::vector<cv::Mat>	cornerPoints;		// corner points in model coords
	cv::Mat					cameraMatrix, distortionCoefficients;
	std::list<cv::Mat>		controlPoints;		// list of control points
	int						pointsPerEdge;
public:
					Model(const cv::Mat &T, const cv::Mat *cornerPoints, int pointsPerEdge, const cv::Mat &cameraMatrix, const cv::Mat &distortionCoefficients);
					Model(const cv::Mat &T, const std::vector<cv::Mat> cornerPoints, int pointsPerEdge, const cv::Mat &cameraMatrix, const cv::Mat &distortionCoefficients);
					~Model();
	cv::Mat			Outline(const cv::Mat &source);			// projects the model onto the image
private:
	cv::Point2d		Project(const cv::Mat &_3DPoint, const cv::Mat &rotationVector, const cv::Mat &translateVector);
	cv::Point2d		Project(const cv::Mat &_3DPoint);
	cv::Point2d		Project(const cv::Mat &_3DPoint, double scaleCoeff, const cv::Point2d &translateVector);
	void			SetControlPoints();						// fills control points list with points evenly located on the edges
	void            AddControlPointsFromTheEdge(int i, int j);
};

#endif