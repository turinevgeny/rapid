#ifndef __MODEL_H
#define __MODEL_H

#include <list>

#include <opencv2/core/core.hpp>

class Model
{
public:
	Model() {}
	Model(const cv::Mat& T,
          const cv::Mat* cornerPoints,
          const int      pointsPerEdge,
          const cv::Mat& cameraMatrix,
          const cv::Mat& distortionCoefficients,
          const cv::Mat& rotationVector,
          const cv::Mat& translateVector);
	Model(const cv::Mat& T,
          const std::vector<cv::Mat> cornerPoints,
          const int      pointsPerEdge,
          const cv::Mat& cameraMatrix,
          const cv::Mat& distortionCoefficients,
          const cv::Mat& rotationVector,
          const cv::Mat& translateVector);
	~Model();
	cv::Mat         Outline(const cv::Mat& source);			// projects the model onto the image
	void			updatePose(const cv::Mat& rotVector, const cv::Mat& transVector); // 3d vectors
	void			updatePose(const cv::Mat& solution);	// solution = rotation angles concat translation distances
private:
	cv::Mat					T;					// model coordinate system origin in camera coords
	std::vector<cv::Mat>	cornerPoints;		// corner points in model coords
	cv::Mat					cameraMatrix, distortionCoefficients;
	std::list<cv::Mat>		controlPoints;		// list of control points
	std::list<cv::Mat>		companionPoints;	// list of companion points correspond to control points determining the control edges.
	int						pointsPerEdge;
	cv::Mat					translateVector;	// is used for projection purposes
	cv::Mat					rotationVector;		// is used for projection purposes
private:
	void		    RotateAndTranslate(const cv::Mat& rotationVector, const cv::Mat& translateVector);
	/*cv::Point2d     Project(const cv::Mat &_3DPoint,
						   const cv::Mat &rotationVector = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0)),
						   //const cv::Mat &translateVector = (cv::Mat_<double>(3,1) << -12, 11, 70)) const;
						   const cv::Mat &translateVector = (cv::Mat_<double>(3,1) << -11, 10, 55)) const;*/
	cv::Point2d     Project(const cv::Mat& Point3d) const;
	cv::Point2d		Project(const cv::Mat& Point3d, 
                            double scaleCoeff, 
                            const cv::Point2d& translateVector) const;
    cv::Point2d     ManualProject(const cv::Mat& Point3d) const;
	void			SetControlPoints();						// fills control points list with points evenly located on the edges
	void            AddControlPointsFromTheEdge(int i, int j);

	friend class RAPIDTracker;
};

#endif