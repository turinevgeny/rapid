#ifndef __MODEL_H
#define __MODEL_H

#include <list>

#include <opencv2/core/core.hpp>

class Model
{
public:
	Model() {}
	Model(const cv::Mat* cornerPoints,
          const int      pointsPerEdge,
          const cv::Mat& cameraMatrix,
          const cv::Mat& distortionCoefficients,
          const cv::Mat& rotationVector,
          const cv::Mat& translateVector);
	Model(const std::vector<cv::Mat> cornerPoints,
          const int      pointsPerEdge,
          const cv::Mat& cameraMatrix,
          const cv::Mat& distortionCoefficients,
          const cv::Mat& rotationVector,
          const cv::Mat& translateVector);
    Model(const Model& model);
    //Model& operator= (const Model& other);
	~Model();
    void            DrawReferencePoints(const cv::Mat&   source, 
                                        cv::Mat& patternOrigin3D, 
                                        int numFrame,
                                        int numIter); // to draw the origin of model and the origin of pattern
	cv::Mat         Outline(const cv::Mat&   source,
                            const bool       isDrawControlPoints = true,
                            const cv::Scalar color = (cv::Scalar::all(255)),
                            const bool       isDrawCompanionPoints = false);	      // projects the model onto the image
	void			updatePose(const cv::Mat& rotVector, const cv::Mat& transVector); // 3d vectors
	void			updatePose(const cv::Mat& solution);	// solution = rotation angles concat translation distances
private:
	std::vector<cv::Mat>	cornerPoints;		// corner points in model coords
	cv::Mat					cameraMatrix, distortionCoefficients;
	std::list<cv::Mat>		controlPoints;		// list of control points
	std::list<cv::Mat>		companionPoints;	// list of companion points correspond to control points determining the control edges.
	int						pointsPerEdge;
	cv::Mat					translateVector;	// is used for projection purposes
	cv::Mat					rotationVector;		// is used for projection purposes
protected:
	cv::Point2d     Project(const cv::Mat& Point3d) const;
    cv::Point2d     ManualProject(const cv::Mat& Point3d) const;
	virtual void	SetControlPoints();						// fills control points list with points evenly located on the edges
	void            AddControlPointsFromTheEdge(int i, int j);

	friend class RAPIDTracker;
};

#endif