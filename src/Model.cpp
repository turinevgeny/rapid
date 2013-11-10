#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Model.hpp"

using std::endl;
using std::cout;
using namespace cv;

Model::Model(const Mat* _cornerPoints,
             const int      _pointsPerEdge,
             const Mat& _cameraMatrix,
             const Mat& _distortionCoefficients,
             const Mat& _rotationVector,
             const Mat& _translateVector)
{
	for(int i = 0; i < 8; i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge   = _pointsPerEdge;
    rotationVector  = _rotationVector;
    translateVector = _translateVector;

	SetControlPoints();
}

Model::Model(const std::vector<Mat> _cornerPoints,
             const int      _pointsPerEdge,
             const Mat& _cameraMatrix,
             const Mat& _distortionCoefficients,
             const Mat& _rotationVector,
             const Mat& _translateVector)
{
	for(unsigned int i = 0; i < _cornerPoints.size(); i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge   = _pointsPerEdge;
    rotationVector  = _rotationVector;
    translateVector = _translateVector;

	SetControlPoints();
}

Model::Model(const Model& model)
{
    {
        std::vector<Mat>::const_iterator cornerPointsIterSrc = model.cornerPoints.begin();
        while (cornerPointsIterSrc != model.cornerPoints.end())
        {
            this->cornerPoints.push_back((*cornerPointsIterSrc).clone());
	        cornerPointsIterSrc++;
        }
    }

    {
        std::list<Mat>::const_iterator controlPointsIterSrc = model.controlPoints.begin();
        while (controlPointsIterSrc != model.controlPoints.end())
        {
            this->controlPoints.push_back((*controlPointsIterSrc).clone());
	        controlPointsIterSrc++;
        }
    }

    {
        std::list<Mat>::const_iterator companionPointsIterSrc = model.companionPoints.begin();
        while (companionPointsIterSrc != model.companionPoints.end())
        {
            this->companionPoints.push_back((*companionPointsIterSrc).clone());
	        companionPointsIterSrc++;
        }
    }

    this->translateVector = model.translateVector.clone();
    this->rotationVector = model.rotationVector.clone();
    this->cameraMatrix = model.cameraMatrix.clone();
    this->distortionCoefficients = model.distortionCoefficients.clone();
    this->pointsPerEdge = model.pointsPerEdge;
}

Model::~Model()
{
	std::list<Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		controlPointsIter->release();
		controlPointsIter++;
	}

	std::list<Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		companionPointsIter->release();
		companionPointsIter++;
	}

	std::vector<Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		cornerPointsIter->release();
		cornerPointsIter++;
	}
}

// Manual projecting based on camera calibrating data.
Point2d Model::ManualProject(const Mat& _3DPoint) const
{
    Mat Box3DPoint = _3DPoint.clone();
    Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);

    Box3DPoint = rotationMatrix * Box3DPoint.t() + translateVector;

    double x = Box3DPoint.at<double>(0,0) / Box3DPoint.at<double>(2,0);
	double y = Box3DPoint.at<double>(1,0) / Box3DPoint.at<double>(2,0);

	double rQuad = x*x+y*y;

	double x1 = x*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);
	double y1 = y*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);

	double u = x1*cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
	double v = y1*cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);

	return Point2d(u,v);
}

// Projecting using OpenCV function
Point2d Model::Project(const Mat& _3DPoint) const
{
	std::vector<Point2d> projectedPoints;
	projectPoints(_3DPoint, rotationVector, translateVector, cameraMatrix, distortionCoefficients, projectedPoints);
    return projectedPoints[0];
}

void Model::DrawReferencePoints(const Mat& source, Mat& patternOrigin3D, int numFrame)
{
    Mat view = source.clone();

    std::stringstream ss;
    std::string text;
    ss << numFrame;
    text = ss.str();

    putText(view, text, cvPoint(70,30), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0,255,0), 3);

    Mat boxOrigibPoint3D = Mat::zeros(3, 1, CV_64F);
    Mat boxOrigibPoint2D = Mat::zeros(2, 1, CV_64F);
    Mat patternOrigin2D = Mat::zeros(2, 1, CV_64F);

    projectPoints(boxOrigibPoint3D.t(), rotationVector, translateVector, cameraMatrix, distortionCoefficients, boxOrigibPoint2D);
    projectPoints(patternOrigin3D.t(), rotationVector, translateVector, cameraMatrix, distortionCoefficients, patternOrigin2D);

    Point2d boxCenter(boxOrigibPoint2D.at<double>(0,0), boxOrigibPoint2D.at<double>(0,1));
    Point2d patternCenter(patternOrigin2D.at<double>(0,0), patternOrigin2D.at<double>(0,1));

    //draw box's reference point
    circle(view, boxCenter, 2, Scalar(0,0,255), 2); //red
    //draw pattern's reference point
    circle(view, patternCenter, 2, Scalar(0,255,0), 2); //green

    namedWindow("DrawReferencePoints", CV_WINDOW_AUTOSIZE);
    imshow("DrawReferencePoints", view);

    cout << "Coordinates of box's reference point in camera coordinates" << endl << translateVector << endl; 
}

Mat Model::Outline(const Mat&   source,
                       const bool       isDrawControlPoints,
                       const Scalar color,
                       const bool       isDrawCompanionPoints)
{
	Mat result = source.clone();

	// drawing edges
	line(result, Project(cornerPoints[0]), Project(cornerPoints[1]), color, 1, 8);
	line(result, Project(cornerPoints[1]), Project(cornerPoints[2]), color, 1, 8);
 	line(result, Project(cornerPoints[2]), Project(cornerPoints[3]), color, 1, 8);
	line(result, Project(cornerPoints[0]), Project(cornerPoints[4]), color, 1, 8);
 
 	line(result, Project(cornerPoints[4]), Project(cornerPoints[5]), color, 1, 8);
 	line(result, Project(cornerPoints[5]), Project(cornerPoints[6]), color, 1, 8);
	line(result, Project(cornerPoints[6]), Project(cornerPoints[7]), color, 1, 8);
	line(result, Project(cornerPoints[7]), Project(cornerPoints[4]), color, 1, 8);

 	line(result, Project(cornerPoints[0]), Project(cornerPoints[3]), color, 1, 8);
	line(result, Project(cornerPoints[1]), Project(cornerPoints[5]), color, 1, 8);
	line(result, Project(cornerPoints[2]), Project(cornerPoints[6]), color, 1, 8);
	line(result, Project(cornerPoints[3]), Project(cornerPoints[7]), color, 1, 8);

    
    if (isDrawControlPoints)
    {
	    std::list<Mat>::iterator controlPointsIter = controlPoints.begin();
	    while (controlPointsIter != controlPoints.end())
	    {
		    circle(result, Project(*controlPointsIter), 4, color);
		    controlPointsIter++;
	    }
    }

    if (isDrawCompanionPoints)
	{
 	    std::list<Mat>::iterator companionPointsIter = companionPoints.begin();
 	    while (companionPointsIter != companionPoints.end())
 	    {
 		    circle(result, Project(*companionPointsIter), 5, Scalar(0,255,0) );
 		    companionPointsIter++;
 	    }
    }

	return result;
}

// adds control points located on i-j edge
void Model::AddControlPointsFromTheEdge(int i, int j)
{
    const double offset = 0.3;
	const double companionPointsOffset = 0.3;

	const Mat direction = cornerPoints[j] - cornerPoints[i];

    Mat p, s;

    p = cornerPoints[i] + direction*offset;
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] + direction*companionPointsOffset*2;
	companionPoints.push_back(s);
	s.release();
	
    p = cornerPoints[i] + direction*(1 - offset);
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] + direction*(1-companionPointsOffset*2);
	companionPoints.push_back(s);
	s.release();

    for (int k = 1; k < pointsPerEdge-1; k++)
	{
		p = cornerPoints[i] + direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		p.release();

		s = cornerPoints[i] + direction*(k / (double) (pointsPerEdge-1) + companionPointsOffset);
		companionPoints.push_back(s);
		s.release();
	}
}

void Model::SetControlPoints()
{
	// pointsPerEdge control point correspond to every edge
	//AddControlPointsFromTheEdge(1, 2);
	AddControlPointsFromTheEdge(0, 1);
 	//AddControlPointsFromTheEdge(2, 3);
	AddControlPointsFromTheEdge(0, 4);

	//AddControlPointsFromTheEdge(4, 5);
	AddControlPointsFromTheEdge(5, 6);
	//AddControlPointsFromTheEdge(6, 7);
	AddControlPointsFromTheEdge(7, 4);

	AddControlPointsFromTheEdge(0, 3);
	AddControlPointsFromTheEdge(1, 5);
	//AddControlPointsFromTheEdge(2, 6);
	AddControlPointsFromTheEdge(3, 7);
}

void Model::updatePose(const Mat &solution)
{
	Mat angle = Mat(solution, Range(0,3), Range(0,1));
	Mat distance = Mat(solution, Range(3,6), Range(0,1));

	updatePose(angle, distance);
}

void Model::updatePose(const Mat& angle, const Mat& distance)
{
    translateVector += distance; 
    rotationVector += angle;     
}