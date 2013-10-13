#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Model.hpp"

Model::Model(/*const cv::Mat& _T,*/
             const cv::Mat* _cornerPoints,
             const int      _pointsPerEdge,
             const cv::Mat& _cameraMatrix,
             const cv::Mat& _distortionCoefficients,
             const cv::Mat& _rotationVector,
             const cv::Mat& _translateVector)
{
	//T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge   = _pointsPerEdge;
    rotationVector  = _rotationVector;
    translateVector = _translateVector;

	SetControlPoints();
}

Model::Model(/*const cv::Mat& _T,*/
             const std::vector<cv::Mat> _cornerPoints,
             const int      _pointsPerEdge,
             const cv::Mat& _cameraMatrix,
             const cv::Mat& _distortionCoefficients,
             const cv::Mat& _rotationVector,
             const cv::Mat& _translateVector)
{
	//T = _T.clone();

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
        std::vector<cv::Mat>::const_iterator cornerPointsIterSrc = model.cornerPoints.begin();
        while (cornerPointsIterSrc != model.cornerPoints.end())
        {
            this->cornerPoints.push_back((*cornerPointsIterSrc).clone());
	        cornerPointsIterSrc++;
        }
    }

    {
        std::list<cv::Mat>::const_iterator controlPointsIterSrc = model.controlPoints.begin();
        while (controlPointsIterSrc != model.controlPoints.end())
        {
            this->controlPoints.push_back((*controlPointsIterSrc).clone());
	        controlPointsIterSrc++;
        }
    }

    {
        std::list<cv::Mat>::const_iterator companionPointsIterSrc = model.companionPoints.begin();
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
	std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		controlPointsIter->release();
		controlPointsIter++;
	}

	std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		companionPointsIter->release();
		companionPointsIter++;
	}

	std::vector<cv::Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		cornerPointsIter->release();
		cornerPointsIter++;
	}
}

// Projecting points manually. Parameters selection based on luck and attentiveness.
cv::Point2d Model::Project(const cv::Mat& _3DPoint, double scaleCoeff, const cv::Point2d &translateVector) const
{
	// projecting
	cv::Point2d point(_3DPoint.at<double>(0,0) / (_3DPoint.at<double>(0,2)),
			      _3DPoint.at<double>(0,1) / (_3DPoint.at<double>(0,2)));

	// scaling
	point *= scaleCoeff;

	// moving
	point += translateVector;

	return point;
}

// Manual projecting based on camera calibrating data.
cv::Point2d Model::ManualProject(const cv::Mat& _3DPoint) const
{
    cv::Mat Box3DPoint = _3DPoint.clone();
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    Box3DPoint = rotationMatrix * Box3DPoint.t() + translateVector;

    double x = Box3DPoint.at<double>(0,0) / Box3DPoint.at<double>(2,0);
	double y = Box3DPoint.at<double>(1,0) / Box3DPoint.at<double>(2,0);

	double rQuad = x*x+y*y;

	double x1 = x*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);
	double y1 = y*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);

	double u = x1*cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
	double v = y1*cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);

	return cv::Point2d(u,v);
}

// Projecting using OpenCV function
cv::Point2d Model::Project(const cv::Mat& _3DPoint) const
{
	std::vector<cv::Point2d> projectedPoints;
	cv::projectPoints(_3DPoint, rotationVector, translateVector, cameraMatrix, distortionCoefficients, projectedPoints);
    return projectedPoints[0];
}

void Model::DrawReferencePoints(const cv::Mat& source, cv::Mat& patternOrigin3D, int numFrame)
{
    cv::Mat view = source.clone();

    std::stringstream ss;
    std::string text;
    ss << numFrame;
    text = ss.str();

    cv::putText(view, text, cvPoint(70,30), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,255,0), 3);

    cv::Mat boxOrigibPoint3D = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat boxOrigibPoint2D = cv::Mat::zeros(2, 1, CV_64F);
    cv::Mat patternOrigin2D = cv::Mat::zeros(2, 1, CV_64F);

    cv::projectPoints(boxOrigibPoint3D.t(), rotationVector, translateVector, cameraMatrix, distortionCoefficients, boxOrigibPoint2D);
    cv::projectPoints(patternOrigin3D.t(), rotationVector, translateVector, cameraMatrix, distortionCoefficients, patternOrigin2D);

    cv::Point2d boxCenter(boxOrigibPoint2D.at<double>(0,0), boxOrigibPoint2D.at<double>(0,1));
    cv::Point2d patternCenter(patternOrigin2D.at<double>(0,0), patternOrigin2D.at<double>(0,1));

    //draw box's reference point
    cv::circle(view, boxCenter, 2, cv::Scalar(0,0,255), 2); //red
    //draw pattern's reference point
    cv::circle(view, patternCenter, 2, cv::Scalar(0,255,0), 2); //green

    cv::namedWindow("DrawReferencePoints", CV_WINDOW_AUTOSIZE);
    cv::imshow("DrawReferencePoints", view);

    std::cout << "Coordinates of box's reference point in camera coordinates" << std::endl << translateVector << std::endl; 

}

cv::Mat Model::Outline(const cv::Mat&   source,
                       const bool       isDrawControlPoints,
                       const cv::Scalar color,
                       const bool       isDrawCompanionPoints)
{
	cv::Mat result = source.clone();

	// drawing edges
	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[1]), color, 1, 8);
	cv::line(result, Project(cornerPoints[1]), Project(cornerPoints[2]), color, 1, 8);
 	cv::line(result, Project(cornerPoints[2]), Project(cornerPoints[3]), color, 1, 8);
	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[4]), color, 1, 8);
 
 	cv::line(result, Project(cornerPoints[4]), Project(cornerPoints[5]), color, 1, 8);
 	cv::line(result, Project(cornerPoints[5]), Project(cornerPoints[6]), color, 1, 8);
	cv::line(result, Project(cornerPoints[6]), Project(cornerPoints[7]), color, 1, 8);
	cv::line(result, Project(cornerPoints[7]), Project(cornerPoints[4]), color, 1, 8);

 	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[3]), color, 1, 8);
	cv::line(result, Project(cornerPoints[1]), Project(cornerPoints[5]), color, 1, 8);
	cv::line(result, Project(cornerPoints[2]), Project(cornerPoints[6]), color, 1, 8);
	cv::line(result, Project(cornerPoints[3]), Project(cornerPoints[7]), color, 1, 8);

    
    if (isDrawControlPoints)
    {
	    std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
	    while (controlPointsIter != controlPoints.end())
	    {
		    cv::circle(result, Project(*controlPointsIter), 4, color);
		    controlPointsIter++;
	    }
    }

    if (isDrawCompanionPoints)
	{
 	    std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
 	    while (companionPointsIter != companionPoints.end())
 	    {
 		    cv::circle(result, Project(*companionPointsIter), 5, cv::Scalar(0,255,0) );
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

	const cv::Mat direction = cornerPoints[i] - cornerPoints[j];

    cv::Mat p, s;

    p = cornerPoints[i] - direction*offset;
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] - direction*companionPointsOffset*2;
	companionPoints.push_back(s);
	s.release();
	

    p = cornerPoints[i] - direction*(1 - offset);
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] - direction*(1-companionPointsOffset*2);
	companionPoints.push_back(s);
	s.release();

    for (int k = 1; k < pointsPerEdge-1; k++)
	{
		p = cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		p.release();

		s = cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1) + companionPointsOffset);
		companionPoints.push_back(s);
		s.release();
	}
}

void Model::SetControlPoints()
{
	// pointsPerEdge control point correspond to every edge
	//AddControlPointsFromTheEdge(1, 2);
	//AddControlPointsFromTheEdge(0, 1);
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

void Model::updatePose(const cv::Mat &solution)
{
	cv::Mat angle = cv::Mat(solution, cv::Range(0,3), cv::Range(0,1));
	cv::Mat distance = cv::Mat(solution, cv::Range(3,6), cv::Range(0,1));

	updatePose(angle, distance);
}

void Model::updatePose(const cv::Mat& angle, const cv::Mat& distance)
{
	//std::cout<<"--From Update Pose:  T= "<<std::endl<<T<<std::endl;
    //std::cout<<"--From Update Pose:  (model)Translate= "<<std::endl<<translateVector<<std::endl;
    //std::cout<<"--From Update Pose:  (model)Rotate= "<<std::endl<<rotationVector<<std::endl;

	//T+=distance.t();    

    translateVector += distance; 
    rotationVector += angle;     

    // There's no need to move control points here, moving rotate & translate is sufficient

	/*std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
    
	while (controlPointsIter != controlPoints.end())
	{
		std::cout<<"Point= "<<(*controlPointsIter)<<std::endl;
		(*controlPointsIter)+=angle.t().cross(*controlPointsIter) + distance.t();
		//std::cout<<"newPoint= "<<(*controlPointsIter)<<endl;
		controlPointsIter++;
	}

	std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		(*companionPointsIter)+=angle.t().cross(*companionPointsIter) + distance.t();
		companionPointsIter++;
	}

	std::vector<cv::Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		(*cornerPointsIter)+=angle.t().cross(*cornerPointsIter) + distance.t();
		cornerPointsIter++;
	}*/
}