#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "RAPIDTracker.hpp"

#define ENABLE_TESTING

RAPIDTracker::RAPIDTracker(const Model& _model)
{
	model = _model;
}

cv::Mat RAPIDTracker::ExtractEdges(const cv::Mat& image) const
{
	cv::Mat edges;
	cv::cvtColor(image, edges, CV_BGR2GRAY); //TODO: Is it necessary for RapidTesting?

#ifndef ENABLE_TESTING
	cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
	cv::Canny(edges, edges, 20, 100, 3);
#endif

	return edges;
}

bool RAPIDTracker::GetDisplacement(cv::Point2d controlPoint, cv::Point2d companionPoint, const cv::Mat& edges, cv::Point2d& foundPoint, cv::Point2d& foundPoint2, double& length)
{
	double kx=1/model.cameraMatrix.at<double>(0,0);
	double ky=1/model.cameraMatrix.at<double>(1,1);

	double beta = ky/kx;

	double tempX=(companionPoint.x-controlPoint.x);
	double tempY=(companionPoint.y-controlPoint.y);

//	if (tempY < 0)
//	{
//		tempX *= -1;
//		tempY *= -1;
//	}

	double sineAlpha    = tempY/sqrt(tempX*tempX + tempY*tempY);
	double cosineAlpha  = tempX/sqrt(tempX*tempX + tempY*tempY);
	double tangentAlpha = sineAlpha/cosineAlpha;

	double top = tan(CV_PI/4 + beta/2);
	double bottom = tan(beta/2);

	Direction foundDirection;
	int dx1,dy1,dx2,dy2;

	if( (tangentAlpha > (-1)*top) && (tangentAlpha < (-1)*bottom) )
	{
		foundDirection=DOWNWARD_DIAGONAL;
		dx1=-1; dy1= 1;
		dx2= 1; dy2=-1;
	}
	if( (tangentAlpha < top) && (tangentAlpha > bottom) )
	{
		foundDirection=UPWARD_DIAGONAL;
		dx1=-1; dy1=-1;
		dx2= 1; dy2= 1;
	}
	if( (tangentAlpha >= top) || (tangentAlpha <= (-1)*top) )
	{
		foundDirection=HORIZONTAL;
		dx1= 1; dy1=0;
		dx2=-1; dy2=0;
	}
	if( (tangentAlpha <= bottom) && (tangentAlpha >= (-1)*bottom) )
	{
		foundDirection=VERTICAL;
		dx1=0; dy1= 1;
		dx2=0; dy2=-1;
	}

	// findImageEdge

	int num=0;
	int diff1=0,diff2=0;
	double currX1=controlPoint.x;
	double currY1=controlPoint.y;
	double currX2=currX1;
	double currY2=currY1;

	int cols=edges.cols;
	int rows=edges.rows;

 	diff2=diff1=edges.at<uchar>(currY1,currX1);

	while( (currX1<cols)&&(currX2<cols)&&(currY1<rows)&&(currY2<rows)&&
		   (currX1>0)&&(currX2>0)&&(currY1>0)&&(currY2>0)&& 
		   (diff1!=255) && (diff2!=255) )
	{
        currX1+=dx1;
		currY1+=dy1;
		currX2+=dx2;
		currY2+=dy2;
		diff1=edges.at<uchar>(currY1,currX1);
		diff2=edges.at<uchar>(currY2,currX2);
        num++;
	}

	if (diff1==255) 
    {
		foundPoint = cv::Point2d(currX1, currY1);
         //foundPoint2 = cv::Point2d(currX2, currY2);
        if ((diff2==255) && (num == 0))
        { 
            std::cout<<"Warning: The found point and the control point are the same! Control point: ( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<std::endl;
        }
        if ((diff2==255) && (num > 0))
        {
            foundPoint2 = cv::Point2d(currX2, currY2);
            std::cout<<"Warning: Found two points! Control point: ( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<std::endl;
            std::cout<<"Warning: First 2D point:  ( "<<foundPoint.x<<" : "<<foundPoint.y<<" )"<<std::endl;
            std::cout<<"Warning: Second 2D point:  ( "<<foundPoint2.x<<" : "<<foundPoint2.y<<" )"<<std::endl;
        }
    }
	else 
    {
        if(diff2==255)
        {
		    foundPoint = cv::Point2d(currX2, currY2);
            //foundPoint2 = cv::Point2d(currX1, currY1);
        }
        else
        {
            foundPoint = cv::Point2d(currX1, currY1);
            foundPoint2 = cv::Point2d(currX2, currY2);
            std::cout<<"Warning: Point isn't found for control point( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<std::endl;
            std::cout<<"Warning: First 2D point:  ( "<<foundPoint.x<<" : "<<foundPoint.y<<" )"<<std::endl;
            std::cout<<"Warning: Second 2D point:  ( "<<foundPoint2.x<<" : "<<foundPoint2.y<<" )"<<std::endl;
            return false;
        }
    }

    //std::cout<<"controlPoint  "<<controlPoint.x<<" : "<<controlPoint.y<<endl;
    //std::cout<<"foundPoint  "<<foundPoint.x<<" : "<<foundPoint.y<<endl;

    //Displays how many iterations are searched foundPoints
	//std::cout<<"num  "<<num<<std::endl;

    //std::cout<<"foundDirection  "<<foundDirection<<std::endl;

	switch(foundDirection)
	{
		case DOWNWARD_DIAGONAL:
			length = num*(ky*cosineAlpha+kx*sineAlpha);
			break;
		case UPWARD_DIAGONAL:
			length = num*(ky*cosineAlpha-kx*sineAlpha);
			break;
		case VERTICAL:
			length = num*ky*cosineAlpha;
			break;
		case HORIZONTAL:
			length = -num*kx*sineAlpha;
			break;
	}
	return true;
}

//Model RAPIDTracker::ProcessFrame(const cv::Mat& frame)
Model RAPIDTracker::ProcessFrame(const cv::Mat& frame)
{
	cv::Mat result=frame.clone();

	std::list<cv::Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<cv::Mat>::iterator companionPointsIter = model.companionPoints.begin();

	cv::Mat edges = ExtractEdges(result); 

#ifndef ENABLE_TESTING
	cv::namedWindow("canny", CV_WINDOW_AUTOSIZE);
	imshow("canny",edges);
#endif

	cv::Point2d foundPoint,foundPoint2;
	double l;
	double tempX,tempY;
	double sineAlpha,cosineAlpha;

	cv::Mat a,b,c;
	cv::Mat right = cv::Mat::zeros(6, 1, CV_64F);
	cv::Mat left  = cv::Mat::zeros(6, 6, CV_64F);

    std::vector<cv::Point2d> foundBoxPoints2D;
    std::vector<cv::Point3d> modelPoints3D;

	while (controlPointsIter != model.controlPoints.end())
	{
		cv::Point2d r = model.Project(/*model.T+*/*controlPointsIter);
		cv::Point2d s = model.Project(/*model.T+*/*companionPointsIter);

        if (GetDisplacement(r,s,edges,foundPoint,foundPoint2,l))
        {
            foundBoxPoints2D.push_back(foundPoint);
		    std::cout << "length:" << l << std::endl;

		    cv::circle(result, foundPoint, 4, cv::Scalar(0,0,255));
            cv::circle(result, foundPoint2, 4, cv::Scalar(255,0,255));
		    cv::line(result, foundPoint, r, cv::Scalar(0,255,0), 1, 8);

		    tempX=(s.x-r.x);
		    tempY=(s.y-r.y);
		    sineAlpha    = tempY/sqrt(tempX*tempX + tempY*tempY);
		    cosineAlpha  = tempX/sqrt(tempX*tempX + tempY*tempY);

		    double x=r.x;
		    double y=r.y;
		    double Px=(*controlPointsIter).at<double>(0,0);
		    double Py=(*controlPointsIter).at<double>(0,1);
		    double Pz=(*controlPointsIter).at<double>(0,2);

            modelPoints3D.push_back(cv::Point3f(Px, Py, Pz));

		    double Tz=model.T.at<double>(0,2);

		    a = (cv::Mat_<double>(6,1) <<  -x*Py, x*Px+Pz, -Py, 1, 0,-x);
		    b = (cv::Mat_<double>(6,1) <<  -y*Py-Pz, y*Px,  Px, 0, 1,-y);
		    a/=Tz+Pz;
		    b/=Tz+Pz;
		    c=a*cosineAlpha-b*sineAlpha;

		    left += c*c.t();
		    right += c*l;	// why does it move so strange ?????
		    //right-=c*abs(l);// why doesn't it move ?????????????
        }
        else 
        {
            cv::circle(result, foundPoint, 1, cv::Scalar(255,0,255));
            cv::circle(result, foundPoint2, 1, cv::Scalar(255,0,255));
		    cv::line(result, foundPoint, foundPoint2, cv::Scalar(0,255,0), 1, 8);
            cv::line(result, foundPoint, r, cv::Scalar(255,0,0), 1, 8);
        }

		controlPointsIter++;
		companionPointsIter++;
	}
	cv::Mat solution;
	cv::solve(left,right,solution);
    std::cout << std::endl << "Algoritm solution " << solution << std::endl;

    cv::Mat rvec,tvec;
    cv::solvePnP(cv::Mat(modelPoints3D), cv::Mat(foundBoxPoints2D), model.cameraMatrix,
        model.distortionCoefficients, rvec, tvec, false);
    std::cout << "Rotate vector" << std::endl << rvec << std::endl << "Translate vector=" << std::endl << tvec << std::endl;

	//std::cout << std::endl << "right " << right << std::endl << "; left "<< left*solution << std::endl

	model.updatePose(rvec,tvec);

	cv::namedWindow("Current: foundPoints", CV_WINDOW_AUTOSIZE);
	cv::imshow("Current: foundPoints", result);

 	return model;
}

