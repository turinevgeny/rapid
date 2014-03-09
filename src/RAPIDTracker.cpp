#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#include "RAPIDTracker.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTracker::RAPIDTracker(Model& _model)
    :model(_model)
{}

Mat RAPIDTracker::ExtractEdges(const Mat& image) const
{
	Mat edges;
	cvtColor(image, edges, CV_BGR2GRAY);

	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	Canny(edges, edges, 20, 100, 3);

	namedWindow("canny", CV_WINDOW_AUTOSIZE);
	imshow("canny",edges);

	return edges;
}

bool RAPIDTracker::FindPoints(Point2d controlPoint,
                              Point2d companionPoint,
                              const Mat& edges,
                              Point2d& foundPoint,
                              Point2d& foundPoint2)
{
	double kx=1/model.cameraMatrix.at<double>(0,0);
	double ky=1/model.cameraMatrix.at<double>(1,1);

	double beta = ky/kx;

	double tempX=(companionPoint.x-controlPoint.x);
	double tempY=(companionPoint.y-controlPoint.y);

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
        dx1=-1; dy1=-1;
		dx2= 1; dy2= 1;

	}
	if( (tangentAlpha < top) && (tangentAlpha > bottom) )
	{
		foundDirection=UPWARD_DIAGONAL;
	    dx1=-1; dy1= 1;
		dx2= 1; dy2=-1;
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
    int diff_add_hr_1=0, diff_add_vr_1=0;
    int diff_add_hr_2=0, diff_add_vr_2=0;

	double currX1=controlPoint.x;
	double currY1=controlPoint.y;
	double currX2=currX1;
	double currY2=currY1;

	int cols=edges.cols;
	int rows=edges.rows;

    if( (currX1 < cols)&&(currX1 > 0)&&
        (currY1 < rows)&&(currY1 > 0) )
    {
        diff2=diff1=edges.at<uchar>(currY1,currX1);
    }
    else
    {
        foundPoint = Point2d(currX1, currY1);
        foundPoint2 = Point2d(currX2, currY2);
        std::cout<<"Warning: The control point doesn't exist to image! Coordinates: ( "<<currX1<<" : "<<currY1<<" )"<<std::endl;
        return false; // The point won't be taken into account
    }

    while( (currX1+dx1<cols)&&(currY1+dy1<rows)&&(currX1+dx1>0)&&(currY1+dy1>0)&&
    	   (currX2+dx2<cols)&&(currY2+dy2<rows)&&(currX2+dx2>0)&&(currY2+dy2>0)&&
           (diff_add_hr_1!=255)&&(diff_add_vr_1!=255)&&
           (diff_add_hr_2!=255)&&(diff_add_vr_2!=255)&&
		   (diff1!=255) && (diff2!=255) )
	{
		diff1=edges.at<uchar>(currY1+dy1,currX1+dx1);
		diff2=edges.at<uchar>(currY2+dy2,currX2+dx2);

        if ((foundDirection == DOWNWARD_DIAGONAL) || (foundDirection == UPWARD_DIAGONAL))
        {
            diff_add_hr_1=edges.at<uchar>(currY1+ 0 ,currX1+dx1);
            diff_add_vr_1=edges.at<uchar>(currY1+dy1,currX1+ 0 );
		    diff_add_hr_2=edges.at<uchar>(currY2+ 0 ,currX2+dx2);
            diff_add_vr_2=edges.at<uchar>(currY2+dy2,currX2+ 0 );
        }

        currX1+=dx1;
		currY1+=dy1;
		currX2+=dx2;
		currY2+=dy2;
        num++;
	}

	if (diff1==255)
    {
		foundPoint = Point2d(currX1, currY1);
        //foundPoint2 = Point2d(currX2, currY2); //to draw purple point at which the search stopped
        if ((diff2==255) && (num == 0))
        {
            cout<<"Warning: The found point and the control point are the same! Control point: ( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<endl;
        }
        if ((diff2==255) && (num > 0))
        {
            foundPoint2 = Point2d(currX2, currY2);
            cout<<"Warning: Found two points! Control point: ( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<endl;
            cout<<"Warning: First 2D point:  ( "<<foundPoint.x<<" : "<<foundPoint.y<<" )"<<endl;
            cout<<"Warning: Second 2D point:  ( "<<foundPoint2.x<<" : "<<foundPoint2.y<<" )"<<endl;
        }
    }
	else
    {
        if(diff2==255)
        {
		    foundPoint = Point2d(currX2, currY2);
            //foundPoint2 = Point2d(currX1, currY1); //to draw purple point at which the search stopped
        }
        else if (diff_add_hr_1 == 255) {
            foundPoint = Point2d(currX1 + 0, currY1-dy1);
        }
        else if (diff_add_vr_1 == 255) {
            foundPoint = Point2d(currX1-dx1, currY1 + 0);
        }
        else if (diff_add_hr_2 == 255) {
            foundPoint = Point2d(currX2 + 0, currY2-dy2);
        }
        else if (diff_add_vr_2 == 255) {
            foundPoint = Point2d(currX2-dx2, currY2 + 0);
        }
        else
        {
            foundPoint = Point2d(currX1, currY1);
            foundPoint2 = Point2d(currX2, currY2);
            cout<<"Warning: Point isn't found for control point( "<<controlPoint.x<<" : "<<controlPoint.y<<" )"<<endl;
            cout<<"Warning: First 2D point:  ( "<<foundPoint.x<<" : "<<foundPoint.y<<" )"<<endl;
            cout<<"Warning: Second 2D point:  ( "<<foundPoint2.x<<" : "<<foundPoint2.y<<" )"<<endl;
            return false; // The point won't be taken into account
        }
    }

	return true;
}

Model RAPIDTracker::ProcessFrame(const Mat& frame)
{
	Mat result = frame.clone();
	Mat edges = ExtractEdges(result);

	Point2d foundPoint,foundPoint2;

	std::list<Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<Mat>::iterator companionPointsIter = model.companionPoints.begin();

	std::vector<Point2d> foundBoxPoints2D;
    std::vector<Point3d> modelPoints3D;

	while (controlPointsIter != model.controlPoints.end())
	{
		Point2d r = model.Project(*controlPointsIter);
		Point2d s = model.Project(*companionPointsIter);
        if (FindPoints(r, s, edges, foundPoint, foundPoint2))
        {
            foundBoxPoints2D.push_back(foundPoint);

		    circle(result, foundPoint, 4, Scalar(0,0,255));
            circle(result, foundPoint2, 4, Scalar(255,0,255));
		    line(result, foundPoint, r, Scalar(0,255,0), 1, 8);

		    double Px = (*controlPointsIter).at<double>(0,0);
		    double Py = (*controlPointsIter).at<double>(0,1);
		    double Pz = (*controlPointsIter).at<double>(0,2);

            modelPoints3D.push_back(Point3f(Px, Py, Pz));
        }
        else
        {
            circle(result, foundPoint, 1, Scalar(255,0,255));
            circle(result, foundPoint2, 1, Scalar(255,0,255));
		    line(result, foundPoint, foundPoint2, Scalar(0,255,0), 1, 8);
            line(result, foundPoint, r, Scalar(255,0,0), 1, 8);
        }

		controlPointsIter++;
		companionPointsIter++;
	}

	Mat rvec,tvec;
    solvePnP(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
                 model.distortionCoefficients, rvec, tvec, false);
    //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
    cout << "---(SolvePnP) delta rotate vector" << endl << rvec - model.rotationVector<< endl;
    cout << "---(SolvePnP) delta translate vector=" << endl << tvec - model.translateVector << endl << endl;

    model.updatePose(rvec - model.rotationVector, tvec - model.translateVector);

	namedWindow("Current: foundPoints", CV_WINDOW_AUTOSIZE);
    imshow("Current: foundPoints", result);

 	return model;
}

double RAPIDTracker::GetConvergenceMeasure(const Model& model1, const Model& model2, int normType)
{
	std::list<Point2d> model1Points = model1.GetProjectedControlPoints();
	std::list<Point2d> model2Points = model2.GetProjectedControlPoints();

	Mat model1PointsCoords(model1Points.size(), 1, CV_64F);
	Mat	model2PointsCoords(model2Points.size(), 1, CV_64F);

	Point2d currentPoint;

	for(std::list<Point2d>::const_iterator pointsIter = model1Points.begin();
		pointsIter != model1Points.end();
		pointsIter++)
	{
		currentPoint = *pointsIter;
		model1PointsCoords.push_back(currentPoint.x);
		model1PointsCoords.push_back(currentPoint.y);
	}

	for(std::list<Point2d>::const_iterator pointsIter = model2Points.begin();
		pointsIter != model2Points.end();
		pointsIter++)
	{
		currentPoint = *pointsIter;
		model2PointsCoords.push_back(currentPoint.x);
		model2PointsCoords.push_back(currentPoint.y);
	}

	return norm(model1PointsCoords, model2PointsCoords, normType);
}
