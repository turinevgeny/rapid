#include "RAPIDTracker.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cmath>

using namespace cv;
using std::cout;
using std::endl;
using std::sqrt;

RAPIDTracker::RAPIDTracker(const std::string _videoFile, const Model &_model)
{
	videoFile = _videoFile;
	model = _model;
}

Mat RAPIDTracker::ExtractEdges(const Mat &image) const
{
	Mat edges;

	cvtColor(image, edges, CV_BGR2GRAY);
	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	Canny(edges, edges, 20, 100, 3);

	return edges;
}


double RAPIDTracker::GetDisplacement(Point2d controlPoint, Point2d companionPoint, const Mat &edges, Point2d &foundPoint)
{
	double kx=1/model.cameraMatrix.at<double>(0,0);
	double ky=1/model.cameraMatrix.at<double>(1,1);
	double beta = ky/kx;

	double tempX=(companionPoint.x-controlPoint.x);
	double tempY=(companionPoint.y-controlPoint.y);

	if (tempY < 0)
	{
		tempX *= -1;
		tempY *= -1;
	}

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

	while( (currX1<cols)&&(currX2<cols)&&(currY1<rows)&&(currY2<rows)&&
		   (currX1>0)&&(currX2>0)&&(currY1>0)&&(currY2>0)&& 
		   (diff1!=255) && (diff2!=255) )
	{
		num++;
		currX1+=dx1;
		currY1+=dy1;
		currX2+=dx2;
		currY2+=dy2;
		diff1=edges.at<uchar>(currY1,currX1);
		diff2=edges.at<uchar>(currY2,currX2);
	}

	if(diff1==255)
		foundPoint=Point2d(currX1,currY1);
	else
		foundPoint=Point2d(currX2,currY2);

//	cout<<"controlPoint  "<<controlPoint.x<<" : "<<controlPoint.y<<endl;
//	cout<<"foundPoint  "<<foundPoint.x<<" : "<<foundPoint.y<<endl;

	// Displays how many iterations are searched foundPoints
		//cout<<"num  "<<num<<endl;

	switch(foundDirection)
	{
		case DOWNWARD_DIAGONAL:
			return num*(ky*cosineAlpha+kx*sineAlpha);
			break;
		case UPWARD_DIAGONAL:
			return num*(ky*cosineAlpha-kx*sineAlpha);
			break;
		case VERTICAL:
			return num*ky*cosineAlpha;
			break;
		case HORIZONTAL:
			return -num*kx*sineAlpha;
			break;
	}
	return -1;
}

//Model RAPIDTracker::ProcessFrame(const Mat &frame)
Model RAPIDTracker::ProcessFrame(const Mat &frame)
{
	Mat result=frame.clone();

	std::list<Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<Mat>::iterator companionPointsIter = model.companionPoints.begin();

	Mat edges = ExtractEdges(result);

//	namedWindow("canny", CV_WINDOW_AUTOSIZE);
//	imshow("canny",edges);

	Point2d foundPoint;
	double l;
	double tempX,tempY;
	double sineAlpha,cosineAlpha;

	Mat a,b,c;
	Mat right = Mat::zeros(6, 1, CV_64F);
	Mat left  = Mat::zeros(6, 6, CV_64F);

	while (controlPointsIter != model.controlPoints.end())
	{
		Point2d r = model.Project(model.T+*controlPointsIter);
		Point2d s = model.Project(model.T+*companionPointsIter);

		//cout << "l:" <<  GetDisplacement(r,s,result,foundPoint) << endl;
		l=GetDisplacement(r,s,edges,foundPoint);

		circle(result, foundPoint, 5, Scalar(0,0,255));

		tempX=(s.x-r.x);
		tempY=(s.y-r.y);
		sineAlpha    = tempY/sqrt(tempX*tempX + tempY*tempY);
		cosineAlpha  = tempX/sqrt(tempX*tempX + tempY*tempY);

		double x=r.x;
		double y=r.y;
		double Px=(*controlPointsIter).at<double>(0,0);
		double Py=(*controlPointsIter).at<double>(0,1);
		double Pz=(*controlPointsIter).at<double>(0,2);
		double Tz=model.T.at<double>(0,2);

		a = (Mat_<double>(6,1) <<  -x*Py, x*Px+Pz, -Py, 1, 0,-x);
		b = (Mat_<double>(6,1) <<  -y*Py-Pz, y*Px,  Px, 0, 1,-y);
		a/=Tz+Pz;
		b/=Tz+Pz;
		c=a*cosineAlpha-b*sineAlpha;

		left +=c*c.t();
		//right-=c*l;	// why does it move so strange ?????
		right-=c*abs(l);// why doesn't it move ?????????????

		controlPointsIter++;
		companionPointsIter++;
	}
	Mat solution;
	solve(left,right,solution);

	model.updatePose(solution);

	namedWindow("foundPoints", CV_WINDOW_AUTOSIZE);
	imshow("foundPoints", result);

 	return model;
}

