#include "RAPIDTracker.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"

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
	/*Функция cvCanny() библиотеки openCV принимает входное изображение в оттенках
	серого, и выходное изображение, которое также должно быть в оттенках серого
	(хотя в действительности будет являться бинарным изображением - содержащим
	только белые пиксели на месте граней и черные для всех остальных точек)
	source: http://locv.ru/wiki/6.5_Canny */

	Mat edges;
	cvtColor(image, edges, CV_BGR2GRAY);
	GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
	Canny(edges, edges, 0, 0, 3);
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

	double num=0;
	int diff1=0,diff2=0;
	double currX1=controlPoint.x;
	double currY1=controlPoint.y;
	double currX2=currX1;
	double currY2=currY1;

	while( (num < 30) && (diff1!=255) && (diff2!=255) )
	{
		num+=1;
		currX1+=dx1;
		currY1+=dy1;
		currX2+=dx2;
		currY2+=dy2;
		diff1=edges.at<uchar>(currX1,currY1);
		diff2=edges.at<uchar>(currX2,currY2);
	}

	if(diff1==255)
		foundPoint=Point2d(currX1,currY1);
	else
		foundPoint=Point2d(currX2,currY2);

	cout<<"foundDirection= "<<foundDirection<<endl;
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

double RAPIDTracker::test(const Mat &image,Point2d controlPoint,Point2d companionPoint,Point2d &foundPoint)
{
	Mat edges=ExtractEdges(image);
	return GetDisplacement(controlPoint,companionPoint,edges,foundPoint);
}

/*Direction RAPIDTracker::FindSearchDirection(Point2d controlPoint, Point2d companionPoint) const
{

}*/

Model RAPIDTracker::ProcessFrame(const Mat &frame)
{
	std::list<Mat>::iterator controlPointsIter = model.controlPoints.begin();
	std::list<Mat>::iterator companionPointsIter = model.companionPoints.begin();

	Mat edges = ExtractEdges(frame);

	while (controlPointsIter != model.controlPoints.end())
	{
		Point2d r = model.Project(*controlPointsIter);
		Point2d s = model.Project(*companionPointsIter);

		Point2d foundPoint;
		cout << "l:" <<  GetDisplacement(r,s,edges,foundPoint) << endl;
		
		controlPointsIter++;
		companionPointsIter++;
	}

	Model model;
	return model;
}
