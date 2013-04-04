#ifndef __MODEL_H
#define __MODEL_H

#include <opencv2/core/core.hpp>
using namespace cv;

class Model
{
public:
	Model();
	Mat& Outline(Mat& source,bool EnableLine, bool EnablePoint);		// outlining (projecting) the model onto the image
	Point* GetPoint2D(Point3d P);
private:
	Point3d T;						// model coordinate system origin in camera coords
	Point3d A,B,C,D,A1,B1,C1,D1;	// control points, so to speak, in model coords
	Point   opt_center;
	Point   focus;
	//int l,w,h;


};

#endif

