#include "Model.hpp"
using namespace cv;

Model :: Model()
{
	opt_center.x=320;
	opt_center.y=240;
	focus.x=648;
	focus.y=648;

	T.x  =  -24; T.y  = 117; T.z  = 352;

	A.x  =    0; A.y  =   0; A.z  =    0;
	B.x  =   53; B.y  =   0; B.z  =   53;
	C.x  =  -47; C.y  =   0; C.z  =  158;
	D.x  =  -100; D.y  =   0; D.z  =  105;

	A1.x =    0; A1.y = -55; A1.z  =    0;
	B1.x  =  53; B1.y = -55; B1.z  =   53;
	C1.x  = -47; C1.y = -55; C1.z  =  158;
	D1.x =  -100; D1.y = -55; D1.z  =  105;

	//T.x  =  -15; T.y  = 120; T.z  = 352;
	//D.x  = -100; D.y  =   0; D.z  = 105;
	//A1.x =    0; A1.y = -55; A1.z =   0;
}
Point* Model :: GetPoint2D(Point3d P)
{
	double p1=(focus.x*(T.x+P.x))/(T.z+P.z)+opt_center.x;
	double p2=(focus.y*(T.y+P.y))/(T.z+P.z)+opt_center.y;
	Point* Temp=new Point( p1,p2 );
	return Temp;
}
Mat& Model :: Outline(Mat& source,bool EnableLine, bool EnablePoint)
{
	Scalar white( 255, 255, 255 );
	Scalar red  ( 0, 0, 255 );
	Scalar green( 0, 255, 0 );

	if(EnableLine)
	{
		line(source, *GetPoint2D(A) ,*GetPoint2D(D),white);
		line(source, *GetPoint2D(A) ,*GetPoint2D(A1),white);
		line(source, *GetPoint2D(A) ,*GetPoint2D(B),white);

		line(source, *GetPoint2D(D1),*GetPoint2D(A1),white);
		line(source, *GetPoint2D(D1),*GetPoint2D(D),white);
		line(source, *GetPoint2D(D1),*GetPoint2D(C1),white);

		line(source, *GetPoint2D(B1),*GetPoint2D(B),white);
		line(source, *GetPoint2D(B1),*GetPoint2D(A1),white);
		line(source, *GetPoint2D(B1),*GetPoint2D(C1),white);

		line(source, *GetPoint2D(C) ,*GetPoint2D(C1),white);
		line(source, *GetPoint2D(C) ,*GetPoint2D(D),white);
		line(source, *GetPoint2D(C) ,*GetPoint2D(B),white);
	}


	int radius = 3;
	int thickness = -1;
	int lineType = 8;

	circle(source,opt_center,4,white,thickness,lineType);

	if(EnablePoint)
	{
		circle(source,*GetPoint2D(A), radius,white,thickness,lineType);
		circle(source,*GetPoint2D(B), radius,white,thickness,lineType);
		circle(source,*GetPoint2D(C), radius,white,thickness,lineType);
		circle(source,*GetPoint2D(D), radius,white,thickness,lineType);
		circle(source,*GetPoint2D(A1),radius,white,thickness,lineType);
		circle(source,*GetPoint2D(B1),radius,white,thickness,lineType);
		circle(source,*GetPoint2D(C1),radius,white,thickness,lineType);
		circle(source,*GetPoint2D(D1),radius,white,thickness,lineType);
	}

	return source;
}
