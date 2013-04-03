#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

#include "Model.hpp"	// model traits and handling methods

void help()
{
	cout
	<< "--------------------------------------------------------------------------" << endl
	<< "RAPID - A Video Rate Object Tracker" << endl
	<< "Real-time attitude and position determination of a known 3D object" << endl
	<< "Usage:" << endl
	<< "./RAPID" << endl
	<< "--------------------------------------------------------------------------" << endl
	<< endl;
}

const char videoFile[] = "../../video/test.MOV";

int main()
{
	VideoCapture cap(videoFile);	// open the video file
	if(!cap.isOpened())		// check if we succeeded
	{
		cout << "The video  " << videoFile << " could not be loaded." << endl;
		return -1;
	}

	namedWindow("canny_edges", CV_WINDOW_AUTOSIZE);
	namedWindow("my_edges", CV_WINDOW_AUTOSIZE);

	Mat frame,edges;
	//while(1)
	{
		if (!cap.read(frame))	// try to grab a frame from the video file
		{
			cout << "A frame could not be loaded" << endl;
			return -1;
		}
		cvtColor(frame, edges, CV_BGR2GRAY);

		GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		Canny(edges, edges, 4,50 , 3);

		Model p;
		p.Outline(edges,false,true);
		imshow("canny_edges", edges);

		p.Outline(frame,true,false);
		imshow("my_edges", frame);
		waitKey();
	}

	return 0;
}
