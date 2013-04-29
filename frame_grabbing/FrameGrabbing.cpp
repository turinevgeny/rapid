#include "opencv2/opencv.hpp"
using namespace cv;

#include <iostream>
#include <sstream>
using namespace std;

void help()
{
	cout
	<< "--------------------------------------------------------------------------" << endl
	<< "This program shows you how to read and extract frame from a video file" << endl
	<< "Usage:" << endl
	<< "./FramesGrabbing inputvideoName" << endl
	<< "--------------------------------------------------------------------------" << endl
	<< endl;
}

#define EXTRACT_FRAMES

#ifdef EXTRACT_FRAMES
#include <fstream>
#endif

int main(int argn, char* argv[])
{
	help();

	if (argn < 2)
	{
		cout << "Not enough parameters" << endl;
		return -1;
	}

	VideoCapture cap(argv[1]); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
	{
		cout << "The video" << argv[1] << " could not be loaded." << endl;
		return -1;
	}
	
	namedWindow("edges", CV_WINDOW_AUTOSIZE);

#ifdef EXTRACT_FRAMES
	ofstream f;
	f.open("names.txt");
	int frameN = 0, ind = 0;
#endif

	for(;;)
	{
		Mat frame;
		Mat edges;
		if (!cap.read(frame)) break; // get a new frame from camera
		cvtColor(frame, edges, CV_BGR2GRAY);
		GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		Canny(edges, edges, 20, 100, 3);
		imshow("edges", edges);
		//waitKey();
#ifdef EXTRACT_FRAMES		
		if (frameN % 31 == 0) {
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(95);
			stringstream r;
			if (ind < 10) r << "0";
			r << ind;
			imwrite("circle_"+r.str() + ".jpg", frame, compression_params);

			f << "circle_"+r.str() + ".jpg" << endl;
			ind++;
		}
		frameN++;
#endif	
		if(waitKey(30) >= 0) break;
	}
	
#ifdef EXTRACT_FRAMES
	cout << frameN;
	f.close();
#endif

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}