#include "opencv2/opencv.hpp"
using namespace cv; 

#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

void help()
{
cout << "\
------------------------------------------------------------------------\n\
This tool is provided for frame extracting from a video file            \n\
Usage:                                                                  \n\
./FramesGrabbing inputvideoName	NumberOfFrames       				    \n\
------------------------------------------------------------------------\n\
";
}

// false if failed
bool ValidateParameters(int argn, char* argv[])
{
	if (argn < 3)
	{
		cerr << "Not enough parameters" << endl;
		help();
		return false;
	}
	return true;
}

// false if failed
bool OpenVideoFile(VideoCapture &cap, char* fileName)
{
	cap.open(fileName);
	if(!cap.isOpened())  // check if we succeeded
	{
		cerr << "The video" << fileName << " could not be opened." << endl;
		return false;
	}

	return true;
}

int main(int argn, char* argv[])
{
	
	if (!ValidateParameters(argn, argv))
		return 1;

	VideoCapture cap;
	if (!OpenVideoFile(cap, argv[1]))
		return 2;

	namedWindow("edges", CV_WINDOW_AUTOSIZE);

	ofstream f;
	f.open(argv[2]);
	int frameN = 0, ind = 0;

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

		if(waitKey(30) >= 0) break;
	}
	

	cout << frameN;
	f.close();

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}