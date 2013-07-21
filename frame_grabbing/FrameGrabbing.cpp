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
Do not expect it working precisely :)                                   \n\
Usage:                                                                  \n\
./FramesGrabbing inputvideoName	NumberOfFrames outputFileNameList 	    \n\
------------------------------------------------------------------------\n\
";
}

// false if failed
bool ValidateParameters(int argn, char* argv[])
{
	if (argn < 4)
	{
		cerr << "Not enough parameters" << endl;
		help();
		return false;
	}

	if (!atoi(argv[2]))
	{
		cerr << "Incorrect number of frames to extract" << endl;
		help();
		return false;
	}

	if (atoi(argv[2]) <= 0)
	{
		cerr << "Number of frames expected to be postitive." << endl;
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

// extracts file name from the path
char* GetVideoFileName(char* path)
{
	char * fileName;
	if (!(fileName = strrchr(path, '\\')))
		if (!(fileName = strrchr(path, '/'))) return path;

	fileName++;
	return fileName;
}

int main(int argn, char* argv[])
{
	
	if (!ValidateParameters(argn, argv))
		return 1;

	VideoCapture cap;
	if (!OpenVideoFile(cap, argv[1]))
		return 2;
	
	int expectedNumberOfFrames = atoi(argv[2]);

	int frameExtractingStep = cap.get(CV_CAP_PROP_FRAME_COUNT) / (expectedNumberOfFrames-1);

	ofstream fileNamesFile(argv[3]);
	
	//namedWindow("edges", CV_WINDOW_AUTOSIZE);
	
	char* videoName = GetVideoFileName(argv[1]);

	cout << "";
	cout << "Extracting...";

	int frameIndex = 0; //magic number, explanation upon request.
	for(;;)
	{
		Mat frame;
		Mat edges;
		if (!cap.read(frame)) break; // get a new frame from camera
		//cvtColor(frame, edges, CV_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		//Canny(edges, edges, 20, 100, 3);
		//imshow("edges", edges);
		//waitKey();
	
		if (frameIndex % frameExtractingStep == 0) {
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(95);
			// feature fails if more than hundred frames are being extracted
			stringstream niceFrameIndex;
			if (frameIndex < 10) niceFrameIndex << "0";
			niceFrameIndex << frameIndex / frameExtractingStep;
			stringstream frameName;
			frameName << videoName << niceFrameIndex.str() << ".jpg";
			imwrite(frameName.str(), frame, compression_params);

			fileNamesFile << ""+niceFrameIndex.str() + ".jpg" << endl;
		}
		frameIndex++;
	}
	cout << "Done." << endl;

	fileNamesFile.close();

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}