#include "opencv2/opencv.hpp"
using namespace cv;

#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

void help()
{
cout << "\
-----------------------------------------------------------------------------------\n\
This tool is provided for frame extracting from a video file                       \n\
Usage:                                                                             \n\
./FramesGrabbing InputVideoName	NumberOfFrames FileForListImages LocationOfImages  \n\
Example on Windows:                                                                \n\
%RAPID%\\video\\calib.MOV 30 %RAPID%\\calib_tool\\image_list.xml %RAPID%\\frames\       \n\
-----------------------------------------------------------------------------------\n\
";
}

// false if failed
bool ValidateParameters(int argn, char* argv[])
{
	if (argn < 5)
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
		cerr << "Number of frames are expected to be postitive." << endl;
		help();
		return false;
	}

    int len = strlen(argv[3]);
    char yaml[6], xml[5];
    yaml[5]=0; xml[4]=0;
    strncpy(yaml, argv[3]+len-5, 5);
    strncpy( xml, argv[3]+len-4, 4);

    if( (strcmp(yaml,".yaml")) && (strcmp(xml,".xml")) )
    {
        cerr << "FileForListImages must have the extension .xml or .yaml " << endl;
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

// extracts file name from its path
char* GetVideoFileName(char* path)
{
	char * fileName;
	if (!(fileName = strrchr(path, '\\')))
		if (!(fileName = strrchr(path, '/'))) return path;

	return ++fileName;
}

int main(int argn, char* argv[])
{
	
	if (!ValidateParameters(argn, argv))
		return 1;

	VideoCapture cap;
	if (!OpenVideoFile(cap, argv[1]))
		return 2;
	
	int expectedNumberOfFrames = atoi(argv[2]);
    int totalNumFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);

    if (totalNumFrames < expectedNumberOfFrames)
    {
        cerr<<"The expected number of frames exceeds the total number of frames"<<endl;
        help();
        return 3;
    }

	int frameExtractingStep = totalNumFrames / expectedNumberOfFrames;

    FileStorage fileWithListImages(argv[3], FileStorage::WRITE);

    if (!fileWithListImages.isOpened())
    {
        cerr << "Failed to open " << argv[3] << endl;
        help();
        return 1;
    }

    fileWithListImages << "images" << "[";

	//namedWindow("edges", CV_WINDOW_AUTOSIZE);
	
	char* videoName = GetVideoFileName(argv[1]);

	cout << endl << "Extracting... ";

	int frameIndex = 0;
	int framesExtracted = 0;

	while( framesExtracted < expectedNumberOfFrames )
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
			// jpeg parameters
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(95);
			// feature fails if more than hundred frames are being extracted
			stringstream niceExtractedFrameIndex;
			if (frameIndex / frameExtractingStep < 10) niceExtractedFrameIndex << "0";
			niceExtractedFrameIndex << frameIndex / frameExtractingStep;
			// building the file name
			stringstream frameName;
			frameName << argv[4] << videoName << niceExtractedFrameIndex.str() << ".jpg";
			// writing jpeg
			imwrite(frameName.str(), frame, compression_params);
			// writing filename of image
			fileWithListImages << frameName.str();
			framesExtracted++;
		}
		frameIndex++;
	}

    fileWithListImages << "]";
    cout << endl << "Extracted " << framesExtracted << " frames" << endl;
    cout << "The Location of the extracted frames: " << endl << "    " << argv[4] << endl;
    cout << "The location of the file with a list of images: " << endl << "   " << argv[3] << endl;
	cout << "Done." << endl;

	fileWithListImages.release();

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}