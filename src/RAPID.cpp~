#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

// Model traits and handling methods
#include "Model.hpp"

// Points in model coords related with the particular video file
#include "new1.hpp"

// Camera calibrating output
const char filename[] = "../out_camera_data.xml";

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

int main(int argn, char* argv[])
{
	// opening video
	VideoCapture cap(videoFile);	// open the video file

	if(!cap.isOpened())				// check if we succeeded
	{
		cout << "The video" << videoFile << " could not be loaded." << endl;
		return -1;
	}

	namedWindow("frames", CV_WINDOW_AUTOSIZE);
	
	Mat frame;

	// trying to grab a frame from the video file
	if (!cap.read(frame))	
	{
		cout << "A frame could not be loaded" << endl;
		help();
		return -2;
	}
	
	// Reading calibration data
	FileStorage Camera_Data;
	Camera_Data.open(filename, FileStorage::READ);
	
	if (!Camera_Data.isOpened())
	{
		cerr << "Failed to open " << filename << endl;
		help();
		return -3;
	}

	Mat Camera_Matrix;
	Mat Distortion_Coefficients;
	Mat Image_Points;

	Camera_Data["Camera_Matrix"] >> Camera_Matrix;
	Camera_Data["Distortion_Coefficients"] >> Distortion_Coefficients;

	Camera_Data.release();

	cout << Camera_Matrix << endl
		 << Distortion_Coefficients << endl
		 ;

	Model model(T, p, 4, Camera_Matrix, Distortion_Coefficients);

	frame = model.Outline(frame);

	imshow("frames", frame);
	waitKey();

	return 0;
}
