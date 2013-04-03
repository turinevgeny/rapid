#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

// model traits and handling methods
#include "Model.hpp"

// points in model coords related with the particular video file
#include "new1.hpp"

// Camera calibrating output
const char filename[] = "../../Calibrating/out_camera_data.xml";

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

	if (!cap.read(frame))	// try to grab a frame from the video file
	{
		cout << "A frame could not be loaded" << endl;
		return -1;
	}

	/*
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(95);

	try {
		imwrite("1stframe.jpg", frame, compression_params);
	}
	catch (runtime_error& ex) {
		fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
		return 1;
	}
	*/
	
	FileStorage camera_data;
	camera_data.open(filename, FileStorage::READ);
	
	if (!camera_data.isOpened())
	{
		cerr << "Failed to open " << filename << endl;
		help();
		return 1;
	}

	Mat Camera_Matrix;
	Mat Distortion_Coefficients;
	//Mat Extrinsic_Parameters; 
	Mat Image_points;

	camera_data["Camera_Matrix"] >> Camera_Matrix;
	camera_data["Distortion_Coefficients"] >> Distortion_Coefficients;
	//camera_data["Extrinsic_Parameters"] >> Extrinsic_Parameters;

	cout << Camera_Matrix << endl
		<< Distortion_Coefficients << endl << endl
		//<< Extrinsic_Parameters.row(1) << endl << endl
		//<< Extrinsic_Parameters.row(2) << endl << endl
		;

	Model model(T, p, 4, Camera_Matrix, Distortion_Coefficients);

	frame = model.Outline(frame);

	imshow("frames", frame);
	waitKey();

	return 0;
}