#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;

// Model traits and handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"

// Points in model coords related with the particular video file
#include "new1.hpp"

void help()
{
	cout
		<< "--------------------------------------------------------------------------" << endl
		<< "RAPID - A Video Rate Object Tracker" << endl
		<< "Real-time attitude and position determination of a known 3D object" << endl
		<< "Usage:" << endl
		<< "./RAPID calibrationData numberOfFirstFrame" << endl
		<< "calibrationData - XML or YAML file containing camera calibration data." << endl
        << "numberOfFirstFrame - Tracking algorithm starts with a given frame in the video" << endl
		<< "--------------------------------------------------------------------------" << endl
		<< endl;
}

int main(int argn, char* argv[])
{
	// checking command line arguments
	if (argn < 3)
	{
		help();
		cerr << "Not enough parameters" << endl;
		return -1;
	}

    int firstFrame = atoi(argv[2]);
    if( !firstFrame )
    {
        cerr << "Incorrect number of first frame" << endl;
		help();
		return -1;
    }

	// opening video
	VideoCapture cap(videoFile);	// open the video file

	if(!cap.isOpened())				// check if we succeeded
	{
		help();
		cout << "The video " << videoFile << " could not be loaded." << endl;
		return -1;
	}

    int totalNumFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    
    if ( (totalNumFrames <= firstFrame) || (firstFrame < 0) )
    {
        cerr<<"The number of first frame should be positive and less than the total number of frames "<<endl;
        help();
        return -1;
    }

	namedWindow("Next: ", CV_WINDOW_AUTOSIZE);

	namedWindow("Current: ", CV_WINDOW_AUTOSIZE);
	
	Mat frame;

	// trying to grab a frame from the video file
	if (!cap.read(frame))	
	{
		help();
		cout << "A frame could not be loaded" << endl;
		return -2;
	}

	// reading calibration data
	FileStorage Camera_Data;
	Camera_Data.open(argv[1], FileStorage::READ);
	
	if (!Camera_Data.isOpened())
	{
		cerr << "Failed to open " << argv[1] << endl;
		help();
		return -3;
	}

	Mat Camera_Matrix;
	Mat Distortion_Coefficients;

	Camera_Data["Camera_Matrix"] >> Camera_Matrix;
	Camera_Data["Distortion_Coefficients"] >> Distortion_Coefficients;

	Camera_Data.release();

	cout <<"Camera_Matrix="<<endl<< Camera_Matrix << endl
		 << "Distortion_Coefficients="<<endl<<Distortion_Coefficients << endl;

	Model model(T, p, 3, Camera_Matrix, Distortion_Coefficients);

    //for ../video/../test.mov firstFrame = 78
	for(int i=0; i<firstFrame; i++)
		cap.read(frame);
	RAPIDTracker tracker("", model);

#if 1
/*      ---Experiment with SolvePnP method---
    The experiment were performed with video from dropbox.com(26July_calib_and_track_video/P1090149.MOV)
    firstFrame = 1945 
    it's return rvec and tvec for pattern with circles (not yet tested)
*/
    vector<Point2f> foundBoardCorners;
    vector<Point3f> boardPoints;
    float squareSize = 17.7;
    Size boardSize(4, 11);
    Mat view = frame;
    Mat rvec, tvec;
    bool found;

    // X-axis and Y-axis offset from the center of the reference circle
    float offsetX = -30; 
    float offsetY = 7;   

    //calcBoardCornerPositions circles
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            boardPoints.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
    
    found = findCirclesGrid( view, boardSize, foundBoardCorners, 2);

    //drawChessboardCorners( view, boardSize, Mat(foundBoardCorners), found );

    //draw reference circle
    circle(view, foundBoardCorners[40], 2, Scalar(0,255,0) ); //green
    
    if (found) {
        solvePnP(Mat(boardPoints), Mat(foundBoardCorners), Camera_Matrix,
                     Distortion_Coefficients, rvec, tvec, false);
        cout<<"Rotate vector"<<endl<<rvec<<endl<<"Translate vector="<<endl<<tvec<<endl;
    }

    Mat Box3DPoint(1, 3, CV_32F); 

    Box3DPoint.at<float>(0,0) = boardPoints[40].x - offsetX;
    Box3DPoint.at<float>(0,1) = boardPoints[40].y + offsetY;
    Box3DPoint.at<float>(0,2) = boardPoints[40].z;

    Mat expProjectedPoint;
    projectPoints( Box3DPoint, rvec, tvec, Camera_Matrix, Distortion_Coefficients, expProjectedPoint);
   
    Point2d center(expProjectedPoint.at<float>(0,0),expProjectedPoint.at<float>(0,1));

    //draw block's reference point 
    circle(view, center, 2, Scalar(255,0,0));
    imshow("drawChessboardCorners: ", view);
#endif

	while(cap.read(frame))
	{
		Mat prev;
		prev=model.Outline(frame);
		imshow("Current: ", prev);
		Model updatedModel=tracker.ProcessFrame(frame);
		frame=updatedModel.Outline(frame);
		imshow("Next: ", frame);

		waitKey();
	}

	return 0;
}
