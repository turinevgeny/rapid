#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

// Model traits and handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"
// VideoInfo struct
#include "VideoInfo.hpp"

void help()
{
	cout << endl <<
    "\
     --------------------------------------------------------------------------\n\
    RAPID - A Video Rate Object Tracker\n\
    Real-time attitude and position determination of a known 3D object\n\
    Usage:\n\
    ./RAPID VideoInfoXmlFile numberOfFirstFrame\n\
    VideoInfoXmlFile - XML or YAML file containing video and model information;\n\
    numberOfFirstFrame - Tracking algorithm starts with a given frame in the video.\n\
    --------------------------------------------------------------------------\n\
    " << endl;
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
    string videoInfoXmlPath = argv[1];

    if( !firstFrame )
    {
        cerr << "Incorrect number of the first frame" << endl;
		help();
		return -2;
    }

    // opening VideoInfo storage
    VideoInfo videoInfo;
    FileStorage videoInfoStorage(videoInfoXmlPath, FileStorage::READ);
    if (!videoInfoStorage.isOpened())
    {
        cerr << "Couldn't open " << videoInfoXmlPath << " file." << endl;
        return -3;
    }
    videoInfoStorage >> videoInfo;
    videoInfoStorage.release();

	// opening video
	VideoCapture cap(videoInfo.GetVideoPath());	// open the video file

	if (!cap.isOpened())				// check if we succeeded
	{
		help();
		cout << "The video " << videoInfo.GetVideoPath() << " could not be loaded." << endl;
		return -4;
	}

    int totalNumFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    
    if ( (totalNumFrames <= firstFrame) || (firstFrame < 0) )
    {
        cerr << "The number of the first frame should be positive and less than the total number of frames." << endl;
        help();
        return -5;
    }

	namedWindow("Next: ", CV_WINDOW_AUTOSIZE);

	namedWindow("Current: ", CV_WINDOW_AUTOSIZE);
	
	Mat frame;

	// trying to grab a frame from the video file
	if (!cap.read(frame))	
	{
		help();
		cout << "A frame could not be loaded" << endl;
		return -6;
	}

	// reading calibration data
	FileStorage Camera_Data;
	Camera_Data.open(videoInfo.GetCalibDataPath(), FileStorage::READ);
	
	if (!Camera_Data.isOpened())
	{
		cerr << "Failed to open " << videoInfo.GetCalibDataPath() << endl;
		help();
		return -7;
	}

	Mat Camera_Matrix;
	Mat Distortion_Coefficients;

	Camera_Data["Camera_Matrix"] >> Camera_Matrix;
	Camera_Data["Distortion_Coefficients"] >> Distortion_Coefficients;

	Camera_Data.release();

	cout <<"Camera_Matrix="<<endl<< Camera_Matrix << endl
		 << "Distortion_Coefficients="<<endl<<Distortion_Coefficients << endl;

    //for ../video/../test.mov firstFrame = 78
	for(int i=0; i<firstFrame; i++)
		cap.read(frame);

    vector<Point2f> foundBoardCorners;
    vector<Point3f> boardPoints;
    float squareSize = 17.7;
    Size boardSize(4, 11);
    Mat view = frame;
    Mat rvec(3,1,CV_64F), tvec(3,1,CV_64F);
    bool found;

    // X, Y, Z -axis offset from the center of the reference circle

    float offsetX = -30;
    float offsetY = 0;   
    float offsetZ = -177 - 7 + 145;

    //calcBoardCornerPositions circles
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            boardPoints.push_back(Point3f(float((2*j + i % 2)*(-squareSize) + offsetX), offsetY, float(i*squareSize) + offsetZ));

    cout<<"boardPoints"<<endl<<boardPoints<<endl;
    found = findCirclesGrid( view, boardSize, foundBoardCorners, 2);

    if (found) {
        //drawChessboardCorners( view, boardSize, Mat(foundBoardCorners), found );

        //draw reference circle
        circle(view, foundBoardCorners[40], 2, Scalar(0,255,0), 2); //green

        solvePnP(Mat(boardPoints), Mat(foundBoardCorners), Camera_Matrix,
                     Distortion_Coefficients, rvec, tvec, false);
        cout<<"Rotate vector"<<endl<<rvec<<endl<<"Translate vector="<<endl<<tvec<<endl;
    }
    else
    {
        cerr<<endl<<"Can't find calibration pattern."<< endl
            <<" Troubleshooting: to change numberOfFirstFrame ("<< firstFrame<<") "<<endl
            <<"or input video("<<videoInfo.GetCalibDataPath()<<")"<<endl;
        help();
        return -4;
    }

    Mat Box3DPoint = Mat::zeros(3, 1, CV_64F);
    
    Mat expProjectedPoint = Mat::zeros(3, 1, CV_64F);
    projectPoints( Box3DPoint.t(), rvec, tvec, Camera_Matrix, Distortion_Coefficients, expProjectedPoint);
   
    Point2d center(expProjectedPoint.at<double>(0,0), expProjectedPoint.at<double>(0,1));

    //draw block's reference point 
    circle(view, center, 2, Scalar(0,0,255),2); 
    imshow("drawChessboardCorners: ", view);

    //How to get the point (Box3DPoint) in camera coordinates.
    Mat rmat(3, 3, CV_64F);
    Rodrigues(rvec, rmat);
    cout<<"Rotate matrix"<<endl<<rmat<<endl;
    cout<<"rmat * Box3DPoint"<<endl<< rmat * Box3DPoint <<endl;
    Box3DPoint =  rmat * Box3DPoint + tvec;

    cout<<"Box3DPoint"<<endl<<Box3DPoint<<endl; 
    // Box3DPoint = (0,0,0) in box coordinates, so it's equal tvec in camera coordinates

    Model model(tvec.t(), videoInfo.GetCornerPoints(), 3, Camera_Matrix, Distortion_Coefficients, rvec, tvec);
    RAPIDTracker tracker("", model);

	while (cap.read(frame))
	{
		Mat prev;
		prev = model.Outline(frame);
		imshow("Current: ", prev);
		Model updatedModel = tracker.ProcessFrame(frame);
		frame = updatedModel.Outline(frame);
		imshow("Next: ", frame);

		waitKey();
	}

	return 0;
}
