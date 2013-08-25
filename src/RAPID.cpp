#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Model traits and handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"
// VideoInfo struct
#include "VideoInfo.hpp"

using std::cout;
using std::cerr;
using std::endl;

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

// false if validation fails
bool ValidateAndInterpretePrameters(const int argn,
                                    const char* argv[],
                                    int& firstFrame,
                                    VideoInfo& videoInfo,
                                    cv::VideoCapture& cap,
                                    cv::Mat& cameraMatrix,
                                    cv::Mat& distortionCoefficients);

// false if the process has failed
bool GetRotationAndTranslationVector(const cv::Mat& view,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distortionCoefficients,
                                     cv::Mat& rVector,
                                     cv::Mat& tVector);

int main(int argn, char* argv[])
{
    int firstFrame;
    VideoInfo videoInfo;
    cv::VideoCapture cap;
    cv::Mat Camera_Matrix;
    cv::Mat Distortion_Coefficients;

    if(!ValidateAndInterpretePrameters(
        argn,
        (const char**)argv,
        firstFrame,
        videoInfo,
        cap,
        Camera_Matrix,
        Distortion_Coefficients
        ))
        return 1;
    
    cv::Mat frame;
    //for ../video/../test.mov firstFrame = 78
    for(int i = 0; i < firstFrame; i++)
        cap.read(frame);

    cv::Mat rVec, tVec;
    if (!GetRotationAndTranslationVector(frame, Camera_Matrix, Distortion_Coefficients, rVec, tVec))
    {
        cerr << endl << "Can't find the calibration pattern."<< endl
             << " Troubleshooting: change numberOfFirstFrame or the input video" << endl;
        help();
        return 1;
    }

    Model model(tVec.t(), videoInfo.GetCornerPoints(), 3, Camera_Matrix, Distortion_Coefficients, rVec, tVec);
    RAPIDTracker tracker("", model);

    const std::string nextWindowName = "Next";
    const std::string currentWindowName = "Current";

    cv::namedWindow(nextWindowName, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(currentWindowName, CV_WINDOW_AUTOSIZE);

	while (cap.read(frame))
	{
		cv::Mat prev = model.Outline(frame);
		cv::imshow(currentWindowName, prev);
		Model updatedModel = tracker.ProcessFrame(frame);
		frame = updatedModel.Outline(frame);
		cv::imshow(nextWindowName, frame);

		cv::waitKey();
	}

	return 0;
}

bool GetRotationAndTranslationVector(const cv::Mat& circlesImage,
                                     const cv::Mat& Camera_Matrix,
                                     const cv::Mat& Distortion_Coefficients,
                                     cv::Mat& rVector,
                                     cv::Mat& tVector)
{
    std::vector<cv::Point2f> foundBoardCorners;
    std::vector<cv::Point3f> boardPoints;
    float squareSize = 17.7;
    cv::Size boardSize(4, 11);
    cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);
    bool found;

    // X, Y, Z - axis offset from the center of the reference circle

    float offsetX = -30;
    float offsetY = 0;
    float offsetZ = -177 - 7 + 145;

    //calcBoardCornerPositions circles
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            boardPoints.push_back(cv::Point3f(float((2*j + i % 2)*(-squareSize) + offsetX), offsetY, float(i*squareSize) + offsetZ));

    cout << "boardPoints" << endl << boardPoints << endl;
    found = cv::findCirclesGrid(circlesImage, boardSize, foundBoardCorners, 2);

    cv::Mat view = circlesImage.clone();

    if (found) 
    {
        //drawChessboardCorners( view, boardSize, Mat(foundBoardCorners), found );

        //draw reference circle
        cv::circle(view, foundBoardCorners[40], 2, cv::Scalar(0,255,0), 2); //green

        cout << "found circles Grid!" << endl;
        cv::solvePnP(cv::Mat(boardPoints), cv::Mat(foundBoardCorners), Camera_Matrix,
                     Distortion_Coefficients, rvec, tvec, false);
        cout << "Rotate vector" << endl << rvec << endl << "Translate vector=" << endl << tvec << endl;
    }
    else
        return false;

    cv::Mat Box3DPoint = cv::Mat::zeros(3, 1, CV_64F);
    
    cv::Mat expProjectedPoint = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints( Box3DPoint.t(), rvec, tvec, Camera_Matrix, Distortion_Coefficients, expProjectedPoint);
   
    cv::Point2d center(expProjectedPoint.at<double>(0,0), expProjectedPoint.at<double>(0,1));

    //draw block's reference point
    cv::circle(view, center, 2, cv::Scalar(0,0,255), 2);
    cv::namedWindow("drawChessboardCorners", CV_WINDOW_AUTOSIZE);
    cv::imshow("drawChessboardCorners", view);

    //How to get the point (Box3DPoint) in camera coordinates.
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    cout << "Rotate matrix" << endl << rmat << endl;
    cout << "rmat * Box3DPoint" << endl << rmat * Box3DPoint << endl;
    Box3DPoint =  rmat * Box3DPoint + tvec;

    cout << "Box3DPoint" << endl << Box3DPoint << endl; 
    // Box3DPoint = (0,0,0) in box coordinates, so it's equal tvec in camera coordinates

    tVector = tvec;
    rVector = rvec;
    return true;
}

bool ValidateAndInterpretePrameters(const int argn,
                                    const char* argv[],
                                    int& firstFrame,
                                    VideoInfo& videoInfo,
                                    cv::VideoCapture& cap,
                                    cv::Mat& cameraMatrix,
                                    cv::Mat& distortionCoefficients)
{
    // checking number of the command line arguments
    if (argn < 3)
    {
        help();
        cerr << "Not enough parameters" << endl;
        return false;
    }

    firstFrame = atoi(argv[2]);
    std::string videoInfoXmlPath = argv[1];

    if (!firstFrame)
    {
        help();
        cerr << "Incorrect number of the first frame" << endl;
        return false;
    }

    // opening VideoInfo storage
    cv::FileStorage videoInfoStorage(videoInfoXmlPath, cv::FileStorage::READ);
    if (!videoInfoStorage.isOpened())
    {
        cerr << "Couldn't open " << videoInfoXmlPath << " file." << endl;
        return false;
    }
    videoInfoStorage >> videoInfo;
    videoInfoStorage.release();

    // opening video
    cap.open(videoInfo.GetVideoPath()); // open the video file
    if (!cap.isOpened())                // check if we succeeded
    {
        cout << "The video " << videoInfo.GetVideoPath() << " could not be loaded." << endl;
        return false;
    }

    int totalNumFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    if ( (totalNumFrames <= firstFrame) || (firstFrame < 0) )
    {
        cerr << "The number of the first frame should be positive and less than the total number of frames." << endl;
        return false;
    }

    // trying to grab a frame from the video file
    cv::Mat frame;
    if (!cap.read(frame))   
    {
        help();
        cerr << "A frame could not be loaded" << endl;
        return false;
    }
    // moving frame pointer to the start of the film
    cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0.0);

    // reading calibration data
    cv::FileStorage cameraData;
    cameraData.open(videoInfo.GetCalibDataPath(), cv::FileStorage::READ);    
    if (!cameraData.isOpened())
    {
        cerr << "Failed to open " << videoInfo.GetCalibDataPath() << endl;
        return false;
    }
    cameraData["Camera_Matrix"] >> cameraMatrix;
    cameraData["Distortion_Coefficients"] >> distortionCoefficients;
    cameraData.release();

    return true;
}