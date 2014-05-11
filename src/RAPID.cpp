#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui_c.h>

// Model traits and handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"
#include "RAPIDTrackerExperiment.hpp"
#include "RAPIDTrackerExperiment_all_k_subsets.hpp"
#include "RAPIDTrackerExperiment_rand_subsets.hpp"
#include "RansacTracker.hpp"
// VideoInfo struct
#include "VideoInfo.hpp"

using std::cout;
using std::cerr;
using std::endl;
using namespace cv;

void help()
{
	cout << endl <<
	"\
--------------------------------------------------------------------------\n\
A Video Rate Object Tracker\n\
Attitude and position determination of a known 3D object\n\
Usage:\n\
./RAPID VideoInfoXmlFile numberOfFirstFrame [-o]\n\
\tVideoInfoXmlFile - XML or YAML file containing video and model information;\n\
\tnumberOfFirstFrame - Tracking algorithm starts with a given frame in the video.\n\
\t[-o] - (optional) if specified, logs will be enabled. \n\
--------------------------------------------------------------------------\n\
	" << endl;
}

// false if validation fails
bool ValidateAndInterpretePrameters(const int argn,
                                    const char* argv[],
                                    int& firstFrame,
                                    bool& isLogsEnabled,
                                    VideoInfo& videoInfo,
                                    VideoCapture& cap,
                                    Mat& cameraMatrix,
                                    Mat& distortionCoefficients);

// false if the process has failed
bool EstimateInititalPose(const Mat& view,
                          const Mat& cameraMatrix,
                          const Mat& distortionCoefficients,
                          Mat& rVector,
                          Mat& tVector,
                          Mat& patternOrigin3D);

int main(int argn, char* argv[])
{
    bool isLogsEnabled = false;
    int firstFrame;
    VideoInfo videoInfo;
    VideoCapture cap;
    Mat Camera_Matrix;
    Mat Distortion_Coefficients;

    if(!ValidateAndInterpretePrameters(
        argn,
        (const char**)argv,
        firstFrame,
        isLogsEnabled,
        videoInfo,
        cap,
        Camera_Matrix,
        Distortion_Coefficients
        ))
        return 1;

    Mat movieFrame;
    //for ../video/../test.mov firstFrame = 78
    for(int i = 0; i < firstFrame; i++)
        cap.read(movieFrame);

    Mat rVec, tVec, patternOrigin3D;
    if (!EstimateInititalPose(movieFrame, Camera_Matrix, Distortion_Coefficients, rVec, tVec, patternOrigin3D))
    {
        cerr << endl << "Can't find the calibration pattern."<< endl
             << " Troubleshooting: change numberOfFirstFrame or the input video" << endl;
        help();
        return 1;
    }

	const int PointsPerEdge = 5;
    Model model(videoInfo.GetCornerPoints(), PointsPerEdge, Camera_Matrix, Distortion_Coefficients, rVec, tVec, isLogsEnabled);

	int n = model.GetNumberControlPoints();

    //RAPIDTracker tracker(model, isLogsEnabled);
	//RAPIDTrackerExperiment_rand_subsets tracker(model, isLogsEnabled, n, n/2);
	RAPIDTrackerExperiment_all_k_subsets tracker(model, isLogsEnabled, 4);

    //RansacTracker tracker(model, isLogsEnabled, 10, 0.5, 1);
    //RansacTracker tracker(model, isLogsEnabled, 100, 8, 20); // correct definition during the whole video (test_small_25.MOV)

    const std::string nextWindowName = "Next";
    const std::string currentWindowName = "Current";

    namedWindow(nextWindowName, CV_WINDOW_AUTOSIZE);
    namedWindow(currentWindowName, CV_WINDOW_AUTOSIZE);

    const int iterationsThreshold = 13;
	const double precisionFreshold = 1e-1;

	while (cap.read(movieFrame))
	{
		double precision = DBL_MAX;

        for(int i = 0; (precision > precisionFreshold) && ( i < iterationsThreshold); i++)
        {
            Mat workFrame = movieFrame.clone();

	        Mat prev = model.Outline(workFrame);
	        imshow(currentWindowName, prev);

			Model prevModel = model;
	        model = tracker.ProcessFrame(workFrame);
			precision = tracker.GetConvergenceMeasure(prevModel, model, NORM_INF);

	        workFrame = model.Outline(workFrame);
	        imshow(nextWindowName, workFrame);

            model.DrawReferencePoints(movieFrame, patternOrigin3D, cap.get(CV_CAP_PROP_POS_FRAMES), i);
	        waitKey();
		}
	}

	return 0;
}

bool EstimateInititalPose(const Mat& circlesImage,
                                     const Mat& Camera_Matrix,
                                     const Mat& Distortion_Coefficients,
                                     Mat& rVector,
                                     Mat& tVector,
                                     Mat& patternOrigin3D)
{
    std::vector<Point2f> foundBoardCorners;
    std::vector<Point3f> boardPoints;
    float squareSize = 17.7;
    Size boardSize(4, 11);
    Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);
    bool found;

    // X, Y, Z - axis offset from the center of the reference circle

    float offsetX = -30;
    float offsetY = 0;
    float offsetZ = -177 - 7 + 145;

    //calcBoardCornerPositions circles
    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            boardPoints.push_back(Point3f(float((2*j + i % 2)*(-squareSize) + offsetX), offsetY, float(i*squareSize) + offsetZ));

    patternOrigin3D = (Mat_<double>(3,1) << boardPoints[40].x, boardPoints[40].y, boardPoints[40].z);
    cout << "boardOrigin" << endl << patternOrigin3D << endl;
    cout << "boardPoints" << endl << boardPoints << endl;
    found = findCirclesGrid(circlesImage, boardSize, foundBoardCorners, 2);

    Mat view = circlesImage.clone();

    if (found)
    {
        //drawChessboardCorners( view, boardSize, Mat(foundBoardCorners), found );
        cout << "found circles Grid!" << endl;
        solvePnP(Mat(boardPoints), Mat(foundBoardCorners), Camera_Matrix,
                     Distortion_Coefficients, rvec, tvec, false);
        cout << "Rotate vector" << endl << rvec << endl << "Translate vector=" << endl << tvec << endl;
    }
    else
        return false;

    tVector = tvec;
    rVector = rvec;
    return true;
}

bool ValidateAndInterpretePrameters(const int argn,
                                    const char* argv[],
                                    int& firstFrame,
                                    bool& isLogsEnabled,
                                    VideoInfo& videoInfo,
                                    VideoCapture& cap,
                                    Mat& cameraMatrix,
                                    Mat& distortionCoefficients)
{
    // checking number of the command line arguments
    if ((argn != 3) && (argn != 4))
    {
        help();
        cerr << "Not enough parameters" << endl;
        return false;
    }

    if (argn == 4)
    {
        if (strcmp(argv[3], "-o"))
        {
           help();
           cerr << "Unknown option: " << argv[3] <<endl;
           return false;
        }
        else
        {
            isLogsEnabled = true;
        }
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
    FileStorage videoInfoStorage(videoInfoXmlPath, FileStorage::READ);
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
    Mat frame;
    if (!cap.read(frame))
    {
        help();
        cerr << "A frame could not be loaded" << endl;
        return false;
    }
    // moving frame pointer to the start of the film
    cap.set(CV_CAP_PROP_POS_AVI_RATIO, 0.0);

    // reading calibration data
    FileStorage cameraData;
    cameraData.open(videoInfo.GetCalibDataPath(), FileStorage::READ);
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