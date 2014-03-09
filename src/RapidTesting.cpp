#include <iostream>
#include <list>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

// Model traits and its handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"

using std::cout;
using std::cerr;
using std::endl;

using namespace cv;

class RAPIDTestingTracker : public RAPIDTracker
{
public:
	RAPIDTestingTracker(Model& model) : RAPIDTracker(model) { }

	virtual cv::Mat	ExtractEdges(const cv::Mat& image) const
	{
		Mat edges;
		cvtColor(image, edges, CV_BGR2GRAY);
		return edges;
	}
};

class RapidTestingModel : public Model
{
public:
	RapidTestingModel() {}
	RapidTestingModel(const cv::Mat* cornerPoints,
		const int      pointsPerEdge,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distortionCoefficients,
		const cv::Mat& rotationVector,
		const cv::Mat& translateVector)
		: Model(cornerPoints, pointsPerEdge, cameraMatrix, distortionCoefficients, rotationVector, translateVector)
	{ }
	RapidTestingModel(const RapidTestingModel& model) : Model(model) { }
	RapidTestingModel(const Model& model) : Model(model) { }

	virtual void SetControlPoints()
	{
		// pointsPerEdge control point correspond to every edge
		AddControlPointsFromTheEdge(1, 2);
		AddControlPointsFromTheEdge(0, 1);
		//AddControlPointsFromTheEdge(2, 3);
		AddControlPointsFromTheEdge(0, 4);

		//AddControlPointsFromTheEdge(4, 5);
		AddControlPointsFromTheEdge(5, 6);
		//AddControlPointsFromTheEdge(6, 7);
		AddControlPointsFromTheEdge(7, 4);

		AddControlPointsFromTheEdge(0, 3);
		AddControlPointsFromTheEdge(1, 5);
		//AddControlPointsFromTheEdge(2, 6);
		AddControlPointsFromTheEdge(3, 7);
	}
};

Model GetHardcodedModel()
{
	const double a = 145.0;
	const double b = 45.0;
	const double c = 65.0;

    std::map <std::string, double> angle;
    angle["default"] = CV_PI/2 - acos(100/a);
    angle["north_east"] = CV_PI - acos(100/a);

	const double alpha = angle["north_east"];

	const Mat rotationMatrix = (Mat_<double>(3,3) <<  cos(alpha), 0, sin(alpha),
		0,          1, 0,
		-sin(alpha), 0, cos(alpha));

	const Mat cornerPoints[] = {
		(Mat_<double>(1,3) << 0, 0, 0)*rotationMatrix,	// bottom anterior point on the left side
		(Mat_<double>(1,3) << b, 0, 0)*rotationMatrix,	// bottom anterior point on the right side
		(Mat_<double>(1,3) << b, 0, a)*rotationMatrix,	// bottom rear point on the right side
		(Mat_<double>(1,3) << 0, 0, a)*rotationMatrix,	// bottom rear point on the left side

		(Mat_<double>(1,3) << 0, -c, 0)*rotationMatrix,	// top anterior point on the left side
		(Mat_<double>(1,3) << b, -c, 0)*rotationMatrix,	// top anterior point on the right side
		(Mat_<double>(1,3) << b, -c, a)*rotationMatrix,	// top rear point on the right side
		(Mat_<double>(1,3) << 0, -c, a)*rotationMatrix,	// top rear point on the left side
	};

	const int pointsPerEdge = 5;
	const Mat cameraMatrix = (Mat_<double>(3,3)	<<
		679.3132512424555, 0, 319.5,
		0, 679.5002034640837, 239.5,
		0, 0, 1);
	const Mat distortionCoefficients =  Mat::zeros(5, 1, CV_64F);
	const Mat rotationVector = Mat::zeros(3, 1, CV_64F);

    std::map <std::string, Mat> tVec;
    tVec["default"] = (Mat_<double>(3,1) << -18, 30, 200);
    tVec["north_east"] = (Mat_<double>(3,1) << 90, 10, 300);

	const Mat translateVector = tVec["north_east"];

	Model model(cornerPoints, pointsPerEdge, cameraMatrix, distortionCoefficients, rotationVector, translateVector);

	return model;
}

class FakeMovie
{
public:
	~FakeMovie() {}
	// filmScenario is a list of a vectors-solutions (angles|translation)
	FakeMovie(std::list<Mat> filmScenario, Model initialModelState, int height, int width)
	{
		Mat initialFrame = initialModelState.Outline(GetBlackFrame(height, width), false);
		movie.push_back(initialFrame);
		Model currentModelState = initialModelState;

		std::list<Mat>::iterator scenarioIterator = filmScenario.begin();
		while (scenarioIterator != filmScenario.end())
		{
			currentModelState.updatePose(*scenarioIterator);
			movie.push_back(currentModelState.Outline(GetBlackFrame(height, width), false));
			scenarioIterator++;
		}

		movieIterator = movie.begin();
	}
	// false when there's no next frame
	bool ReadNextFrame(Mat& destination)
	{
		if (movieIterator == movie.end())
			return false;
		destination = *movieIterator;
		movieIterator++;
		return true;
	}
	void RewindToTheStart()
	{
		movieIterator = movie.begin();
	}
	void Play()
	{
		const std::string fakeMovieWindow = "Fake movie";

		namedWindow(fakeMovieWindow, CV_WINDOW_AUTOSIZE);

		std::list<Mat>::iterator movieIterator = movie.begin();
		while(movieIterator != movie.end())
		{
			imshow(fakeMovieWindow, *movieIterator);
			waitKey();
			movieIterator++;
		}
	}
private:
	std::list<Mat> movie;
	std::list<Mat>::iterator movieIterator;
private:
	Mat GetBlackFrame(int height, int width)
	{
		return Mat::zeros(height, width, CV_8UC3);
	}
};

int main(int argn, char* argv[])
{
	/// Windows names
	const std::string currentWindowName = "Current";
	const std::string nextWindowName = "Next";

	const int VideoHeight = 480;
	const int VideoWidth = 640;

    std::map <std::string, Mat> movementVector6D;
    movementVector6D["default"]              = (Mat_<double>(6,1) << 0.001, 0.002, 0.003, 0.1, 0.1, 0.1);
    movementVector6D["bigTranslate"]         = (Mat_<double>(6,1) << 0, 0, 0, 2.0, 2.0, 2.0);
    movementVector6D["smallTranslate"]       = (Mat_<double>(6,1) << 0, 0, 0, 0.1, 0.1, -0.1);
    movementVector6D["mediumTranslate"]      = (Mat_<double>(6,1) << 0, 0, 0, -0.8, 0.8, -0.8);
    movementVector6D["oneDirectionRotateZ"]  = (Mat_<double>(6,1) << 0, 0, CV_PI/56, 0.0, 0.0, 0.0);
    movementVector6D["oneDirectionRotateY"]  = (Mat_<double>(6,1) << 0, CV_PI/56, 0, 0.0, 0.0, 0.0);
    movementVector6D["oneDirectionRotateX"]  = (Mat_<double>(6,1) << CV_PI/56, 0, 0, 0.0, 0.0, 0.0);
    movementVector6D["oneDirectionRotateZs"] = (Mat_<double>(6,1) << 0, 0, CV_PI/120, 0.0, 0.0, 0.0);

	Model model = (RapidTestingModel) GetHardcodedModel();

	std::list<Mat> fakeMovieScenario;

    for(int i=0;i<1000;i++)
		fakeMovieScenario.push_back(movementVector6D["oneDirectionRotateZ"]);

	FakeMovie movie(fakeMovieScenario, GetHardcodedModel(), VideoHeight, VideoWidth);
	//movie.Play();
    RAPIDTestingTracker tracker(model);

    Mat movieFrame;
    int frameNumber = 0;
    Scalar blueColor = Scalar(255, 0, 0);

    const int iterationsThreshold = 12;
	const double precisionFreshold = 1e-1;

    while(movie.ReadNextFrame(movieFrame))
    {
        frameNumber++;

        double precision = DBL_MAX;

        for(int i = 0; (precision > precisionFreshold) && ( i < iterationsThreshold); i++)
        {
            Mat workFrame = movieFrame.clone();

            Mat prev = model.Outline(workFrame, true, blueColor);
	        imshow(currentWindowName, prev);

			Model prevModel = model;
            model = tracker.ProcessFrame(workFrame);

			precision = tracker.GetConvergenceMeasure(prevModel, model, NORM_INF);

	        workFrame = model.Outline(workFrame, true, blueColor);
            imshow(nextWindowName, workFrame);

            Mat temp = Mat::zeros(3, 1, CV_64F);//Fake point of pattern
            model.DrawReferencePoints(movieFrame, temp, frameNumber, i);
	        waitKey();
        }
    }

	return 0;
}
