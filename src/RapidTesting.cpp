#include <iostream>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Model traits and its handling methods
#include "Model.hpp"
// Algorithm wrapper
#include "RAPIDTracker.hpp"

using std::cout;
using std::cerr;
using std::endl;

Model GetHardcodedModel()
{
	const double a = 145.0;
	const double b = 45.0;
	const double c = 65.0;

	//const cv::Mat T = (cv::Mat_<double>(1,3) << -15, 120, 352);

	const double alpha = CV_PI/2 - acos(100/a);

	const cv::Mat rotationMatrix = (cv::Mat_<double>(3,3) <<  cos(alpha), 0, sin(alpha),
															  0,          1, 0,
															  -sin(alpha), 0, cos(alpha));

	const cv::Mat cornerPoints[] = {
		(cv::Mat_<double>(1,3) << 0, 0, 0)*rotationMatrix,	// bottom anterior point on the left side
		(cv::Mat_<double>(1,3) << b, 0, 0)*rotationMatrix,	// bottom anterior point on the right side
		(cv::Mat_<double>(1,3) << b, 0, a)*rotationMatrix,	// bottom rear point on the right side
		(cv::Mat_<double>(1,3) << 0, 0, a)*rotationMatrix,	// bottom rear point on the left side

		(cv::Mat_<double>(1,3) << 0, -c, 0)*rotationMatrix,	// top anterior point on the left side
		(cv::Mat_<double>(1,3) << b, -c, 0)*rotationMatrix,	// top anterior point on the right side
		(cv::Mat_<double>(1,3) << b, -c, a)*rotationMatrix,	// top rear point on the right side
		(cv::Mat_<double>(1,3) << 0, -c, a)*rotationMatrix,	// top rear point on the left side
	};

	const int     pointsPerEdge = 3;
	const cv::Mat cameraMatrix = (cv::Mat_<double>(3,3)	<<
		679.3132512424555, 0, 319.5,
		0, 679.5002034640837, 239.5,
		0, 0, 1);
	const cv::Mat distortionCoefficients =  cv::Mat::zeros(5, 1, CV_64F);
	const cv::Mat rotationVector = cv::Mat::zeros(3, 1, CV_64F);
	const cv::Mat translateVector = (cv::Mat_<double>(3,1) << -18, 30, 200);

	Model model(/*T,*/cornerPoints, pointsPerEdge, cameraMatrix, distortionCoefficients, rotationVector, translateVector);

	return model;
}

class FakeMovie
{
public:
	~FakeMovie() {}
	// filmScenario is a list of a vectors-solutions (angles|translation)
	FakeMovie(std::list<cv::Mat> filmScenario, Model initialModelState, int height, int width)
	{
		cv::Mat initialFrame = initialModelState.Outline(GetBlackFrame(height, width), false);
		movie.push_back(initialFrame);
		Model currentModelState = initialModelState;

		std::list<cv::Mat>::iterator scenarioIterator = filmScenario.begin();
		while (scenarioIterator != filmScenario.end())
		{
			currentModelState.updatePose(*scenarioIterator);
			movie.push_back(currentModelState.Outline(GetBlackFrame(height, width), false));
			scenarioIterator++;
		}

		movieIterator = movie.begin();
	}
	// false when there's no next frame
	bool ReadNextFrame(cv::Mat& destination)
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

		cv::namedWindow(fakeMovieWindow, CV_WINDOW_AUTOSIZE);

		std::list<cv::Mat>::iterator movieIterator = movie.begin();
		while(movieIterator != movie.end())
		{
			cv::imshow(fakeMovieWindow, *movieIterator);
			cv::waitKey();
			movieIterator++;
		}
	}
private:
	std::list<cv::Mat> movie;
	std::list<cv::Mat>::iterator movieIterator;
private:
	cv::Mat GetBlackFrame(int height, int width)
	{
		return cv::Mat::zeros(height, width, CV_8UC3);
	}
};

int main(int argn, char* argv[])
{
	/// Windows names
	const std::string currentWindowName = "Current";
	const std::string nextWindowName = "Next";

	const int VideoHeight = 480;
	const int VideoWidth = 640;

	cv::Mat firstMovement = (cv::Mat_<double>(6,1) << 0.0, -0.001, 0.002, 0.01, 0.03, -0.01);
    //cv::Mat firstMovement = (cv::Mat_<double>(6,1) << 0.001, 0.002, 0.003, 0.5, 0.5, 0.5);
    //cv::Mat firstMovement = (cv::Mat_<double>(6,1) << 0.0, 0.0, 0.0, 2.0, 2.0, 2.0);// TODO: Why does it crash?

	Model model = GetHardcodedModel();
	//model.updatePose(firstMovement);

	std::list<cv::Mat> fakeMovieScenario;

    for(int i=0;i<1000;i++)
		fakeMovieScenario.push_back(firstMovement);

	FakeMovie movie(fakeMovieScenario, GetHardcodedModel(), VideoHeight, VideoWidth);
	//movie.Play();
    RAPIDTracker tracker(model);

    //Model updatedModel = GetHardcodedModel();
    cv::Mat movieFrame;
    int frameNumber = 0;
    cv::Scalar blueColor = cv::Scalar(255, 0, 0);

	movie.ReadNextFrame(movieFrame);
	movie.ReadNextFrame(movieFrame);
	movie.ReadNextFrame(movieFrame);
    while(movie.ReadNextFrame(movieFrame))
    {   
        frameNumber++;

        cv::Mat cleanFrame = movieFrame.clone();
        cv::Mat prev = model.Outline(movieFrame, true, blueColor);
		cv::imshow(currentWindowName, prev);

        model = tracker.ProcessFrame(movieFrame);

	    movieFrame = model.Outline(movieFrame, true, blueColor);
        cv::imshow(nextWindowName, movieFrame);
        cv::Mat temp = cv::Mat::zeros(3, 1, CV_64F);

        model.DrawReferencePoints(cleanFrame, temp, frameNumber);
		cv::waitKey();
        //movieFrame = cv::Mat::zeros(VideoHeight, VideoWidth, CV_8UC3); //does not help! 
    }
    //movie.Play();

	return 0;
}
