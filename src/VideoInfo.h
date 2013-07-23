#ifndef __VIDEO_INFO_H
#define __VIDEO_INFO_H

class VideoInfo
{
private:
	cv:Mat[] cornerPoints;
public:
	VideoInfo(string fileName,
			  cv::Mat T,
			  double rotationAngle,
			  cv::Mat rotationMatrix,
			  cv:Mat[] cornerPoints);

	~VideoInfo();

	cv:Mat[] GetCornerPoints();
};

#endif