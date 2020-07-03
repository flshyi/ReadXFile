#ifndef read_device_calibration_xml_h__
#define read_device_calibration_xml_h__

#include <opencv2/opencv.hpp>

class CameraPara
{
public:
	CameraPara();
public:
	cv::Mat cameraMatrix;
	cv::Mat distortCoeffs;
	cv::Size imageSize;
	cv::Mat R;
	cv::Mat T;
	double fx, fy, cx, cy;
	double w;
	bool useFisheyeMode;
};

class CalibrationResult
{
public:
	CalibrationResult();
public:
	CameraPara left;
	CameraPara right;
	cv::Mat E, F;
	std::vector<double> vec_ombc, vec_tbc;
	std::vector<double> vec_aBias, vec_wBias;
	std::vector<double> vec_ka, vec_kg;
	double dDelta;
	bool LoadXML(const char* file_name);
};



#endif // read_device_calibration_xml_h__
