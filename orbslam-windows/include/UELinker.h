/**
*
* This file is part of ORB-SLAM2 implementation for ITMO VISLab
*
**/

#include "opencv2/core.hpp"
#include <Converter.h>
#include "System.h"

namespace ORB_SLAM2
{
	
		enum eSensorExp
		{
			MONOCULAR = 0,
			STEREO = 1,
			RGBD = 2
		};

		class UELinker 
		{
		public:
			UELinker() {};
			~UELinker() {};
			 void Init(char* inVocFile, char* inSettingsFile, eSensorExp sensorType = MONOCULAR);
			 void TrackMonocular(const cv::Mat& image, double timestamp);
			 void ShutdownSlam();
			 int GetNumPoints();
			 void GetAllMapPoint(int& outCount, float* pointData, char* fileToWrite = "");
			ORB_SLAM2::System* SLAM;
		};

		//UELinker* CreateLinker(char* inVocFile, char* inSettingsFile, eSensorExp sensorType = MONOCULAR);
		//void AAA();
}