

#pragma once


#include "UELinker.h"


namespace ORB_SLAM2
{
		void UELinker::Init(char* inVocFile, char* inSettingsFile, eSensorExp sensorType)
		{
			//SLAM = new ORB_SLAM2::System(std::string(inVocFile), std::string(inSettingsFile), System::eSensor(sensorType),true,false);
		}

		void UELinker::TrackMonocular(const cv::Mat& image, double timestamp)
		{
			SLAM->TrackMonocular(image, timestamp);
		}

		void UELinker::ShutdownSlam()
		{
			SLAM->Shutdown();
		}

		int UELinker::GetNumPoints()
		{
			if (SLAM == nullptr || SLAM->GetMap() == nullptr) return 0;
			return SLAM->GetMap()->GetNumMapPoints();
		}

};

