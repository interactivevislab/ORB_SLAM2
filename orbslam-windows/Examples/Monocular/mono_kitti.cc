/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
#include <WS2tcpip.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <Converter.h>
#include<opencv2/core/core.hpp>
//#include <Antilatency.InterfaceContract.LibraryLoader.h>
//#include <Antilatency.DeviceNetwork.h>
//#include <Antilatency.StorageClient.h>
//#include <Antilatency.Alt.Tracking.h>
#include <opencv2/core/eigen.hpp>
#include<System.h>

#include<time.h>


//Antilatency::DeviceNetwork::INetwork deviceNetwork;
//Antilatency::Alt::Tracking::ILibrary altTrackingLibrary;
//Antilatency::Alt::Tracking::ITrackingCotask trackingCotask;
//Antilatency::Alt::Tracking::IEnvironment environment;
//Antilatency::Math::floatP3Q placement;

bool exitRequested = false;

/*
BOOL WINAPI consoleHandler(DWORD signal)
{
	if (signal == CTRL_C_EVENT)
	{
		exitRequested = true;
		return FALSE;
	}
	else if (signal == CTRL_CLOSE_EVENT || signal == CTRL_BREAK_EVENT)
	{
		exitRequested = true;
		return FALSE;
	}

	return FALSE;
}

void StopTrackingTask()
{
	if (trackingCotask != nullptr)
	{
		trackingCotask = {};
	}
}

Antilatency::DeviceNetwork::NodeHandle GetTrackingNode()
{
	auto result = Antilatency::DeviceNetwork::NodeHandle::Null;

	auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();

	auto nodes = cotaskConstructor.findSupportedNodes(deviceNetwork);
	if (!nodes.empty())
	{
		for (auto node : nodes)
		{
			if (deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle)
			{
				result = node;
				break;
			}
		}
	}

	return result;
}

void RunTrackingTask(Antilatency::DeviceNetwork::NodeHandle node)
{


	while (trackingCotask != nullptr && !trackingCotask.isTaskFinished())
	{
		if (exitRequested)
		{
			break;
		}

		//Get raw tracker state
		auto state = trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
		std::cout << "Raw position x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;

		//Get extrapolated tracker state with placement correction
		auto extrapolatedState = trackingCotask.getExtrapolatedState(placement, 0.06f);
		std::cout << "Extrapolated position x: " << extrapolatedState.pose.position.x << ", y: " << extrapolatedState.pose.position.y << ", z: " << extrapolatedState.pose.position.z << std::endl;

		std::cout << "Current tracking stage: " << (int32_t)extrapolatedState.stability.stage << std::endl;

		//5 FPS pose printing
		Sleep(200);
	}

	trackingCotask = {};
}

bool altInit(uint32_t& updateId, Antilatency::DeviceNetwork::NodeHandle& trackingNode)
{
	if (!SetConsoleCtrlHandler(consoleHandler, TRUE))
	{
		printf("\nERROR: Could not set control handler");
		return 0;
	}

#ifdef _WIN64
	SetDllDirectory("../../bin/win/x64");
#else
	SetDllDirectory("../../bin/win/x86");
#endif

	//Load libraries
	auto adnLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("AntilatencyDeviceNetwork");
	auto antilatencyStorageClient = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::StorageClient::ILibrary>("AntilatencyStorageClient");
	altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>("AntilatencyAltTracking");

	if (adnLibrary == nullptr)
	{
		std::cout << "Failed to load AntilatencyDeviceNetwork library" << std::endl;
		return 0;
	}

	if (antilatencyStorageClient == nullptr)
	{
		std::cout << "Failed to load AntilatencyStorageClient library" << std::endl;
		return 0;
	}

	if (altTrackingLibrary == nullptr)
	{
		std::cout << "Failed to load AntilatencyAltTracking library" << std::endl;
		return 0;
	}

	//For Android devices you have to call these functions FIRST before using any library functions passing JavaVM and your application context as jobject
#ifdef _ANDROID_
	auto vm = nullptr; //Pointer to JavaVM
	auto context = nullptr; //Your application context

	auto adnJni = adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
	adnJni.initJni(vm, context);

	auto altSystemClientJni = antilatencyStorageClient.queryInterface<AndroidJniWrapper::IAndroidJni>();
	altSystemClientJni.initJni(vm, context);
#endif

	//Set log verbosity level for Antilatency Device Network library.
	adnLibrary.setLogLevel(Antilatency::DeviceNetwork::LogLevel::Info);

	std::cout << "ADN version: " << adnLibrary.getVersion() << std::endl;

	//Alt socket USB device ID
	Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceType;
	antilatencyUsbDeviceType.pid = 0x0000;
	antilatencyUsbDeviceType.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;

	//Alt socket USB device ID (deprecated)
	Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceTypeLegacy;
	antilatencyUsbDeviceTypeLegacy.pid = 0x0000;
	antilatencyUsbDeviceTypeLegacy.vid = Antilatency::DeviceNetwork::UsbVendorId::AntilatencyLegacy;

	deviceNetwork = adnLibrary.createNetwork(std::vector<Antilatency::DeviceNetwork::UsbDeviceType>{antilatencyUsbDeviceType, antilatencyUsbDeviceTypeLegacy});

	auto environmentCode = antilatencyStorageClient.getLocalStorage().read("environment", "default");
	auto placementCode = antilatencyStorageClient.getLocalStorage().read("placement", "default");

	environment = altTrackingLibrary.createEnvironment(environmentCode);
	placement = altTrackingLibrary.createPlacement(placementCode);

	auto markers = environment.getMarkers();
	std::cout << "Environment markers count: " << markers.size() << std::endl;
	for (auto i = 0; i < markers.size(); ++i)
	{
		std::cout << "Marker " << i << ": {" << markers[i].x << ", " << markers[i].y << ", " << markers[i].z << "}" << std::endl;
	}


}
*/
#define PORT     8080
#define BUFSIZE 1024
using namespace std;

// From http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/


void LoadImages(const string& strImagePath, const string& strPathTimes,
	vector<string>& vstrImages, vector<double>& vTimeStamps)
{
	ifstream fTimes;

	fTimes.open(strPathTimes);
	if (!fTimes.is_open())
	{
		cout << "can not open file with timeStep in path: " << strPathTimes;
		getchar();
		exit(1);
	}
	vTimeStamps.reserve(5000);
	vstrImages.reserve(5000);
	cout << "Start Lopp Load";
	while (!fTimes.eof())
	{
		string s;
		getline(fTimes, s);
		if (!s.empty())
		{
			cout << "Image string: " << s << endl;
			stringstream ss;
			ss << s;
			cout << "Load image: " << strImagePath + "/" + ss.str() + ".jpeg" << endl;
			vstrImages.push_back(strImagePath + "/" + ss.str() + ".jpeg");
			double t;
			ss >> t;
			vTimeStamps.push_back(t / 1e9);

		}
		else
		{
			//cout << "Empty string: " << s << endl;
		}
	}
	cout << "Images loaded" << endl;
}

void UDPSend(string message)
{
	//Initialize WinSocket
	WSAData data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(MAKEWORD(2, 2), &data);
	if (wsResult != 0)
	{
		cerr << "WSAStartup failed, Err #" << wsResult << endl;
		cin.get();
		return ;
	}

	//Creating Socket
	SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET)
	{
		cerr << "Can't create a socket, Err #" << WSAGetLastError() << endl;
		cin.get();
		WSACleanup();
		return ;
	}

	int OptVal = 1;
	int OptLen = sizeof(int);
	int rc = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&OptVal, OptLen);
	if (rc == SOCKET_ERROR)
	{
		cerr << "Can't set a sockopt, Err #" << WSAGetLastError() << endl;
		cin.get();
		WSACleanup();
		return ;
	}
	else
		cout << "Socket option SO_REUSEADDR is ON" << endl;

	//Prompt to type an IP of server
	//cout << "IP adress of server : ";
	string ipAdress="172.16.119.227";
	string ipAdressMy = "172.16.119.64";
	//getline(cin, ipAdress);

	//Fill in server and client structures
	sockaddr_in server_addr, client_addr;

	int server_len = sizeof(server_addr);
	ZeroMemory(&server_addr, server_len);
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	inet_pton(AF_INET, ipAdressMy.c_str(), &server_addr.sin_addr);

	int client_len = sizeof(client_addr);
	ZeroMemory(&client_addr, client_len);
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(0);
	client_addr.sin_addr.S_un.S_addr = INADDR_ANY;

	 bind(sock, (struct sockaddr*) & client_addr, client_len);
	/*if (rc == SOCKET_ERROR)
	{
		cerr << "Can't bind a socket, Err #" << WSAGetLastError() << endl;
		cin.get();
		WSACleanup();
		return 0;
	}*/

	//Do-While Loop to send and recieve data

	string userInput= message;


	//Send the text
	int sendResult = sendto(sock, userInput.c_str(), userInput.length(), 0, (sockaddr*)&server_addr, server_len);
	if (sendResult == SOCKET_ERROR)
	{
		cerr << "Can't send msg, Err #" << WSAGetLastError() << endl;
		cin.get();
		return;
	}



	closesocket(sock);
	WSACleanup();
	return;

}

Eigen::Vector4d hamiltonProduct(Eigen::Vector4d a, Eigen::Vector4d b)
{

	Eigen::Vector4d  c;

	c[0] = (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]);
	c[1] = (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]);
	c[2] = (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]);
	c[3] = (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]);

	return c;
}

std::vector<float> GrabTranformCamera(cv::Mat Poses, Eigen::Vector3d& translation)
{
	vector<float> null;
	if (Poses.empty())
		return null;

	Eigen::Matrix3d tf3d;
	tf3d(0, 0) = Poses.at<float>(0,0);
	tf3d(0, 1) = Poses.at<float>(0, 1);
	tf3d(0, 2) = Poses.at<float>(0, 2);

	tf3d(1, 0) = Poses.at<float>(1, 0);
	tf3d(1, 1) = Poses.at<float>(1,1);
	tf3d(1, 2) = Poses.at<float>(1, 2);

	tf3d(2, 0) = Poses.at<float>(2, 0);
	tf3d(2, 1) = Poses.at<float>(2, 1);
	tf3d(2, 2) = Poses.at<float>(2, 2);

	cv::Mat cv_tf3d;

	cv::eigen2cv(tf3d, cv_tf3d);
	vector<float> tfqt=ORB_SLAM2::Converter::toQuaternion(cv_tf3d);
	double aux = tfqt[0];
	tfqt[0] = -tfqt[2];
	tfqt[2] = tfqt[1];
	tfqt[1] = aux;

	Eigen::Vector3d origin(Poses.at<float>(0, 3), Poses.at<float>(1, 3), Poses.at<float>(2, 3));
	
	Eigen::Matrix3d rotate270degXZ;
	
	rotate270degXZ(0, 0) = 0;
	rotate270degXZ(0, 1) = 1;
	rotate270degXZ(0, 2) = 0;

	rotate270degXZ(1, 0) = 0;
	rotate270degXZ(1, 1) = 0;
	rotate270degXZ(1, 2) = 1;

	rotate270degXZ(2, 0) = -1;
	rotate270degXZ(2, 1) = 0;
	rotate270degXZ(2, 2) = 0;

	
	
	
	
	auto translationForCamera = origin.transpose() * rotate270degXZ;
	
	
	Eigen::Vector4d quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
	Eigen::Vector4d secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
	Eigen::Vector4d translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);
	

	
	Eigen::Vector4d translationStepQuat;
	translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

	
	Eigen::Vector3d _translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);
	
	translation = _translation;
	
	return tfqt;
}

int main(int argc, char** argv)
{
	std::ofstream outPos;
	std::ofstream initPoseAlt;
	outPos.open("trajectory.txt");
	string vocabPath = "D:/test_ORB_SLAM/orbslam-windows/Examples/Monocular/ORBvoc.txt";
	string settingsPath = "D:/test_ORB_SLAM/orbslam-windows/Examples/Monocular/webcam.yaml";
	bool SaveMapMode=false;
	if (argc == 2)
	{
		SaveMapMode = (bool)atoi(argv[1]);
		std::cout << "AAA";
		//vocabPath = argv[1];
	}
	else
	{
		cerr << endl << "Usage: mono_webcam.exe path_to_vocabulary path_to_settings" << endl;
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true, SaveMapMode);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	string realtime;
	cout << "Realtime Y/N:\n";
	cin >> realtime;
	bool haveActiveTracker = false;
	bool InitPose = false;

	initPoseAlt.open("initPose");


	if (realtime == "Y")
	{

		/*
#pragma region altTrack
		uint32_t updateId = 0;
		Antilatency::DeviceNetwork::NodeHandle trackingNode;

		altInit(updateId, trackingNode);

		trackingNode = GetTrackingNode();
		auto newUpdateId = deviceNetwork.getUpdateId();

		if (haveActiveTracker == false)
		{
			if (updateId != newUpdateId)
			{
				updateId = newUpdateId;

				std::cout << "Factory update id has been incremented, searching for available tracking node..." << std::endl;

				trackingNode = GetTrackingNode();
				if (trackingNode != Antilatency::DeviceNetwork::NodeHandle::Null)
				{
					//Found tracking node
					auto nodeSerialNo = deviceNetwork.nodeGetStringProperty(deviceNetwork.nodeGetParent(trackingNode), Antilatency::DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
					std::cout << "Tracking node found, serial number: " << nodeSerialNo << std::endl;
					haveActiveTracker = true;
					auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
					trackingCotask = cotaskConstructor.startTask(deviceNetwork, trackingNode, environment);
					std::cout << "Run tracking" << std::endl;
				}
				else
				{
					haveActiveTracker = false;
					std::cout << "Tracking node not found." << std::endl;
				}
			}
		}
		if (haveActiveTracker == true)
		{
			//Get raw tracker state
			auto state = trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
			std::cout << "Raw position x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << std::endl;

			//Get extrapolated tracker state with placement correction
			auto extrapolatedState = trackingCotask.getExtrapolatedState(placement, 0.06f);
			std::cout << "Extrapolated position x: " << extrapolatedState.pose.position.x << ", y: " << extrapolatedState.pose.position.y << ", z: " << extrapolatedState.pose.position.z << std::endl;

			std::cout << "Current tracking stage: " << (int32_t)extrapolatedState.stability.stage << std::endl;

		}
#pragma endregion Alttrack
*/
		cout << "input cam id..." << endl;
		int camId = 0;
		cin >> camId;
		cv::VideoCapture cap(camId);
		if (!cap.isOpened())
		{
			cout << "Dont open cam ..." << endl;
		}
		else
		{
			cout << "open cam 0!!!" << endl;
		}// check if we succeeded

		// From http://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
		__int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();


		// Main loop
		cv::Mat im;
		cv::Mat Tcw;
		while (true)
		{
			
			cap.read(im);

			__int64 curNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

			// Pass the image to the SLAM system
			Tcw = SLAM.TrackMonocular(im, curNow / 1000.0);
			std::cout <<"NumPoints" <<SLAM.GetMap()->GetNumMapPoints();

			// This can write each image with its position to a file if you want
			if (!Tcw.empty())
			{
				string messageUDP="";
				cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

				Eigen::Vector3d rot = ORB_SLAM2::Converter::toVector3d(Rwc);


				Eigen::Vector3d pos = ORB_SLAM2::Converter::toVector3d(twc);
				 vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
				string separ=",";
				string separCol=";";
				//auto state = trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
				//alt init pose write to file
				//if (!InitPose)
				//{
					//InitPose = true;
					
					//initPoseAlt << "x: " << state.pose.position.x << ", y: " << state.pose.position.y << ", z: " << state.pose.position.z << ",rot x:" << state.pose.rotation.x << ", rot y: " << state.pose.rotation.y << ", rot z:" << state.pose.rotation.z << std::endl;
				//}

				//Eigen::Vector3d translation;
				
				//std::vector<float> vectorRot=GrabTranformCamera(Tcw, translation);

				Eigen::Matrix4d transfromMatrix;
				transfromMatrix(0,0) = Rwc.at<float>(0, 0);
				transfromMatrix(1, 0) = Rwc.at<float>(1, 0);
				transfromMatrix(2, 0) = Rwc.at<float>(2, 0);
				transfromMatrix(3, 0) = 0.0;

				transfromMatrix(0, 1) = Rwc.at<float>(0, 1);
				transfromMatrix(1, 1) = Rwc.at<float>(1, 1);
				transfromMatrix(2, 1) = Rwc.at<float>(2, 1);
				transfromMatrix(3, 1) = 0.0;

				transfromMatrix(0, 2) = Rwc.at<float>(0, 2);
				transfromMatrix(1, 2) = Rwc.at<float>(1, 2);
				transfromMatrix(2, 2) = Rwc.at<float>(2, 2);
				transfromMatrix(3, 2) = 0.0;

				transfromMatrix(0, 3) = twc.at<float>(0);
				transfromMatrix(1, 3) = twc.at<float>(1);
				transfromMatrix(2, 3) = twc.at<float>(2);
				transfromMatrix(3, 3) = 1.0;





				/*Eigen::Vector3d translation;
				std::cout <<"tranclation" <<translation<<std::endl;
				std::cout << "rotation" << vectorRot[0] << std::endl;*/

				
				
				messageUDP += std::to_string(transfromMatrix(0, 0));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(1, 0));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(2, 0));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(3, 0));
				messageUDP += separCol;
				
				messageUDP += std::to_string(transfromMatrix(0, 1));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(1, 1));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(2, 1));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(3, 1));
				messageUDP += separCol;
				
				messageUDP += std::to_string(transfromMatrix(0, 2));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(1, 2));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(2, 2));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(3, 2));
				messageUDP += separCol;
				
				messageUDP += std::to_string(transfromMatrix(0, 3));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(1, 3));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(2, 3));
				messageUDP += separ;
				messageUDP += std::to_string(transfromMatrix(3, 3));
				messageUDP += separCol;

				UDPSend(messageUDP);



			}


			// This will make a third window with the color images, you need to click on this then press any key to quit
			cv::imshow("Image", im);


			if (cv::waitKey(1) >= 0)
			{
				getchar();
				break;
			}
		}

		// Stop all threads
		
		
		cap.release();
		// Save camera trajectory
		
	}
	else if(realtime=="N")
	{
		
		string strPathTime= "C:/Users/AAa/source/repos/CameraCapture/CameraCapture/timeStepsOnly.txt";
		string strImagePath= "C:/Users/AAa/source/repos/CameraCapture/images";

		//cout << "Input Image Path\n";
		//cin >> strImagePath;
		//cout << " Input Times Path\n";
		//cin >> strPathTime;


	

		vector<string> vstrImageFilenames;
		vector<double> vTimestamps;
		
		LoadImages(strImagePath, strPathTime, vstrImageFilenames, vTimestamps);
		

		int nImages = vstrImageFilenames.size();

		
		
		if (nImages <= 0)
		{
			cerr << "ERROR: Failed to load images" << endl;
			getchar();
			return 1;
		}
		

		// Vector for tracking time statistics
		vector<float> vTimesTrack;
		vTimesTrack.resize(nImages);
		cout << endl << "-------" << endl;


		cout << "Images in the sequence: " << nImages << endl << endl;
		cv::Mat im;
		for (int ni = 0; ni < nImages; ni++)
		{
			//getchar();
			// Read image from file
			im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
			double tframe = vTimestamps[ni];

			if (im.empty())
			{
				cerr << endl << "Failed to load image at: "
					<< vstrImageFilenames[ni] << endl;
				return 1;
			}

			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

			// Pass the image to the SLAM system
			cv::Mat Tcw = SLAM.TrackMonocular(im, tframe);


			if (!Tcw.empty())
			{


				cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
				cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);
				Eigen::Vector3d rot = ORB_SLAM2::Converter::toVector3d(Rwc);
				Eigen::Vector3d pos = ORB_SLAM2::Converter::toVector3d(twc);
				// vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
				cout << "X:" << pos.x() << "\nY:" << pos.y() << "\nZ:" << pos.z() << "\n\n";
				//cout << "X:" << rot.x() << "\nY:" << rot.y() << "\nZ:" << rot.z() << "\n\n";

				char sep = ';';
				outPos << pos.x() << sep << pos.y() << sep << pos.z() << sep << rot.x() << sep << rot.y() << sep << rot.z() << "\n";

				

				//vector<float> rotate = ORB_SLAM2::Converter::toQuaternion(Rwc);

				//std::ostringstream stream;
				//stream << "imgs/" << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
				//	Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
					//Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << ".jpg";
				//stream << "imgs/" << curNow << ".jpg";
				//string fileName = stream.str();
				//cv::imwrite(fileName, im);
			}



			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

			double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

			vTimesTrack[ni] = ttrack;

			// Wait to load the next frame
			double T = 0;
			if (ni < nImages - 1)
				T = vTimestamps[ni + 1] - tframe;
			else if (ni > 0)
				T = tframe - vTimestamps[ni - 1];

			if (ttrack < T)
				std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack) * 1e6)));
		}
		

	}
	outPos.close();
	SLAM.Shutdown();
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	getchar();
	return 0;
}
