// Copyright (c) 2020 ALT LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of source code located below and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//  
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//  
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "AltTrackingExample.h"

#include <unistd.h>
#include <sstream>
#include <memory>

#include <Antilatency.InterfaceContract.LibraryLoader.h>

std::unique_ptr<AltTrackingExample> trackingInstance;

JNIEXPORT void JNICALL
Java_com_antilatency_alttrackingandroid_MainActivity_Init(JNIEnv* env, jobject instance){
    if(trackingInstance == nullptr){
        trackingInstance = std::make_unique<AltTrackingExample>(env, instance);
    }
}

JNIEXPORT jstring JNICALL
Java_com_antilatency_alttrackingandroid_MainActivity_GetOutput(JNIEnv* env, jobject instance){
    if(trackingInstance == nullptr){
        return (jstring)"Fail";
    }

    return env->NewStringUTF((trackingInstance->readOutput()).c_str());
}

AltTrackingExample::AltTrackingExample(JNIEnv* env, jobject instance){
    //Load libraries.
    _adnLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("libAntilatencyDeviceNetwork.so");
    if(!_adnLibrary){
        throw std::runtime_error("Failed to load AntilatencyDeviceNetwork library");
    }

    _antilatencyStorageLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::StorageClient::ILibrary>("libAntilatencyStorageClient.so");
    if(!_antilatencyStorageLibrary){
        throw std::runtime_error("Failed to load AntilatencyStorageClient library");
    }

    _altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>("libAntilatencyAltTracking.so");
    if(!_altTrackingLibrary){
        throw std::runtime_error("Failed to load AntilatencyAltTracking library");
    }

    JavaVM* jvm;
    env->GetJavaVM(&jvm);

    //Before using any libraries methods, you have to call this methods on Android.
    auto adnJni = _adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    adnJni.initJni(jvm, instance);

    auto antilatencyStorageClientJni = _antilatencyStorageLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    antilatencyStorageClientJni.initJni(jvm, instance);

    //Set log verbosity level for Antilatency Device Network library.
    _adnLibrary.setLogLevel(Antilatency::DeviceNetwork::LogLevel::Info);

    writeOutput("Antilatency Device Network ver.: " + _adnLibrary.getVersion());

    //Alt socket USB device ID
    Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceType;
    antilatencyUsbDeviceType.pid = 0x0000;
    antilatencyUsbDeviceType.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;

    //Alt socket USB device ID (deprecated)
    Antilatency::DeviceNetwork::UsbDeviceType antilatencyUsbDeviceTypeLegacy;
    antilatencyUsbDeviceTypeLegacy.pid = 0x0000;
    antilatencyUsbDeviceTypeLegacy.vid = Antilatency::DeviceNetwork::UsbVendorId::AntilatencyLegacy;

    _deviceNetwork = _adnLibrary.createNetwork(std::vector<Antilatency::DeviceNetwork::UsbDeviceType>{antilatencyUsbDeviceType, antilatencyUsbDeviceTypeLegacy});

    //Get environment code from AltSystem app (default in this case) and create environment from it.
    auto environmentCode = _antilatencyStorageLibrary.getLocalStorage().read("environment", "default");
//    std::string environmentCode = "AAVSaWdpZBcABnllbGxvdwQEBAABAQMBAQEDAAEAAD_W";
    _environment = _altTrackingLibrary.createEnvironment(environmentCode);
    if(_environment){
        auto markers = _environment.getMarkers();
        writeOutput("Environment has been created, markers count: " + std::to_string(markers.size()));
        for(auto marker : markers){
            writeOutput("Marker: { " + std::to_string(marker.x) + ", " + std::to_string(marker.y) + ", " + std::to_string(marker.z) + "}");
        }
    }else{
        throw std::runtime_error("Failed to create environment.");
    }

    //Get placement code from AltSystem app (default in this case) and create placement from it.
    auto placementCode = _antilatencyStorageLibrary.getLocalStorage().read("placement", "default");
//    std::string placementCode = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHye";
    _placement = _altTrackingLibrary.createPlacement(placementCode);

    writeOutput("Placement offset: (" + std::to_string(_placement.position.x) + ", " + std::to_string(_placement.position.y) + ", " + std::to_string(_placement.position.z) + ")");
    writeOutput("Placement rotation: (" + std::to_string(_placement.rotation.x) + ", " + std::to_string(_placement.rotation.y) + ", " + std::to_string(_placement.rotation.z) + ", " + std::to_string(_placement.rotation.w) + ")");

    _trackingThreadRunning = true;
    _trackingThread = std::thread{&AltTrackingExample::doTracking, this};
}

AltTrackingExample::~AltTrackingExample(){
    _trackingThreadRunning = false;
    _trackingThread.join();
}

void AltTrackingExample::doTracking(){
    while (_trackingThreadRunning){
        if(_trackingCotask && _trackingCotask.isTaskFinished()){
            _trackingCotask = {};
        }

        if(!_trackingCotask){
            //Get current Antilatency Device Network update id. It will be incremented every time any supported device is added or removed.
            auto updateId = _deviceNetwork.getUpdateId();
            if(updateId != _updateId){
                _updateId = updateId;
                writeOutput("Antilatency Device Network update id has been incremented: " + std::to_string(_updateId));
                writeOutput("Searching for tracking nodes...");

                //Create tracking cotask constructor to check if node supports tracking and start tracking task on node.
                auto cotaskConstructor = _altTrackingLibrary.createTrackingCotaskConstructor();

                //Get all currently connected nodes that supports tracking task.
                auto nodes = cotaskConstructor.findSupportedNodes(_deviceNetwork);

                for(auto node : nodes){
                    //Check if node is idle, we cannot start task on invalid nodes or on nodes that already has task started on it.
                    if(_deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle){
                        std::stringstream ss;
                        ss << static_cast<std::underlying_type<Antilatency::DeviceNetwork::NodeHandle>::type>(node);
                        writeOutput("Tracking node found, node: " + ss.str());
                        _trackingCotask = cotaskConstructor.startTask(_deviceNetwork, node, _environment);

                        break;
                    }
                }

                if(!_trackingCotask){
                    writeOutput("Tracking node not found");
                }
            }
        }

        if(_trackingCotask){
            //Get raw tracker state
            auto state = _trackingCotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);

            writeOutput("Tracking stage: " + std::to_string(static_cast<int32_t>(state.stability.stage)));
            writeOutput("Raw position: (" + std::to_string(state.pose.position.x) + ", " + std::to_string(state.pose.position.y) + ", " + std::to_string(state.pose.position.z) + ")");

            //Get extrapolated tracker state with placement correction
            auto extrapolatedState = _trackingCotask.getExtrapolatedState(_placement, extrapolationTime);
            writeOutput("Extrapolated position: (" + std::to_string(extrapolatedState.pose.position.x) + ", " + std::to_string(extrapolatedState.pose.position.y) + ", " + std::to_string(extrapolatedState.pose.position.z) + ")");
        }

        sleep(1);
    }
}

std::string AltTrackingExample::readOutput(){
    std::lock_guard<std::mutex> lock(_outputMutex);
    auto output = _output;
    _output = "";
    return output;
}

void AltTrackingExample::writeOutput(const std::string &text){
    std::lock_guard<std::mutex> lock(_outputMutex);
    _output += text + "\n";
}