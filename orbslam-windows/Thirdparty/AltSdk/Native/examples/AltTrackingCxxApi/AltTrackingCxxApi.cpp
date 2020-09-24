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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <Windows.h>

#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>
#include <Antilatency.StorageClient.h>
#include <Antilatency.Alt.Tracking.h>

Antilatency::DeviceNetwork::INetwork deviceNetwork;
Antilatency::Alt::Tracking::ILibrary altTrackingLibrary;
Antilatency::Alt::Tracking::ITrackingCotask trackingCotask;
Antilatency::Alt::Tracking::IEnvironment environment;
Antilatency::Math::floatP3Q placement;

bool exitRequested = false;

void StopTrackingTask() {
    if (trackingCotask != nullptr) {
        trackingCotask = {};
    }
}

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT) {
        exitRequested = true;
        return FALSE;
    } else if (signal == CTRL_CLOSE_EVENT || signal == CTRL_BREAK_EVENT) {
        exitRequested = true;
        return FALSE;
    }

    return FALSE;
}

//Returns the first idle alt tracker node just for demonstration purposes
Antilatency::DeviceNetwork::NodeHandle GetTrackingNode() {
    auto result = Antilatency::DeviceNetwork::NodeHandle::Null;

    auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();

    auto nodes = cotaskConstructor.findSupportedNodes(deviceNetwork);
    if (!nodes.empty()) {
        for (auto node : nodes) {
            if (deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {
                result = node;
                break;
            }
        }
    }

    return result;
}

//Run tracking task on node and print tracking data
void RunTrackingTask(Antilatency::DeviceNetwork::NodeHandle node) {
    auto cotaskConstructor = altTrackingLibrary.createTrackingCotaskConstructor();
    trackingCotask = cotaskConstructor.startTask(deviceNetwork, node, environment);

    while (trackingCotask != nullptr && !trackingCotask.isTaskFinished()) {
        if (exitRequested) {
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

int main() {
    if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
        printf("\nERROR: Could not set control handler");
        return 1;
    }

#ifdef _WIN64
	SetDllDirectory(L"../../bin/win/x64");
#else
	SetDllDirectory(L"../../bin/win/x86");
#endif

    //Load libraries
    auto adnLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("AntilatencyDeviceNetwork");
    auto antilatencyStorageClient = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::StorageClient::ILibrary>("AntilatencyStorageClient");
    altTrackingLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::Alt::Tracking::ILibrary>("AntilatencyAltTracking");

    if (adnLibrary == nullptr) {
        std::cout << "Failed to load AntilatencyDeviceNetwork library" << std::endl;
        return 1;
    }

    if (antilatencyStorageClient == nullptr) {
        std::cout << "Failed to load AntilatencyStorageClient library" << std::endl;
        return 1;
    }

    if (altTrackingLibrary == nullptr) {
        std::cout << "Failed to load AntilatencyAltTracking library" << std::endl;
        return 1;
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
    uint32_t updateId = 0;

    Antilatency::DeviceNetwork::NodeHandle trackingNode;

    auto environmentCode = antilatencyStorageClient.getLocalStorage().read("environment", "default");
    auto placementCode = antilatencyStorageClient.getLocalStorage().read("placement", "default");

    environment = altTrackingLibrary.createEnvironment(environmentCode);
    placement = altTrackingLibrary.createPlacement(placementCode);

    auto markers = environment.getMarkers();
    std::cout << "Environment markers count: " << markers.size() << std::endl;
    for (auto i = 0; i < markers.size(); ++i) {
        std::cout << "Marker " << i << ": {" << markers[i].x << ", " << markers[i].y << ", " << markers[i].z << "}" << std::endl;
    }

    while (true) {
        if (exitRequested) {
            break;
        }
        auto newUpdateId = deviceNetwork.getUpdateId();
        if (updateId != newUpdateId) {
            updateId = newUpdateId;
            
            std::cout << "Factory update id has been incremented, searching for available tracking node..." << std::endl;

            trackingNode = GetTrackingNode();
            if (trackingNode != Antilatency::DeviceNetwork::NodeHandle::Null) {
                //Found tracking node
                auto nodeSerialNo = deviceNetwork.nodeGetStringProperty(deviceNetwork.nodeGetParent(trackingNode), Antilatency::DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
                std::cout << "Tracking node found, serial number: " << nodeSerialNo << std::endl;
                RunTrackingTask(trackingNode);
            } else {
                std::cout << "Tracking node not found." << std::endl;
            }
        }
        Yield();
    }

    return 0;
}
