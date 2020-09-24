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

#ifndef ALTTRACKINGANDROID_ALTTRACKINGEXAMPLE_H
#define ALTTRACKINGANDROID_ALTTRACKINGEXAMPLE_H

#include <string>
#include <jni.h>
#include <thread>
#include <mutex>

#include "Antilatency.DeviceNetwork.h"
#include "Antilatency.StorageClient.h"
#include <Antilatency.Alt.Tracking.h>
#include "Antilatency.Api.h"

extern "C"{
    JNIEXPORT void JNICALL
    Java_com_antilatency_alttrackingandroid_MainActivity_Init(JNIEnv* env, jobject instance);

    JNIEXPORT jstring JNICALL
    Java_com_antilatency_alttrackingandroid_MainActivity_GetOutput(JNIEnv* env, jobject instance);
}

class AltTrackingExample {
public:
    AltTrackingExample(JNIEnv* env, jobject instance);
    ~AltTrackingExample();

    std::string readOutput();

    float extrapolationTime = 0.06f;

private:
    Antilatency::DeviceNetwork::ILibrary _adnLibrary;
    Antilatency::StorageClient::ILibrary _antilatencyStorageLibrary;
    Antilatency::Alt::Tracking::ILibrary _altTrackingLibrary;

    std::string _output;
    std::mutex _outputMutex;

    std::thread _trackingThread;
    bool _trackingThreadRunning = false;

    Antilatency::DeviceNetwork::INetwork _deviceNetwork;
    uint32_t _updateId = 0;

    Antilatency::Alt::Tracking::IEnvironment _environment;
    Antilatency::Math::floatP3Q _placement;
    Antilatency::Alt::Tracking::ITrackingCotask _trackingCotask;

    void writeOutput(const std::string &text);
    void doTracking();
};


#endif //ALTTRACKINGANDROID_ALTTRACKINGEXAMPLE_H
