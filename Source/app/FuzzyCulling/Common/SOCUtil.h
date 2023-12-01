//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#pragma once

#include "Vec3f.h"
#include "MathUtil.h"

#if defined(SDOC_NATIVE)
#include <assert.h>
#endif

#if defined(SDOC_WIN)
#include <direct.h> //for windows usage
#endif

#if defined(SDOC_NATIVE)
#include <fstream>
#include <iostream>
#include <sstream>
#endif

#ifdef SDOC_IOS
//#import <Foundation/Foundation.h>
//#include <CoreFoundation/CoreFoundation.h>
//extern "C"
//{
//	void NSLog(CFStringRef format, ...);
//}
//#define LOGI_IOS(format, ...) NSLog(CFSTR(format), ##__VA_ARGS__)
#endif

#ifdef __ANDROID__
#include <android/log.h>
#define LOGI_Android(...) __android_log_print(ANDROID_LOG_VERBOSE, "SDOC", __VA_ARGS__)
#endif

#include <iostream>
#include <stdarg.h>
#include <mutex>

class SOCLogger
{
public:
	static SOCLogger Singleton;
	SOCLogger() 
	{
		mlogs.reserve(8);
		mReadIdx = 0;
	}

	std::mutex mMutex;

	int mReadIdx;
	//some more code
	std::vector<std::string> mlogs;
	void AddLog(std::string s);
	bool GetLog(char* output);

	bool mStoreMsg = false;
	void SetStoreMsgConfig(bool store)
	{
		mStoreMsg = store;
	}


	std::string OutputDir = "";

	static bool IsPathValid(std::string pathname)
	{
		struct stat info;
		if (stat(pathname.c_str(), &info) != 0)
			return false;
		else if (info.st_mode & S_IFDIR)  // S_ISDIR() doesn't exist on my windows 
			return true;

		return false;
	}

	static std::string GetOutputDirectory()
	{
		std::string output = SOCLogger::Singleton.OutputDir;
		bool pathValid = true;
#if defined(SDOC_ANDROID)
		if (output.length() == 0)
		{
			output = "/sdcard/SOC/";
		}
		std::system(("mkdir -p " + output).c_str());
#elif defined(SDOC_WIN)
		if (output == "") {
			return "D:/";
		}
		else {
			_mkdir(output.c_str());
			pathValid = SOCLogger::IsPathValid(output);
		}
#elif defined(SDOC_OSX)
		//_mkdir(output.c_str());
        std::system(("mkdir " + output).c_str());
		pathValid = SOCLogger::IsPathValid(output);
#endif
		if (pathValid == false)
		{
			return "";
		}

		return output;
	}	

	void SetCaptureOutputPath(std::string output)
	{
		const std::lock_guard<std::mutex> lock(mMutex);
		this->OutputDir = output;
		if (this->mStoreMsg)
		{
			this->mlogs.push_back(output);
		}
	}

};


static void LOGI(const char* format, ...)
{
	char message[256];
	va_list args;
	va_start(args, format);
	vsnprintf(message, 256, format, args);
	va_end(args);

	if (SOCLogger::Singleton.mStoreMsg)
	{
		SOCLogger::Singleton.AddLog(std::string(message));
	}
	else
	{
#ifdef __ANDROID__
		LOGI_Android("%s", message);
#elif SDOC_IOS
        SOCLogger::Singleton.AddLog(std::string(message));
#else
		std::cout << message << std::endl;
#endif
	}
}

static inline void socAssert(bool b)
{
#if defined(SDOC_NATIVE)
	assert(b);
#else
	if (b == false) 
	{
		LOGI("soc assert fail");
	}
#endif
}
namespace util
{
	class RapidRasterizer;
} // namespace util

namespace common
{



	enum AlgoEnum
	{
		Rasterizer_FullTriangle = 1 << 2,

#if defined(SDOC_NATIVE_DEBUG)
		Rasterizer_FullTriangle2 = 1 << 3,
		Rasterizer_FullTriangle3 = 1 << 4,
		Rasterizer_FullTriangle4 = 1 << 5,
		Rasterizer_FullTriangle5 = 1 << 6,
#endif
	};



	enum DumpImageMode
	{
		DumpFull = 0,
#ifdef SDOC_NATIVE_DEBUG


		DumpHiz = 9,
		DumpBlockMask = 10,
		CheckerBoard_blackPattern = 11,
		CheckerBoard_whitePattern = 12,
		CheckerBoard_oddColumn = 13,
		CheckerBoard_evenColumn = 14,
		CheckerBoard_oddBlack = 15,
		CheckerBoard_evenBlack = 16,
		CheckerBoard_evenWhite = 17,
		CheckerBoard_oddWhite = 18,
		
#endif
	};


	struct OccluderMesh
	{
		unsigned int VerticesNum = 0;
		unsigned int QuadSafeBatchNum = 0;
		unsigned int TriangleBatchIdxNum = 0;
		uint16_t SuperCompress = 0;

		uint16_t EnableBackface = 0;
		const float *Vertices = nullptr;
		const uint16_t *Indices = nullptr;
	};

class SOCFrameInfo
{
public:
	void startNewFrame();
#if defined( SUPPORT_ALL_FEATURE)
	void submitOccluder( const float *vertices, const unsigned short *indices,
            unsigned int nVert, unsigned int nIdx, 
            const float *localToWorld, bool backfaceCull);
#endif

	void recordOccludee(const float *vertices, unsigned int num);


	// width, height & near plane
	uint32_t Width = 0;
	uint32_t Height = 0;
	float NearPlane = 0.0f;

	// boolean of Prev/Curr ViewProj
	bool IsSameCameraWithPrev = false;
	bool mCaptureFrame = false;

	uint64_t FrameCounter = START_FRAME_COUNT; //start from frame 2


	common::Vec3f mCameraViewDir;
	// Camera Position
	float CameraPos[3];


	
	float ViewProjArray[16];


	~SOCFrameInfo();
	SOCFrameInfo();


	util::RapidRasterizer* m_rapidRasterizer;

	void StopFrameCapture();
	std::string mOutputSaveCap="";
	bool mIsRecording = false;
public:


	void StartRecordFrame(const float * CameraPos, const float * ViewDir, const float * ViewProj) ;


	void RecordOccluder(const float * vertices, const unsigned short * indices, unsigned int nVert, unsigned int nIdx, const float * localToWorld, int backfaceCull);
	FILE *mFileWriter = nullptr;
};

} // namespace common
