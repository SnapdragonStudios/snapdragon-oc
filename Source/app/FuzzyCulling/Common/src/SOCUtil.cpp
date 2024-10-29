//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
#include "../SOCUtil.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#if defined(SDOC_ANDROID_ARM)
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <unistd.h>
#include <fstream>
#elif defined(SDOC_NATIVE)
#include <fstream>
#include <iostream>
#endif

#if defined(SDOC_WIN)
#pragma warning( disable : 4996  )
#endif
#include "../../Util/RapidRasterizer/RapidRasterizer.h"



SOCLogger SOCLogger::Singleton;

void SOCLogger::AddLog(std::string s)
{
    if(mStoreMsg){
        const std::lock_guard<std::mutex> lock(mMutex);
		if (mlogs.size() < 16) //at most cache 16 logs 
		{
			this->mlogs.push_back(s);
		}
    }
}

bool SOCLogger::GetLog(char* output)
{
	const std::lock_guard<std::mutex> lock(mMutex);
	if (mlogs.size() <= mReadIdx)
	{
		return false;
	}
	std::string data = mlogs[mReadIdx];

	mReadIdx++;
	if (mlogs.size() == mReadIdx) 
	{
		mlogs.clear();
		mReadIdx = 0;
	}

	int endIndex = std::min<int>(255, (int)data.length());
	strncpy(output, data.c_str(), endIndex);
	output[endIndex] = '\0';
	return true;
}

namespace SDOCCommon
{
    void SOCFrameInfo::startNewFrame()
	{
        FrameCounter++;
    }

#if defined( SUPPORT_ALL_FEATURE) && defined( SDOC_NATIVE_DEBUG) && defined( SDOC_NATIVE)
    void SOCFrameInfo::submitOccluder(const float *vertices, const unsigned short *indices,
                                      unsigned int nVert, unsigned int nIdx,
                                      const float *localToWorld, bool backfaceCull)
	{
		{
			if (indices == nullptr) {
				this->m_rapidRasterizer->SubmitBakedOccluder((unsigned short*)vertices, localToWorld);
			}
			else {
				this->m_rapidRasterizer->SubmitRawOccluder(vertices, indices, nVert, nIdx, localToWorld, backfaceCull);
			}
		}

		if (this->mCaptureFrame) 
		{
			RecordOccluder(vertices, indices, nVert, nIdx, localToWorld, backfaceCull, "");
		}
	}
	bool SOCFrameInfo::queryOccludeeMesh(const float* vertices, const unsigned short* indices,
		unsigned int nVert, unsigned int nIdx,
		const float* localToWorld, bool backfaceCull)
	{
		bool result =	this->m_rapidRasterizer->QueryRawOccludee(vertices, indices, nVert, nIdx, localToWorld, backfaceCull, nullptr);
		
		if (this->mCaptureFrame)
		{
			RecordOccluder(vertices, indices, nVert, nIdx, localToWorld, backfaceCull, "");
		}
		return result;
	}
#endif



    void SOCFrameInfo::recordOccludee(const float *vertices, unsigned int num, bool obbQuery)
    {
		if (mFileWriter == nullptr) {
			return;
		}

		const std::lock_guard<std::mutex> lock(mWriteMutex);
		// batch queries
		if(obbQuery == false)
			fprintf(mFileWriter, "%s\n", BATCHED_OCE_HEADER.c_str());
		else
			fprintf(mFileWriter, "%s\n", BATCHED_OCE_HEADER_OBB.c_str());
		fprintf(mFileWriter, "%d\n", num);
		const float * boxPtr = vertices;
		if (obbQuery == false) {
			for (uint32_t i = 0; i < num; ++i, boxPtr += BBOX_STRIDE)
			{
				fprintf(mFileWriter, "%.9g %.9g %.9g\n%.9g %.9g %.9g\n",
					boxPtr[0], boxPtr[1], boxPtr[2],
					boxPtr[3], boxPtr[4], boxPtr[5]);
			}
		}
		else {
			//record 18 float
			for (uint32_t i = 0; i < num; ++i, boxPtr += BBOX_STRIDE)
			{
				fprintf(mFileWriter, "%.9g %.9g %.9g\n%.9g %.9g %.9g\n",
					boxPtr[0], boxPtr[1], boxPtr[2],
					boxPtr[3], boxPtr[4], boxPtr[5]);
				boxPtr += BBOX_STRIDE;
				fprintf(mFileWriter, "%.9g %.9g %.9g\n%.9g %.9g %.9g\n",
					boxPtr[0], boxPtr[1], boxPtr[2],
					boxPtr[3], boxPtr[4], boxPtr[5]);
				boxPtr += BBOX_STRIDE;
				fprintf(mFileWriter, "%.9g %.9g %.9g\n%.9g %.9g %.9g\n",
					boxPtr[0], boxPtr[1], boxPtr[2],
					boxPtr[3], boxPtr[4], boxPtr[5]);
			}
		}
	}

	void SOCFrameInfo::StopFrameCapture()
	{
		LOGI("StopFrameCapture");
		if (mFileWriter != nullptr)
		{
			fclose(mFileWriter);
			mFileWriter = nullptr;
		}
	}


	SOCFrameInfo::~SOCFrameInfo()
	{
	}

	SOCFrameInfo::SOCFrameInfo()
	{
		memset(ViewProjArray, 0, 16 * sizeof(float));
	}


	void SOCFrameInfo::StartRecordFrame(const float * InCameraPos, const float * ViewDir, const float * vp) 
	{
		const std::lock_guard<std::mutex> lock(mWriteMutex);
		// get name and open the file
		std::string outputDir = SOCLogger::GetOutputDirectory();

		time_t now = time(0);
		tm* ltm = localtime(&now);
		int year = 1900 + ltm->tm_year;
		int month = 1 + ltm->tm_mon;
		int day = ltm->tm_mday;
		int hour = ltm->tm_hour;
		int minute = ltm->tm_min;
		int sec = ltm->tm_sec;
		std::string timeStamp =
			std::to_string(year) + "_"
			+ std::to_string(month) + "_"
			+ std::to_string(day) + "_"
			+ std::to_string(hour) + "_"
			+ std::to_string(minute) + "_"
			+ std::to_string(sec) ;


		std::string file_name = outputDir + CAPTURE_PREFIX +
			std::to_string(this->FrameCounter) +"_" + timeStamp + CAPTURE_APPENDIX;
		if (mOutputSaveCap != "") 
		{
			file_name = outputDir + mOutputSaveCap;
		}



		mFileWriter = fopen(file_name.c_str(), "w");
		if (mFileWriter == nullptr)
		{
			return ;
		}
		LOGI("Saving %s", file_name.c_str());

		this->mIsRecording = true;

		// width, height, nearplane
		fprintf(mFileWriter, "%s\n", FB_SETTING_HEADER.c_str());
		int cw = m_rapidRasterizer->GetCW() * 1000000;
		fprintf(mFileWriter, "%d %d %.9g\n", this->Width, this->Height + cw, this->NearPlane);

		fprintf(mFileWriter, "Frame\n");
		fprintf(mFileWriter, "1\n");
		fprintf(mFileWriter, "%s\n", CAM_POS_HEADER.c_str());
		fprintf(mFileWriter, "%.9g %.9g %.9g %.9g %.9g %.9g\n", InCameraPos[0], InCameraPos[1], InCameraPos[2], ViewDir[0], ViewDir[1], ViewDir[2]);

		fprintf(mFileWriter, "%s\n", VIEW_PROJ_HEADER.c_str());

		fprintf(mFileWriter, "%.9g %.9g %.9g %.9g\n", vp[0], vp[1], vp[2], vp[3]);
		fprintf(mFileWriter, "%.9g %.9g %.9g %.9g\n", vp[4], vp[5], vp[6], vp[7]);
		fprintf(mFileWriter, "%.9g %.9g %.9g %.9g\n", vp[8], vp[9], vp[10], vp[11]);
		fprintf(mFileWriter, "%.9g %.9g %.9g %.9g\n", vp[12], vp[13], vp[14], vp[15]);

		
		
	}

	void SOCFrameInfo::RecordOccluder(const float * vertices, const unsigned short * indices, unsigned int nVert, unsigned int nIdx, const float * localToWorld, int backfaceCull, std::string prefix)
	{
		const std::lock_guard<std::mutex> lock(mWriteMutex);
		auto fptr = mFileWriter;

		uint32_t nFace = nIdx/3;

		if (nVert == 0 && nIdx == 0)
		{

			uint16_t *meta = (uint16_t *)vertices;

			SDOCCommon::OccluderMesh raw;
			raw.QuadSafeBatchNum = meta[2]; 
			raw.TriangleBatchIdxNum =meta[3]; 

			
			int idxNum = raw.QuadSafeBatchNum * 16 + raw.TriangleBatchIdxNum * 12;
			int vertNum = meta[1];
			if (vertNum <= SDOCCommon::SuperCompressVertNum)
			{
				idxNum >>= 1;
			}

			//uint16_t *pIdx = meta + 16; //first 16 uint16 for metadata and aabb
			
			int vertSize = vertNum * 6; //currently save vert as float, rather than uint16

			int total16 = vertSize + idxNum + 16;

			int line8 = total16 / 8;
			int left = (total16 & 7);

			uint16_t * data = (uint16_t*)vertices;

			if (prefix == "Query") {
				fprintf(fptr, "QueryCompactOccluder\n");
			}
			else {
				fprintf(fptr, "CompactOccluder\n");
			}

			fprintf(fptr, "%d\n", (int)(total16 + 7) >> 3);

			for (int i_vert = 0; i_vert < line8; ++i_vert, data += 8)
			{
				fprintf(fptr, "%hu %hu %hu %hu %hu %hu %hu %hu\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
			}
			if (left > 0) 
			{
				for (int i_vert =0; i_vert < left; ++i_vert, data++)
				{
					fprintf(fptr, "%hu ", data[0]);
				}
				for (int i_vert = left; i_vert <8; ++i_vert)
				{
					fprintf(fptr, "0 ");
				}
				fprintf(fptr, "\n");
			}
			
		}
		else {
			if (prefix == "Query") {
				fprintf(fptr, "QueryOccluder\n");
			}
			else {
				fprintf(fptr, "Occluder\n");
			}

			int backfaceCullOff = !backfaceCull;
			fprintf(fptr, "%d %d\n", (int)nVert + (backfaceCullOff * 10000000), (int)nFace); //encode backface cull property into occluder record

			for (uint32_t i_vert = 0; i_vert < nVert; ++i_vert)
			{
				fprintf(fptr, "%.9g %.9g %.9g\n", vertices[i_vert * 3 + 0], vertices[i_vert * 3 + 1], vertices[i_vert * 3 + 2]);
			}

			for (uint32_t i_face = 0; i_face < nFace; ++i_face)
			{
				fprintf(fptr, "%d %d %d\n", indices[i_face * 3 + 0], indices[i_face * 3 + 1], indices[i_face * 3 + 2]);
			}
		}

		if (localToWorld == nullptr) {
			fprintf(fptr, "%.9g %.9g %.9g %.9g\n", 1.0, 0.0, 0.0, 0.0);
			fprintf(fptr, "%.9g %.9g %.9g %.9g\n", 0.0, 1.0, 0.0, 0.0);
			fprintf(fptr, "%.9g %.9g %.9g %.9g\n", 0.0, 0.0, 1.0, 0.0);
			fprintf(fptr, "%.9g %.9g %.9g %.9g\n", 0.0, 0.0, 0.0, 1.0);
		}
		else {
			const float* pose = localToWorld;
			for (int i = 0; i < 4; i++, pose += 4)
			{
				fprintf(fptr, "%.9g %.9g %.9g %.9g\n", pose[0], pose[1], pose[2], pose[3]);
			}
		}
	}

} // namespace common
