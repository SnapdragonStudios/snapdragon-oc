//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
#include "API/SDOCAPI.h"
#include "SOCPrivate.h"
#include "Common/SOCUtil.h"
#include "Common/MathUtil.h"
#include "Util/RapidRasterizer/OccluderQuad.h"
#include "Util/RapidRasterizer/RapidRasterizer.h"

using namespace SOC;



void* sdocInit(unsigned int width, unsigned int height, float nearPlane)
{

	SOCPrivate *pSDOC = new SOCPrivate();
    if (pSDOC == nullptr)
    {
		//LOGI("sdocInit failed");
        return nullptr;
    }
	

    if (!pSDOC->socConfig(width, height, nearPlane))
    {
		//LOGI("sdocInit Config failed");

        // try to setup SOC, return false and destroy the instance if fails.
		pSDOC->socDestroy();
        return nullptr;
    }

#if defined(SDOC_STATIC)
	LOGI("Static SDOC Version %d.%d Started", common::VERSION_MAJOR, common::VERSION_SUB);
#else
	LOGI("Dynamic SDOC Version %d.%d Started", common::VERSION_MAJOR, common::VERSION_SUB);
#endif

	
	return pSDOC;
}


bool sdocStartNewFrame(void *pSDOC, const float *ViewPos, const float *ViewDir, const float *ViewProj)
{

	SOCPrivate *instance = (SOCPrivate *)pSDOC;
	if (instance == nullptr)
	{
		return false;
	}
    instance->startNewFrame(ViewPos, ViewDir, ViewProj);
    return true;
}
void sdocRenderBakedOccluder(void *pSDOC, unsigned short *compressedModel, const float *localToWorld)
{
	SOCPrivate *instance = (SOCPrivate *)pSDOC;
	if (instance == nullptr)
	{
		return;
	}	
	//direct call rapid rasterizer
	instance->m_rapidRasterizer->SubmitBakedOccluder(compressedModel, localToWorld);
	if (instance->m_frameInfo->mIsRecording)
	{
		bool backfaceCull = true; //this info is not used in backed model recording
		instance->m_frameInfo->RecordOccluder((float*)compressedModel, nullptr, 0, 0, localToWorld, backfaceCull);
	}
}
void sdocRenderOccluder(void*pSDOC, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx, const float *localToWorld, bool backfaceCull)
{

	SOCPrivate *instance = (SOCPrivate *)pSDOC;
	if (instance == nullptr)
	{
		return ;
	}
	//direct call rapid rasterizer
	bool validMesh = instance->m_rapidRasterizer->SubmitRawOccluder(vertices, indices, nVert, nIdx, localToWorld, backfaceCull);
	if (instance->m_frameInfo->mIsRecording)
	{
		if (validMesh)
		{
			instance->m_frameInfo->RecordOccluder(vertices, indices, nVert, nIdx, localToWorld, backfaceCull);
		}
	}
}

#if defined(SDOC_NATIVE)

unsigned short* sdocMeshBake( int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx,   float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint)
{
		if (outputCompressSize == nullptr) {

			util::OccluderQuad::OfflineTestClearBakeBuffer();			return nullptr;
		}

		unsigned short* data = util::OccluderQuad::sdocMeshLodBake(outputCompressSize, vertices, indices, nVert, nIdx, quadAngle, enableBackfaceCull, counterClockWise, TerrainGridAxisPoint);
		return data;
}


bool sdocMeshLod(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel)
{
	return util::OccluderQuad::sdocMeshSimplify(vertices, indices, nVert, nIdx, modelId, targetFaceNum, saveModel);
}

#else
unsigned short* sdocMeshBake(int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint)
{
	return nullptr;
}

bool sdocMeshLod(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel)
{
	return true;
}
#endif

bool sdocQueryOccludees(void * pSDOC, const float *bbox, unsigned int nMesh, bool *results)
{

	SOCPrivate *instance = (SOCPrivate *)pSDOC;
	if (instance == nullptr)
	{
		return false;
	}

	///////batchQuery would do input check
	return instance->batchQuery(bbox, nMesh, results);
}
static void CheckRecording(SOCPrivate *instance, int occluderNum) 
{
	if (instance->m_frameInfo->mIsRecording)
	{
		int occNum = occluderNum;
		if (occNum == -1) {
			occNum = instance->m_rapidRasterizer->mOccluderCenter->mLastFinalRendered;
		}
		for (int idx = 0; idx < occNum; idx++)
		{
			util::OccluderInput* occ = instance->m_rapidRasterizer->mOccluderCenter->mPool[idx];
			if (occ->IsRawMesh && occ->IsValidRawMesh == false) 
			{
				continue;
			}

			instance->m_frameInfo->RecordOccluder(occ->inVtx, occ->inIdx, occ->nVert, occ->nIdx,
				occ->modelWorld,  occ->backfaceCull);
		}

	}
}
static void UsePreviousOccluder(SOCPrivate *instance, int sameOccluderNum) {
	instance->m_rapidRasterizer->UsePrevFrameOccluders(sameOccluderNum);
}

bool sdocSet(void*pSDOC, unsigned int ID, unsigned int configValue)
{

	SOCPrivate *instance = (SOCPrivate *)pSDOC;

	switch (ID)
	{

	case SDOC_BeforeQueryTreatTrueAsCulled:
		instance->m_rapidRasterizer->BeforeQueryTreatTrueAsCulled();
		return true;

	case SDOC_Set_UsePrevDepthBuffer:
		CheckRecording(instance, -1);
		instance->m_rapidRasterizer->UsePrevDepthData();
		
		return true;
	case SDOC_Set_UsePrevFrameOccluders:
		CheckRecording(instance, configValue);
		UsePreviousOccluder(instance, configValue);
		return true;

	case SDOC_DestroySDOC: 		instance->socDestroy();						return true;
	default:		return instance->setConfig(ID, configValue);
	}

}



#if defined(SDOC_NATIVE_DEBUG) && defined(SDOC_NATIVE)
static int replayConfig = 0;
static int replayFrameNum = 0;
static uint64_t replaySetting = false;
static float * replayResult;
bool socReplay(const char *file_path)
{
    if (file_path == nullptr)
    {
        return false;
    }

    SOCPrivate *instance = (SOCPrivate *)sdocInit(384, 96, 1.0f);
    if (instance == nullptr)
    {
        return false;
    }

    bool replied = instance->replay(file_path, replayConfig, replayFrameNum, replaySetting, replayResult);
	delete instance;
	return replied;
}
#endif


static const char * gImageSavePath = nullptr;
bool sdocSync(void * pSDOC, unsigned int id, void *param)
{
#if defined(SDOC_NATIVE_DEBUG) && defined(SDOC_NATIVE)
	if (id == SDOC_Test_Result) {
		replayResult = (float*)param;
		return true;
	}
	if (id == SDOC_Test_Config) 
	{
		replayConfig = *((int*)param);
		return true;
	}
	else if (id == SDOC_Test_FrameNum) {
		replayFrameNum = *((int*)param);
		return true;
	}
	else if (id == SDOC_Test_SettingConfig)
	{
		replaySetting = *((uint64_t*)param);
		return true;
	}
	else if (id == SDOC_Test_ReplayAndDestroy) 
	{
		socReplay((const char *)param);
		SOCPrivate *instance = (SOCPrivate*)pSDOC;
		instance->socDestroy();
		
		util::OccluderQuad::OfflineTestClearBakeBuffer();
		
		return true;
	}
#endif

	if (id == SDOC_GetOccluderPotentialVisibleSet) {
		SOCPrivate *instance = (SOCPrivate*)pSDOC;
		if (instance == nullptr)
		{
			return false;
		}

		bool* value = reinterpret_cast<bool*>(param);
		instance->m_rapidRasterizer->SyncOccluderPVS(value);
		return true;
	}

	if (id == SDOC_SetQueryTreeData) {
		SOCPrivate *instance = (SOCPrivate*)pSDOC;
		if (instance == nullptr)
		{
			return false;
		}

		uint16_t* value = reinterpret_cast<uint16_t*>(param);
		instance->m_rapidRasterizer->ConfigQueryChildData(value);
		return true;
	}
	

	if (id == SDOC_Get_IsSameCamera) 
	{
		SOCPrivate *instance = (SOCPrivate*)pSDOC;
		if (instance == nullptr)
		{
			return false;
		}

		bool* value = reinterpret_cast<bool*>(param);
		*value = instance->m_frameInfo->IsSameCameraWithPrev;
		return true;
	}
	if (id == SDOC_Get_Log)
	{
		char* msg = reinterpret_cast<char*>(param);
		if (msg == nullptr)
		{
			return false;
		}
		return SOCLogger::Singleton.GetLog(msg);
	}
	else if (id == SDOC_SetPrintLogInGame)
	{
		int* value = reinterpret_cast<int*>(param);
		if (value != nullptr)
		{
			SOCLogger::Singleton.SetStoreMsgConfig((*value) == 1);
			return true;
		}
		return false;
	}
	else if (id == SDOC_Set_FrameCaptureOutputPath) {
		char* value = reinterpret_cast<char*>(param);
		if (value != nullptr)
		{
			std::string output = std::string(value);
			SOCLogger::Singleton.SetCaptureOutputPath(output);
#if defined(SDOC_NATIVE)
			util::OccluderQuad::SetOutputPath(output);
#endif
			return true;
		}
		return false;
	}
	
#if defined(SDOC_NATIVE)
	else if (id == 888) {
		char* value = reinterpret_cast<char*>(param);
		if (value != nullptr)
		{
			std::string output = std::string(value);
			util::OccluderQuad::SetOutputPath(output);
			util::OccluderQuad::SetSaveModel(1);
			return true;
		}
		return false;
	}
	else if (id == SDOC_BakeMeshSimplifyConfig) {
		int* value = reinterpret_cast<int*>(param);
		if (value != nullptr)
		{
			util::OccluderQuad::AllowPlanarQuadMerge(value[0] != 0);
			if (value[1] >= 0 && value[1] <= 2) {
				util::OccluderQuad::SetTerrainGridOptimization(value[1]);
			}		
			if (value[2] >= 80 && value[2] <= 89) {
				util::OccluderQuad::SetTerrainRectangleAngle(value[2]);
			
			}if (value[3] >= 1 && value[3] <= 10) {
				util::OccluderQuad::SetTerrainRectangleMergeAngle(value[3]);
			}
			return true;
		}
		return false;
	}
#endif
	else if (id == SDOC_Get_Version)
	{
		int* value = reinterpret_cast<int*>(param);
		if (value != nullptr) 
		{
			union w {
				int a;
				char b;
			}c;
			c.a = 1;
			if (c.b == 1) {
				*value = common::VERSION_MAJOR * 10 + common::VERSION_SUB;
			}
			else {
				*value = 0; //special version to indicate it is a big endian system.
				return false;
			}

		}
		return true;
	}



	SOCPrivate *instance = (SOCPrivate*)pSDOC;
	if (instance == nullptr)
	{
		return false;
	}

	if (param == nullptr)
	{
		return false;
	}
	if (id == SDOC_Get_DepthBufferWidthHeight)
	{
		if (param == nullptr)
		{
			return false;
		}
		int* input = (int*)param;
		input[0] = instance->m_frameInfo->Width;
		input[1] = instance->m_frameInfo->Height;
		return true;
	}
	else if (id == SDOC_Get_DepthMap)
	{
		unsigned char* data = (unsigned char*)param;
		return instance->doDumpDepthMap(data, common::DumpImageMode::DumpFull);
	}
	else if (id == SDOC_Set_CoherentModeSmallRotateDotAngleThreshold)
	{
		instance->SmallRotateDotAngleThreshold = *(float*)param;
	}
	else if (id == SDOC_Set_CoherentModeLargeRotateDotAngleThreshold)
	{
		instance->LargeRotateDotAngleThreshold = *(float*)param;
	}
	else if (id == SDOC_Set_CoherentModeCameraDistanceNearThreshold)
	{
		instance->CameraNearDistanceThreshold = *(float*)param;
	}
	else if (id == SDOC_Reset_DepthMapWidthAndHeight)
	{
		unsigned int* widthHeight = reinterpret_cast<unsigned int*>(param);
		if (widthHeight == nullptr) {
			return false;
		}
		return instance->resize(widthHeight[0], widthHeight[1]);
	}
	else if (id == SDOC_Get_MemoryUsed)
	{
		int* output = (int*)param;
		*output = (int)(instance->getMemoryByteUsage() >> 10);

		return true;
	}
	else if (id == SDOC_Save_DepthMap)
	{
		unsigned char * fullImgData = reinterpret_cast<unsigned char *>(param);
		if (fullImgData != nullptr)
		{
			bool r = instance->saveColorImage(fullImgData, gImageSavePath);
			gImageSavePath = nullptr;
			return r;
		}
		return false;
	}
	else if (id == SDOC_Save_DepthMapPath)
	{
		gImageSavePath = reinterpret_cast<const char *>(param);
		return false;
	}

	
	else
	{
		return false;
	}
	return true;
}