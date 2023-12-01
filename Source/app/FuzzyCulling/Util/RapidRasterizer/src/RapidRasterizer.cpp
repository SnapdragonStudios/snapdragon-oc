//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
#include "Util/RapidRasterizer/RapidRasterizer.h"

#include "Rasterizer.h"

#include "MathUtil.h"
#if defined(SDOC_NATIVE)
#include <string>
#endif
#include <fstream>

namespace util
{

	static constexpr int Config_AABBCacheSize = 4; //must be power of 2
	
	static const int LargeTerrainOccluderPriority = 2;
RapidRasterizer::RapidRasterizer()
{
	AABBNextStoreIdx = 0;
	AABBCache = new OccluderAABB[Config_AABBCacheSize];
	m_instance = new Rasterizer();

	mOccluderCenter = new OccluderManager();

	//this->mBackFaceCullOffFirst = false;
	//this->mDelayLargeOccluderThreshold = 9999;
}


#if defined(SDOC_WIN)
#pragma warning(disable : 4996)
#endif



RapidRasterizer::~RapidRasterizer()
{
	if (AABBCache != nullptr) delete[] AABBCache;
	AABBCache = nullptr;

	delete m_instance;
	m_instance = nullptr;

	delete mOccluderCenter;
	mOccluderCenter = nullptr;
#ifdef SDOC_DEBUG
	LOGI( "release RapidRasterizer" );
#endif
}

void RapidRasterizer::setResolution(unsigned int width, unsigned int height)
{
    m_instance->setResolution(width, height);
}



bool RapidRasterizer::dumpDepthMap(unsigned char *depthMap, common::DumpImageMode mode) 
{
	OnRenderFinish();
    if (!depthMap)
    {
        return false;
    }
	return m_instance->readBackDepth(depthMap, mode);
}
bool RapidRasterizer::batchQuery(const float * bbox, unsigned int nMesh, bool * results) 
{
	if (results == nullptr)
	{ 
		//this is used for multi-thread query
		if (mInRenderingState == false)
		{
			//Assume mWidthIn1024 is always true
			return  m_instance->queryVisibility<false, true, true>(bbox);
		}
		else 
		{
			return true; //WRONG state would be auto treated as visible
		}
	}

	OnRenderFinish();

	if (m_instance->mWidthIn1024) {
		m_instance->batchQuery<true>(bbox, nMesh, results);
	}
	else {
		m_instance->batchQuery<false>(bbox, nMesh, results);
	}


	if (mShowCulled)
	{
		for (unsigned int idx = 0; idx < nMesh; idx++)
		{
			results[idx] = !results[idx];
		}
	}
#if 0
	int visible = 0;
#if defined(SDOC_WIN)
	for (unsigned int idx = 0; idx < nMesh; idx++)
	{
		visible += results[idx];
		LOGI("Idx %d Visible %d Current %d", idx, visible, (int)results[idx]);
	}
#else
	std::string path = "/sdcard/Android/data/com.qualcomm.sdocdemo/files/";
	std::string queryOutput = path + "result.txt";
	std::ofstream stream(queryOutput.c_str(), std::ofstream::out);
	for (unsigned int idx = 0; idx < nMesh; idx++)
	{
		visible += results[idx];
		stream << "Idx " << idx << "Visible " << visible << "Current " << (int)results[idx] << "\n";
	}
	stream.close();
#endif
#endif

	return true;
}




void RapidRasterizer::OnRenderFinish()
{
	if (mInRenderingState) 
	{
		FlushCachedOccluder(mOccluderCenter->mCurrentOccNum);
		mOccluderCenter->OnRenderEnd();
		m_instance->onOccluderRenderFinish();
		mInRenderingState = false;



		m_instance->setModelViewProjectionT(mViewProjT);
		m_instance->UpdateFrustumCullPlane();
	}
}


void RapidRasterizer::onNewFrame(uint64_t frame, bool criticalFrame, bool isRotating)
{
	
	OnRenderFinish(); //force render finish

	if (PrintNumberOfOccluderOnce) {
		int occNum = this->mOccluderCenter->mLastFinalRendered;
		int bakedOcc = 0;
		int fullOcc = 0;
		for(int idx = 0; idx<occNum; idx++){

			OccluderInput* occ = mOccluderCenter->mPool[idx];
			if (occ->inIdx == nullptr) {
				bakedOcc++;
			}
			else {
				fullOcc++;
			}
		}

		if (mLastBakedOccluderNum != bakedOcc || mLastFullOccluderNum != fullOcc) 
		{
			LOGI("SDOC OcculderNum Baked from %d to %d, Full From %d to %d", mLastBakedOccluderNum, bakedOcc, mLastFullOccluderNum, fullOcc);
			mLastBakedOccluderNum = bakedOcc;
			mLastFullOccluderNum = fullOcc;
		}
	}
	if (mInvalidRawMeshNum > 0) 
	{
		this->mInvalidRawMeshGroup += mInvalidRawMeshNum;
		if ((this->mFrameNum & 127) == 0) 
		{
			LOGI("Error: SDOC Received invalid empty occluder mesh Num : %d among %d Last 128 frame total %d", 
				this->mInvalidRawMeshNum, 
				this->mOccluderCenter->mLastFinalRendered, 
				this->mInvalidRawMeshGroup);
			this->mInvalidRawMeshGroup = 0;
		}

		mInvalidRawMeshNum = 0;
	}

	this->mFrameNum = frame;

	m_instance->mOccludeeTrueAsCulled = false;


	m_instance->configGlobalFrameNum(this->mFrameNum);


	if (criticalFrame || m_instance->mDebugRenderType != 0)
	{
	}
	else 
	{
		m_instance->mInterleave.configRotating(isRotating);
	}

	m_instance->mInterleave.CurrentFrameInterleaveDrawing =
		!criticalFrame
		&& this->m_instance->InterleaveRendering
		&& (this->mFrameNum > (common::START_FRAME_COUNT + 1))
		&& this->m_instance->mDebugRenderType == 0;

	this->mInRenderingState = true;

	if (mValidAABB > 0)
	{
		for (int i = 0; i < mValidAABB; i++)
		{
			AABBCache[i].LastOccluderVertices = nullptr;
		}
		mValidAABB = 0;
	}

}

void RapidRasterizer::setNearPlane(float nearPlane)
{
	if (nearPlane < 1.0f) nearPlane = 1.0f;
	m_instance->mNearPlane = nearPlane;
}

size_t RapidRasterizer::getMemoryByteUsed()
{
	size_t memory =  0;

	memory += m_instance->getMemoryUsage();

	memory += sizeof(RapidRasterizer);
	if(this->mOccluderCenter != nullptr)
		memory +=  this->mOccluderCenter->GetMemory() ;
	return memory;
}



void RapidRasterizer::configCoherentMode(int octuple)
{
	m_instance->mInterleave.mOctuple = octuple;
	m_instance->InterleaveRendering = octuple > 0;

	
	m_instance->configCoherent();
}


void RapidRasterizer::SetCCW(bool IsModelCCW)
{
	//LOGI("SDOC MODEL CCW = %d", (int)IsModelCCW);
	m_instance->mClockWise = IsModelCCW == false;
}
int RapidRasterizer::GetCW()
{
	return m_instance->mClockWise;
}


void RapidRasterizer::ShowOccludeeInDepthmap(int value)
{
	m_instance->ShowOccludeeInDepthMapNext = value == 1;
}

static bool NeedFlipFace(const float * modelWorld) 
{
	//return true;  //direct return true if no negative scale models
	float determinant = modelWorld[0] * (modelWorld[5] * modelWorld[10] - modelWorld[6] * modelWorld[9])
		- modelWorld[1] * (modelWorld[4] * modelWorld[10] - modelWorld[6] * modelWorld[8])
		+ modelWorld[2] * (modelWorld[4] * modelWorld[9] - modelWorld[5] * modelWorld[8]);
	return determinant < 0;
}

bool RapidRasterizer::RasterizeOccluder(OccluderInput* occ) 
{
	m_instance->mUpdateAnyBlock = false;

	common::Matrix4x4 LocalToWorldT;
	LocalToWorldT.updateTranspose(occ->modelWorld);

	common::Matrix4x4 LocalToClipT;
	common::Matrix4x4::Multiply(mViewProjT, LocalToWorldT, LocalToClipT);
	m_instance->setModelViewProjectionT(LocalToClipT);


	if (occ->IsRawMesh == false)
	{
		const float *minExtents = occ->inVtx + 2; //minExtents

		bool visible = false;
		visible = m_instance->queryVisibility<true, false, false>(minExtents);

		if (visible)
		{
			m_instance->mOccluderCache.FlipOccluderFace = NeedFlipFace(occ->modelWorld);

			uint16_t *meta = (uint16_t *)occ->inVtx;

			common::OccluderMesh raw;
			raw.EnableBackface = meta[0] & 1;
			raw.SuperCompress = meta[1] <= common::SuperCompressVertNum;
			raw.QuadSafeBatchNum = meta[2];
			raw.TriangleBatchIdxNum = meta[3];

			//ignore first 8 float as they are stored 64bit meta data and 6float for minExtent
			raw.Vertices = (float*)(occ->inVtx + 8);




			m_instance->doRasterize(raw);

		}
	}
	else
	{
		if (occ->IsValidRawMesh == false) 
		{
			this->mInvalidRawMeshNum++;
			return true;//for invalid occluder, potential visible set as true
		}
		bool visible = false;

		__m128* OccluderMinExtent = CalculateAABB(occ->nVert, occ->inVtx);

		float minExtent[6];
		float* inputME= (float*)OccluderMinExtent;
		memcpy(minExtent, inputME, 3 * sizeof(float));
		memcpy(minExtent+3, inputME+4, 3 * sizeof(float));

		visible = m_instance->queryVisibility<true, false, false>(minExtent);


		if (visible)
		{
			m_instance->mOccluderCache.FlipOccluderFace = NeedFlipFace(occ->modelWorld);

			__m128 scalingXYZW = _mm_setr_ps(1.0f, 1.0f, 1.0f, 0);
			__m128 InvExtents = _mm_div_ps(scalingXYZW, OccluderMinExtent[1]);
			//check whether any of BoundsRefinedExtents is zero
			__m128 positive = _mm_cmpgt_ps(OccluderMinExtent[1], _mm_setzero_ps());
			__m128 invExtents = _mm_and_ps(InvExtents, positive);


			__m128 minusRefMinInvExtents = _mm_mul_ps(_mm_negate_ps_soc(invExtents), OccluderMinExtent[0]);
			minusRefMinInvExtents = _mm_add_ps(minusRefMinInvExtents, _mm_setr_ps(0, 0, 0, 1));


			//temp set of rasterize required input
			m_instance->mOccluderCache.FullMeshInvExtents = invExtents;
			m_instance->mOccluderCache.FullMeshMinusRefMinInvExtents = minusRefMinInvExtents;

			common::OccluderMesh raw;
			raw.Indices = occ->inIdx;
			raw.Vertices = occ->inVtx;
			raw.TriangleBatchIdxNum = occ->nIdx;
			raw.VerticesNum = occ->nVert;
			raw.EnableBackface = occ->backfaceCull;

			m_instance->doRasterize(raw);

		}

	}
	return m_instance->mUpdateAnyBlock;
}
void RapidRasterizer::SubmitBakedOccluder(unsigned short * inVtx, const float * modelWorld)
{

	////uint16_t* pVint = (uint16_t*)compressData;
	//////pVint += 3;
	////pVint[0] = (int)enableBackfaceCull + ((int)quadData.IsTerrain << 1);
	////pVint[1] = quadData.IsPlanar;
	////pVint[2] = quadBatchNum;
	////pVint[3] = triangleBatchNum;


	uint16_t *meta = inVtx;

	

	OccluderInput* occ = mOccluderCenter->RequestOccluder();
	occ->inVtx = (float*)inVtx;
	occ->inIdx = nullptr;
	occ->nVert = 0;
	occ->nIdx = 0;
	occ->modelWorld = modelWorld;
	occ->backfaceCull = meta[0] & 1;

	occ->priority = occ->backfaceCull;
	occ->IsRawMesh = false;
	if (meta[0] & 32 ) //terrain at bit 5
	{
		occ->priority |= LargeTerrainOccluderPriority;
	}
}


bool RapidRasterizer::SubmitRawOccluder(const float * inVtx, const unsigned short * inIdx, unsigned int nVert, unsigned int nIdx, const float * modelWorld, bool backfaceCull)
{
	OccluderInput * occ = mOccluderCenter->RequestOccluder();
	occ->inVtx = inVtx;
	occ->inIdx = inIdx;
	occ->nVert = nVert;
	occ->nIdx = nIdx;
	occ->modelWorld = modelWorld;
	occ->backfaceCull = backfaceCull;
	occ->priority = backfaceCull;
	occ->IsRawMesh = true;
	occ->IsValidRawMesh = nVert > 0 && nIdx > 0 && (nIdx % 3 == 0);
	return occ->IsValidRawMesh;
}


void RapidRasterizer::UsePrevDepthData()
{
	//use same frame as previous
	m_instance->mCurrValidOccluderNum = m_instance->mLastOccluderNum;
	mInRenderingState = false;
}

void RapidRasterizer::setRenderType(int renderType)
{
	if (renderType == -1) 
	{
		//enum RenderType {
		//	Mesh = 0,
		//	MeshLine = 1,
		//	MeshPoint = 2,
		//	Line = 3,
		//	Point = 4,
		//};
		this->m_instance->mDebugRenderType = (this->m_instance->mDebugRenderType + 1) % 5;
	}
	else
	{
		this->m_instance->mDebugRenderType = renderType;
	}

	if (m_instance->m_totalPixels != 0 && this->m_instance->mDebugRenderType != 0)
	{
		m_instance->m_depthBufferPointLines.resize(m_instance->m_totalPixels);
	}

	this->m_instance->mDebugRenderMode = (this->m_instance->mDebugRenderType > 0) ;
	LOGI("Set Render Mode to %d", this->m_instance->mDebugRenderType);
}

void RapidRasterizer::UsePrevFrameOccluders(int sameOccluderNum)
{
	mOccluderCenter->mCurrentOccNum = sameOccluderNum;
	mOccluderCenter->mCurrentRasterizedNum = 0;
	mOccluderCenter->mLastFinalRendered = 0;
}
void RapidRasterizer::FlushCachedOccluder(int endOccluderNum)
{
	//Record the state of Latest frame interleave state for occluder PVS usage
	mOccluderCenter->LatestFrameInterleaveDraw = m_instance->mInterleave.CurrentFrameInterleaveDrawing;

	//already finish the rendering
	if (mOccluderCenter->mCurrentRasterizedNum >= mOccluderCenter->mCurrentOccNum || mInRenderingState == false)
	{
		return;
	}

	//delay configBeforeRasterization to support same frame skip feature
	if (this->m_instance->mCurrValidOccluderNum == 0)
	{
		this->m_instance->configBeforeRasterization();
		this->m_instance->mCurrValidOccluderNum++;
	}


	if (mSortByPriorityQueue == false) {
		for (int idx = mOccluderCenter->mCurrentRasterizedNum; idx < endOccluderNum; idx++)
		{
			OccluderInput* occ = mOccluderCenter->mPool[idx];
			occ->potentialVisible = RasterizeOccluder(occ);
		}
	}
	else {
		if (this->mBackFaceCullOffFirst)
		{
			for (int idx = mOccluderCenter->mCurrentRasterizedNum; idx < endOccluderNum; idx++)
			{
				OccluderInput* occ = mOccluderCenter->mPool[idx];
				if (occ->priority == 0)
				{
					occ->potentialVisible = RasterizeOccluder(occ);
				}
			}
			for (int idx = mOccluderCenter->mCurrentRasterizedNum; idx < endOccluderNum; idx++)
			{
				OccluderInput* occ = mOccluderCenter->mPool[idx];
				if (occ->priority == 1)
				{
					occ->potentialVisible = RasterizeOccluder(occ);
				}
			}
		}
		else
		{
			for (int idx = mOccluderCenter->mCurrentRasterizedNum; idx < endOccluderNum; idx++)
			{
				OccluderInput* occ = mOccluderCenter->mPool[idx];
				if (occ->priority < 2)
				{
					OccluderInput* occ = mOccluderCenter->mPool[idx];
					occ->potentialVisible = RasterizeOccluder(occ);
				}
			}
		}

		for (int idx = mOccluderCenter->mCurrentRasterizedNum; idx < endOccluderNum; idx++)
		{
			OccluderInput* occ = mOccluderCenter->mPool[idx];
			if (occ->priority >= LargeTerrainOccluderPriority)
			{
				OccluderInput* occ = mOccluderCenter->mPool[idx];
				occ->potentialVisible = RasterizeOccluder(occ);
			}
		}
	}

	mOccluderCenter->mCurrentRasterizedNum = mOccluderCenter->mCurrentOccNum;

}

__m128* RapidRasterizer::CalculateAABB(unsigned int nVert, const float * vertices)
{
	for(int idx = 0; idx< mValidAABB; idx++)
	{
		if (AABBCache[idx].LastOccluderVertices == vertices)
		{
			return AABBCache[idx].OccluderMinExtent;
		}

		//used to verify the effectiveness when replaying...
		if (false) 
		{
			if (AABBCache[idx].LastOccluderVertices != nullptr) {
				if (AABBCache[idx].LastOccluderVertices[0] == vertices[0] &&
					AABBCache[idx].LastOccluderVertices[1] == vertices[1] &&
					AABBCache[idx].LastOccluderVertices[2] == vertices[2] &&
					AABBCache[idx].LastOccluderVertices[3] == vertices[3] &&
					AABBCache[idx].LastOccluderVertices[4] == vertices[4] &&
					AABBCache[idx].LastOccluderVertices[5] == vertices[5] &&
					AABBCache[idx].LastOccluderVertices[6] == vertices[6] &&
					AABBCache[idx].LastOccluderVertices[7] == vertices[7] &&
					AABBCache[idx].LastOccluderVertices[8] == vertices[8])
				{
					return AABBCache[idx].OccluderMinExtent;
				}
			}
		}
	}

	__m128 refMin = _mm_set1_ps(std::numeric_limits<float>::infinity());
	__m128 refMax = _mm_set1_ps(-std::numeric_limits<float>::infinity());

	const float * pVertices = vertices;
	unsigned int nVert3 = nVert * 3;
	for (unsigned int idx = 0; idx < nVert3; idx += 3)
	{
		__m128 p = _mm_setr_ps(pVertices[0], pVertices[1], pVertices[2], 1.0f);
		refMin = _mm_min_ps(p, refMin);
		refMax = _mm_max_ps(p, refMax);

		pVertices += 3;
	}

	OccluderAABB & cache = AABBCache[AABBNextStoreIdx];
	cache.OccluderMinExtent[0] = refMin;
	cache.OccluderMinExtent[1] = _mm_sub_ps(refMax, refMin);
	cache.LastOccluderVertices = vertices;
	AABBNextStoreIdx++;
	AABBNextStoreIdx &= (Config_AABBCacheSize - 1); //store 4 cache only
	mValidAABB++;
	mValidAABB = std::min<int>(mValidAABB, Config_AABBCacheSize);

	return cache.OccluderMinExtent;
}


void RapidRasterizer::EnablePriorityQueue(bool value)
{
	this->mSortByPriorityQueue = value;
}

void RapidRasterizer::SyncOccluderPVS(bool* value)
{
	int frameOccluderNum = mOccluderCenter->mCurrentOccNum;
	if(mInRenderingState == false) frameOccluderNum = mOccluderCenter->mLastFinalRendered;

	for (int idx = 0; idx < frameOccluderNum; idx++) 
	{
		value[idx] = mOccluderCenter->mPool[idx]->potentialVisible;
	}
	value[frameOccluderNum] = mOccluderCenter->LatestFrameInterleaveDraw;// m_instance->mInterleave.CurrentFrameInterleaveDrawing;

	
}

void RapidRasterizer::BeforeQueryTreatTrueAsCulled()
{
	m_instance->mOccludeeTrueAsCulled = true;
}

void RapidRasterizer::ConfigQueryChildData(uint16_t* value)
{
	this->m_instance->mOccludeeTreeData = value;
}

void OccluderManager::OnRenderEnd()
{
	mLastFinalRendered = mCurrentOccNum;

	mCurrentOccNum = 0;
	mCurrentRasterizedNum = 0;
}

} // namespace uti
