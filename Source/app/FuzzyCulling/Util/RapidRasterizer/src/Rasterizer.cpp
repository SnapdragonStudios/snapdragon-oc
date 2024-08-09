//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
/*
**
** Code from https://github.com/rawrunprotected/rasterizer/blob/master/SoftwareRasterizer/Rasterizer.cpp under CC0 1.0 Universal (CC0 1.0) Public Domain Dedication.
*/
#if defined(SDOC_NATIVE)
#define PLATFORM_WINDOWS 1
#endif
#if defined(SDOC_IOS)
#define PLATFORM_IOS 1
#endif
#if defined(SDOC_ANDROID)
#define PLATFORM_ANDROID_ARM64 1
#endif


#if PLATFORM_WINDOWS || PLATFORM_ANDROID_ARM64 || PLATFORM_IOS
#include "Rasterizer.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "../../../Common/MemUtil.h"
#include "../../../Common/SOCUtil.h"


#include <chrono>
#include <iostream>

#if defined(SDOC_NATIVE)
#pragma warning( disable : 4996  )
#endif

#if defined(SDOC_ANDROID)
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <unistd.h>
#include <fstream>
#endif
using namespace common;

#if defined(SDOC_ANDROID) || defined(__aarch64__)
#define SDOC_ARM
#endif

//****************************************************************************************************************************************
//Debug tips
//1. set SaveSimplifyModel to check the input model
//2. set bForceMeshLineDebug to true to check the meshline, set MeshLineScale to a smaller value if wish to check meshline outside viewport
//3. off backface culling: set bDebugForceBackfaceCullingOFF to true
//4. enable bDebugVerySmallOccluders to relax mask constraints for small occluders
//****************************************************************************************************************************************
namespace util
{
	static constexpr bool bEnableOccludeeOBBQuery = false; //if wish to use OBB query, set this to true

	static constexpr bool bDebugForceBackfaceCullingOFF = false; //SET true to disable BFC to debug!!


	enum RenderType {
		Mesh = 0,
		MeshLine = 1,
		MeshPoint = 2,
		Line = 3,
		Point = 4,
	};



	static const     float maxInvW = std::sqrt(std::numeric_limits<float>::max());
#if defined(SDOC_ANDROID) && !defined(__aarch64__)
	static constexpr bool ARMV7 = true;
#else
	static constexpr bool ARMV7 = false;
#endif
	
	static constexpr bool bDebug_ValidateMinMaxBlockIdx = false;

	////if occludee clipped with near plane, do the frustum culling check
	////otherwise, latest using quad could achieve frustum culling
	static constexpr bool bFrustumCullIfClip = true; 


	static constexpr bool bSplitQuadToTriangleForOccludee = true;
	static constexpr float ExtremeSmallPrimitiveAreaTh = 2.0/64.0f; //Primitive that cover less than 2 pixel area
	static constexpr bool bDebugVerySmallOccluders = false;

	

	//Use branch to fast check whether occluder local to world transform having scales..
	static constexpr bool bFastSetUpMatOp = true;

	//10% perf improvement for precomputeRasterizationTable
	static constexpr bool bMaskTableOffsetOptimization = true;

	static constexpr bool bQuadToTriangleMergeOp = true; // in case only two quad active, could pack into one triangle patch

	static constexpr bool bConvexOptimization = true;


	static constexpr bool bDrawOccludeeToDepthMap = false; //Debug function, draw all primitives in occludee to depth map. Early exit would be disabled!!!
	static constexpr bool bForceMeshLineDebug = false;
	static constexpr float MeshLineScale = 1.0f;
	

	static constexpr int nOcludeeOBBSize = 18;   //Debug: change to 12 if input only 4 points, otherwise 18

	static constexpr bool bOccludeeBitScanOp = 1;
	static constexpr uint16_t bOccludeeMinDepthThreshold = 150 << 8; //depth 9360


#if defined( SUPPORT_ALL_FEATURE)
	static constexpr bool bDumpTriangle = false;
#endif 

	static constexpr bool bPixelAABBClipping = true;  //developer todo: set this to false to improve performance. it is used to handle degenerated hair which should not affect correctness
	

	static constexpr bool bTestObbByAABB = false; //enable this to print
	static constexpr bool bTestQUADByAABB = false; //enable this to print

	static constexpr int DEBUG_DATA_SIZE = 256; //used to store all the counters
	static constexpr bool PrintOccludeeState = false;

	static constexpr bool DebugOccluderOccludee = 0;
	static constexpr bool bDebugOccluderOnly = 0;
	static constexpr int bDebugOccluderPixelX = -1;
	static constexpr int bDebugOccluderPixelY = -1;

	static constexpr bool bDumpBlockColumnImage = false;
	static constexpr int DebugDumpBlockX = -1 / 8;
	static constexpr int DebugDumpBlockY = -1 / 8;

	static constexpr uint16_t MIN_UPDATED_BLOCK_DEPTH = 1;
	static constexpr uint16_t MIN_UPDATED_BLOCK_DEPTH2 = 2;
#if defined(SDOC_ANDROID) && !defined(__aarch64__)
	static constexpr float MIN_PIXEL_DEPTH_FLOAT = 1.17549435082e-38; // ARMV7 minimal positive non denormalized number
#else 
	static constexpr float MIN_PIXEL_DEPTH_FLOAT = 3.44383110592e-41; // the float value of  0x00006000;
#endif


	static constexpr int PureCheckerBoardApproach = 8;
	static constexpr int CheckerBoardVizMaskApproach = 9; //2 x 4.5 _m128i
	static constexpr int FullBlockApproach = 16; //2 x 8 rows
	static constexpr int PairBlockNum = 9; //change to 16 for full block approach
	static constexpr bool VRS_X4Y4_Optimzation = true;
 
	static constexpr bool OCCLUDEE_NEARCLIP_BBOX_CHECK_IGNORE_OPTIMIZATION = false;
	
	

	static constexpr bool Rasterize_ClippedOccludee_AS_OCCLUDER_AABB = false;
	static constexpr bool Rasterize_ClippedOccludee_AS_OCCLUDER_OBB = true;
	static constexpr bool Rasterize_ClippedOccludee_AS_OCCLUDER = Rasterize_ClippedOccludee_AS_OCCLUDER_AABB || Rasterize_ClippedOccludee_AS_OCCLUDER_OBB;

	static constexpr bool NEAR_CLIP_SURE_VISIBLE_OPTIMIZATION = true;
	static constexpr int MAX_DEPTH = 0xFFFF;
	
	//Effective Optimization configs
	static constexpr bool CULL_FEATURE_HizPrimitiveCull = true;


	


	

	static constexpr bool ReflectBoostBlockMask_Optimization = true;  //reduce the initial mask calculation time by HALF


	
	static constexpr int BBOX_STRIDE = 6;

static constexpr float floatCompressionBias = 2.5237386e-29f; // 0xFFFF << 12 reinterpreted as float
static constexpr float minEdgeOffset = -0.45f;


static constexpr int OFFSET_QUANTIZATION_BITS = 6;
static constexpr int OFFSET_QUANTIZATION_FACTOR = 1 << OFFSET_QUANTIZATION_BITS;

static constexpr float maxOffset = -minEdgeOffset;
// Remap [minOffset, maxOffset] to [0, OFFSET_QUANTIZATION]


//static constexpr float OFFSET_mul = (OFFSET_QUANTIZATION_FACTOR - 1) / (maxOffset - minEdgeOffset); //default
//static constexpr float OFFSET_add = 0.5f - minEdgeOffset * OFFSET_mul;

// scale and left shift one block. still can guarantee first all one and last all zero
static constexpr float OFFSET_mul = (OFFSET_QUANTIZATION_FACTOR - 1) * 64.4f / 63.0f / (maxOffset - minEdgeOffset); 
static constexpr float OFFSET_add = 0.5f - minEdgeOffset * OFFSET_mul - 1.0f;

static constexpr int SLOPE_QUANTIZATION_BITS = 6;
static constexpr int SLOPE_QUANTIZATION_FACTOR = 1 << SLOPE_QUANTIZATION_BITS;






Rasterizer::Rasterizer()
{
	mCameraPos[3] = 1.0;
	mAABBUnitArray = new __m128[17];
	const float scale = 1.0f;
	mAABBUnitArray[0] = _mm_setr_ps(0, scale, scale, 0);
	mAABBUnitArray[1] = _mm_setr_ps(scale, scale, 0, 0);
	mAABBUnitArray[2] = _mm_setr_ps(0, scale, scale, 0);
	mAABBUnitArray[3] = _mm_setr_ps(0, 0, scale, scale);
	mAABBUnitArray[4] = _mm_setr_ps(0, scale, scale, 0);
	mAABBUnitArray[5] = _mm_setr_ps(0, scale, scale, 0);
	mAABBUnitArray[6] = _mm_setr_ps(0, 0, scale, scale);
	mAABBUnitArray[7] = _mm_setr_ps(scale, scale, scale, scale);
	mAABBUnitArray[8] = _mm_setr_ps(0, 0, 0, 0);
	

	mAABBUnitArray[9] = _mm_setr_ps(scale, scale, 0, 0);
	mAABBUnitArray[10] = _mm_setr_ps(0, 0, 0, 0);
	mAABBUnitArray[11] = _mm_setr_ps(scale, 0, 0, 0);
	mAABBUnitArray[12] = _mm_setr_ps(scale, 0, 0, 0);
	mAABBUnitArray[13] = _mm_setr_ps(0, scale, 0, 0);
	mAABBUnitArray[14] = _mm_setr_ps(0, scale, 0, 0);
	mAABBUnitArray[15] = _mm_setr_ps(scale, 0, scale, scale);
	mAABBUnitArray[16] = _mm_setr_ps(scale, 0, scale, scale);

	mAliveIdxMask[0] = 0;
	mAliveIdxMask[1] = 0; //0001                                   0
	mAliveIdxMask[2] = 1; //0010                                   1  
	mAliveIdxMask[3] = 1 << 2; //0011                              4            
	mAliveIdxMask[4] = 2; //0100                                   2      
	mAliveIdxMask[5] = 2 << 2; //0101                              8
	mAliveIdxMask[6] = (2 << 2) | 1; //0110                        9
	mAliveIdxMask[7] = (2 << 4) | (1<<2); //0111                  36

	mAliveIdxMask[8] = 3;//1000                                    3
	mAliveIdxMask[9] = 3<<2; //1001                               12
	mAliveIdxMask[10] = (3 << 2) | 1; //1010                      13
	mAliveIdxMask[11] = (3<<4) | (1 << 2); //1011                 52  
	mAliveIdxMask[12] = (3 << 2) | 2; //1100                      14
	mAliveIdxMask[13] = (3 << 4) | (2 << 2); //1101               56
	mAliveIdxMask[14] = (3 << 4) | (2 << 2) | 1; //1110           57
	mAliveIdxMask[15] = (3 << 6) | (2 << 4) | (1 << 2); //1111   228

	
	xFactors[0] = _mm_setr_ps(0.375f, 0.875f, 0.375f, 0.875f);
	xFactors[1] = _mm_setr_ps(0, .5f, 0, 0.5f);
	


	if (DebugOccluderOccludee)
	{
		if (DebugData == nullptr) {
			DebugData = new uint32_t[DEBUG_DATA_SIZE];
			memset(DebugData, 0, DEBUG_DATA_SIZE * sizeof(uint32_t));
		}
	}

	mCheckerBoardQueryOffset[0] = 0;
	mCheckerBoardQueryOffset[1] = mCheckerBoardQueryOffset[0] + 8; //8
	mCheckerBoardQueryOffset[2] = mCheckerBoardQueryOffset[1] + 7;//15
	mCheckerBoardQueryOffset[3] = mCheckerBoardQueryOffset[2] + 6; //21
	mCheckerBoardQueryOffset[4] = mCheckerBoardQueryOffset[3] + 5; //26
	mCheckerBoardQueryOffset[5] = mCheckerBoardQueryOffset[4] + 4; //30
	mCheckerBoardQueryOffset[6] = mCheckerBoardQueryOffset[5] + 3; //33 
	mCheckerBoardQueryOffset[7] = mCheckerBoardQueryOffset[6] + 2; //35

	uint64_t checkerPattern = 0xAA55AA55AA55AA55; //white pattern,   refer to vertical black pixel
	uint8_t* checkerMask = (uint8_t*)&checkerPattern;
	int maskIdx = 0;
	for (uint8_t y0 = 0; y0 <= 7; y0++)
	{
		for (uint8_t y1 = y0; y1 <= 7; y1++)
		{
			mCheckerBoardQueryMask[maskIdx] = 0;
			uint8_t* mask8 = (uint8_t*)(mCheckerBoardQueryMask + maskIdx);
			maskIdx++;
			for (uint16_t y = y0; y <= y1; ++y)
			{
				mask8[y >> 1] |= checkerMask[y];
			}
		}
	}

}

#if defined( SUPPORT_ALL_FEATURE)
// The most important function, _mm_packus_epi32 is extremely fast. But in SSE2 or Neon, there are no instructions can catch up with it.
static __m128i packDepthPremultiplied(__m128 depthAf, __m128 depthBf, __m128i maxDepth)
{
	__m128i depthA = _mm_castps_si128(depthAf);
	__m128i depthB = _mm_castps_si128(depthBf);
	depthA = _mm_min_epi32(depthA, maxDepth);
	depthB = _mm_min_epi32(depthB, maxDepth);
	__m128i minV = _mm_castps_si128(_mm_set1_ps(MIN_PIXEL_DEPTH_FLOAT));
	depthA = _mm_max_epi32(depthA, minV);
	depthB = _mm_max_epi32(depthB, minV);
	return _mm_packus_epi32(_mm_srai_epi32(depthA, 12), _mm_srai_epi32(depthB, 12));
}
#endif

inline static bool _mm_anybit_one_soc(__m128i depthA) {
	uint64_t* data = (uint64_t*)&depthA;
	return (data[0] | data[1]) != 0;
}
inline static __m128i packQueryDepth(__m128 depthA)
{
	__m128i depthAi = _mm_srli_epi32(_mm_castps_si128(depthA), 12);
	return _mm_min_epi32(depthAi, _mm_set1_epi32(0xFFFF));
}


inline static __m128i packDepthPremultipliedVRS12Fast(__m128i depthA)
{
	__m128i d16 = _mm_srli_epi32(depthA, 12);
	__m128i d162 = _mm_slli_epi32(d16, 16);
	return _mm_or_si128(d16, d162);
}


//inline static __m128i PackPositiveBatchZ(__m128 maxZ) 
//{
//	__m128i maxZi = _mm_srai_epi32(_mm_castps_si128(maxZ), 12);
//	return _mm_min_epu32(maxZi, _mm_set1_epi32(65535));
//}



Rasterizer::~Rasterizer()
{
	delete[]mAABBUnitArray;
	delete mOccluderCache;
	mOccluderCache = nullptr;
	delete mPixelBound;
	mPixelBound = nullptr;	

}
void Rasterizer::setResolution(unsigned int width, unsigned int height)
{
	m_width = width;




	m_height = height;
	this->m_totalPixels = m_width * m_height;
	m_MaxCoord_WHWH = _mm_setr_epi32(m_width - 1, m_height - 1, m_width - 1, m_height - 1);
	
	mWidthIn1024 = width <= 1024;
    m_blocksX = width >> 3;
	m_blocksY = height >> 3;
	m_blocksYMinusOne = m_blocksY - 1;
	m_blocksXMinusOne = m_blocksX - 1;

	if(bOccludeeBitScanOp)
		mAnyDataBlockMask.resize(m_blocksY);


	mBlockWidthMin = 0;
	mBlockWidthMax = m_blocksX - 1;

	
	m_blocksXFullDataRows = m_blocksX * PairBlockNum;



	m_depthBuffer.resize(m_blocksY * m_blocksXFullDataRows / 2);
	m_pDepthBuffer =(uint64_t*) &m_depthBuffer[0];

	this->m_blockSize = m_blocksX * m_blocksY;
	//309 - 311 -> Align data to make the size can be divided by 8
	//All test cases still pass after comment of 309 - 311 so far. But we leave it for safety.
	this->m_HizBufferSize = m_blockSize;

	int hizSize = m_HizBufferSize * 2; //append hizMin, hizMax
	int counter64 = hizSize / 4;
	int MaskTable = (OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR);
	int totalMaskTable = MaskTable + counter64;
	
	int hizStart = MaskTable;
	

	if (m_precomputedRasterTables.size() == 0)
	{
		m_precomputedRasterTables.resize(totalMaskTable, 0);
		precomputeRasterizationTable();
	}
	else 
	{
		if (m_precomputedRasterTables.size() < totalMaskTable) 
		{
			m_precomputedRasterTables.resize(totalMaskTable);
		}
	}


	m_pMaskTable = &m_precomputedRasterTables[0];
	m_pHiz = (uint16_t*) (m_pMaskTable + hizStart);
	m_pHizMax = m_pHiz + m_HizBufferSize;
	configCoherent();
	//
	if (mDebugRenderType != 0)
	{
		m_depthBufferPointLines.resize(this->m_totalPixels);
	}


	m_MaxCoordOccludee_WHWH = _mm_setr_epi32(m_width - 1, m_height - 1, m_width - 1, m_height - 1);
}

void Rasterizer::setModelViewProjectionT(const common::Matrix4x4 &input, bool simpleOccludee, OccluderRenderCache* occ)
{

    __m128 mat0 = input.Row[0];
    __m128 mat1 = input.Row[1];
    __m128 mat2 = input.Row[2];
    __m128 mat3 = input.Row[3];

	

	__m128 plane0 = _mm_add_ps(mat3, mat0);
	__m128 plane2 = _mm_add_ps(mat3, mat1);
	__m128 plane5 = _mm_sub_ps(mat3, mat2);


	__m128* localToClip ;
	if (simpleOccludee) {
		localToClip = m_OccludeelocalToClip;
	}
	else {
		localToClip = occ->m_localToClipData;
		occ->m_localToClipPointer = localToClip;
	}
	__m128* FrustumPlane = localToClip + 4;

	////////Refer to http://www.lighthouse3d.com/tutorials/view-frustum-culling/clip-space-approach-extracting-the-planes/
	////////Left Plane : x' = -1
	////////Right Plane : x' = 1
	////////Top Plane : y' = 1
	////////Bottom Plane : y' = -1
	////////Near Plane : z' = -1
	////////Far Plane : z' = 1
	////// Store rows
	FrustumPlane[0] = plane0;// _mm_add_ps(mat3, mat0);	//left
	FrustumPlane[1] = _mm_sub_ps(mat3, mat0);	//right
	FrustumPlane[2] = plane2;// _mm_add_ps(mat3, mat1);	//bottom
	FrustumPlane[3] = _mm_sub_ps(mat3, mat1);	//up
	FrustumPlane[4] = _mm_add_ps(mat3, mat2);	//near
	FrustumPlane[5] = plane5;// _mm_sub_ps(mat3, mat2);	//far

	// Bake viewport transform into matrix
	localToClip[0] = _mm_mul_ps_scalar_soc(plane0, static_cast<float>(m_width >> 1));  //m_width must be multiple of 16
	localToClip[1] = _mm_mul_ps_scalar_soc(plane2, static_cast<float>(m_height >> 1));  //m_height must be multiple of 8

	// Map depth from [-1, 1] to [bias, 0]
	localToClip[2] = _mm_mul_ps_scalar_soc(plane5, 0.5f * floatCompressionBias);

	localToClip[3] = mat3;

	
	

	_MM_TRANSPOSE4_PS(localToClip[0], localToClip[1],
		localToClip[2], localToClip[3]);
}


void Rasterizer::configCoherent()
{
	if(this->m_blocksX > 0 && this->m_blocksY > 0)
	{
		this->mInterleave.config(this->m_blocksX, this->m_blocksY);
	}
}

void  Rasterizer::configGlobalFrameNum(uint64_t frameNum)
{
	this->mGlobalFrameNum = frameNum;
	this->mCurrValidOccluderNum = 0;

	ShowOccludeeInDepthMap = ShowOccludeeInDepthMapNext;
	if (DebugOccluderOccludee) {
		ShowOccludeeInDepthMap = true;
	}

	if (ShowOccludeeInDepthMap) 
	{
		mOccludeeResults.clear();
		mCurrentOccludee = -1;
	}
}
static inline void ZeroHizUpdateBlocks(uint16_t * pHiz, uint32_t m_blocksX, uint32_t m_blocksY, uint32_t blockNum, uint32_t m_HizBufferSize)
{
	uint32_t size = blockNum * sizeof(uint16_t);

	uint32_t blockY = 0;
	while (blockY < m_blocksY)
	{
		memset(pHiz, 0, size); 
		memset(pHiz + m_HizBufferSize, 0, size);  
		pHiz += m_blocksX;
		++blockY;
	}
}

void Rasterizer::configBeforeRasterization()
{
	if(bDrawOccludeeToDepthMap){
		mDebugRenderMode = true;
		//mDebugRenderType = MeshLine;
	}
	if (bForceMeshLineDebug) {
		mDebugRenderMode = true;  
		mDebugRenderType = MeshLine;
	}
	mBlockWidthMin = 0;
	mBlockWidthMax = m_blocksXMinusOne;
	if(mInterleave.CurrentFrameInterleaveDrawing == false)
	{
		m_MaxCoord_WHWHOccluder = m_MaxCoord_WHWH;
		m_MinCoord_WHWHOccluder = _mm_setzero_si128();
		memset(m_pHiz, 0, m_HizBufferSize * sizeof(uint32_t)); //hizMin + hizMax, each one is uint16_t
	}
	else
	{
		mInterleave.InterleaveFrame++;
		mInterleave.mRenderRight = mInterleave.InterleaveFrame & 1;



		int PixelXMin = 0;
		int PixelXMax = m_width - 1;
		
		if (mInterleave.mRenderRight) 
		{
			mBlockWidthMin = mInterleave.mBlock_XRightStart;
			PixelXMin = mInterleave.mPixel_XRightStart;
		}
		else 
		{
			mBlockWidthMax = mInterleave.mBlock_XLeftEnd;
			PixelXMax = mInterleave.mPixel_XLeftEnd;
		}
		ZeroHizUpdateBlocks(m_pHiz+ mBlockWidthMin, this->m_blocksX, this->m_blocksY,  mBlockWidthMax - mBlockWidthMin + 1, m_HizBufferSize);

		m_MaxCoord_WHWHOccluder = _mm_setr_epi32(PixelXMax, m_height - 1, PixelXMax, m_height - 1);
		m_MinCoord_WHWHOccluder = _mm_setr_epi32(PixelXMin, 0, PixelXMin, 0);
	}
	if (mDebugRenderType != 0) 
	{	
		m_depthBufferPointLines.resize(m_totalPixels);
		memset(&m_depthBufferPointLines[0], 0, sizeof(uint16_t) * m_totalPixels);
	}
}
//only two cases to handle, PairBlockNum = 9
static inline uint64_t* GetMaskData(uint64_t * data, int bit)
{
	if (PairBlockNum == PureCheckerBoardApproach) return nullptr;
	//0->8  1--> 0
	return data + ((1^bit) << 3);
}

static inline __m128i* GetDepthData(uint64_t * data, int bit)
{
	if (PairBlockNum == PureCheckerBoardApproach) return (__m128i*) (data);

	return (__m128i*) (data + bit);
}



static uint64_t CheckerBoardTransform(uint64_t mask)
{
	//start from row/column 1
	static constexpr uint64_t blackPattern = 0x55AA55AA55AA55AA;
	static constexpr uint64_t whitePattern = 0xAA55AA55AA55AA55;
	static constexpr uint64_t oddColumn    = 0xAAAAAAAAAAAAAAAA;
	static constexpr uint64_t evenColumn   = 0x5555555555555555;

	static constexpr uint64_t oddBlack = (oddColumn & blackPattern);
	static constexpr uint64_t evenBlack = (evenColumn & blackPattern);
	static constexpr uint64_t evenWhite = (evenColumn & whitePattern);
	static constexpr uint64_t oddWhite = (oddColumn & whitePattern);

	uint64_t oddBlackMask = mask & oddBlack;
	uint64_t evenBlackMask = mask & evenBlack;
	uint64_t evenWhiteMask = mask & evenWhite;
	uint64_t oddWhiteMask = mask & oddWhite;

	uint64_t even = evenWhiteMask | (oddWhiteMask >> 1);
	uint64_t odd = oddBlackMask | (evenBlackMask << 1); //only black pixel are drawn

	return even | odd;
}

static void GrayCheckBoardWhitePixel(__m128i* TargetBlockData)
{
	uint16_t value = 65535;
	if (bDumpCheckerboardImageBinary) 
	{
		uint16_t* row = (uint16_t*)TargetBlockData;
		uint16_t binary[2];
		binary[0] = 0;
		binary[1] = value;
		for (int i = 0; i < 64; i++)
		{
			row[i] = binary[row[i] > 0];
		}
		return;
	}
	if (bDumpCheckerboardImageBlack) {
		value = 0;
	}
	if (bDumpCheckerboardImage)
	{
		uint16_t* row = (uint16_t*)TargetBlockData;
		for (int i = 0; i < 8; i++)
		{
			if (i & 1) {
				row[0] = row[2] = row[4] = row[6] = value;
			}
			else {
				row[1] = row[3] = row[5] = row[7] = value;
			}
			row += 8;
		}
	}
}
static void RecoverPartialBlockData(__m128i*TargetBlockData,  __m128i * depthRows, uint64_t* pbitMask)
{
	if (PairBlockNum != PureCheckerBoardApproach) 
	{
		
		__m128i min = _mm_set1_epi32(-1);
		for (int i = 0; i < 4; i++)
		{
			__m128i input = _mm_cmpeq_epi16(depthRows[i], _mm_set1_epi32(0));
			input = _mm_or_si128(input, depthRows[i]);
			min = _mm_min_epu16(min, input);
		}
		uint16_t minValue = _mm_min_epu16(min);
		if (_mm_min_epu16(min) == 65535) //all zero
		{
			//LOGI("Find zero min data block. SET to white");
			//minValue = 255 << 8;
			minValue = 25 ;
		}
		__m128i defaultV = _mm_set1_epi16(minValue);
		for (int i = 0; i < 4; i++) 
		{
			 __m128i t = _mm_srli_epi32(depthRows[i], 16);
			 __m128i a = _mm_or_si128(t, _mm_slli_epi32(t, 16));
			 t = _mm_slli_epi32(depthRows[i], 16);
			 __m128i b = _mm_or_si128(t, _mm_srli_epi32(t, 16));

			 __m128i zero = _mm_set1_epi32(0);
			 __m128i a0b = _mm_and_si128(b, _mm_cmpeq_epi16(a, zero));

			 __m128i r = _mm_or_si128(a, a0b);
			 a0b = _mm_and_si128(defaultV, _mm_cmpeq_epi16(r, zero));

			 TargetBlockData[(i << 1) | 1] = _mm_or_si128(r, a0b);
			 a0b = _mm_and_si128(a, _mm_cmpeq_epi16(b, zero));
			 
			 r = _mm_or_si128(b, a0b);
			 a0b = _mm_and_si128(defaultV, _mm_cmpeq_epi16(r, zero));

			 TargetBlockData[(i << 1)  ] = _mm_or_si128(r, a0b);
		}
		
		if (bDumpCheckerboardImage) 
		{
			GrayCheckBoardWhitePixel(TargetBlockData);
		}


		__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(CheckerBoardTransform(pbitMask[0]));
		//in case of checkerboard sample, the recovered depth might not be correct
		TargetBlockData[0] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[0]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[1] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[1]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[2] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[2]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[3] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[3]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[4] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[4]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[5] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[5]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[6] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[6]); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
		TargetBlockData[7] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), TargetBlockData[7]);
	}
	else 
	{
		TargetBlockData[0] = depthRows[0];
		TargetBlockData[1] = depthRows[0];
		TargetBlockData[2] = depthRows[1];
		TargetBlockData[3] = depthRows[1];
		TargetBlockData[4] = depthRows[2];
		TargetBlockData[5] = depthRows[2];
		TargetBlockData[6] = depthRows[3];
		TargetBlockData[7] = depthRows[3];
	}

	
}


void Rasterizer::onOccluderRenderFinish()
{

	//time to expand the depth map horizontally to cull more pixels


	int previousOcc = mLastOccluderNum;
	mLastOccluderNum = mCurrValidOccluderNum;
	if (mCurrValidOccluderNum == 0) 
	{
		//if (previousOcc != 0 && this->mInterleave.CurrentFrameInterleaveDrawing)   //this one has bug, m_pHiz might leak
		if (previousOcc != 0) 
		{
			memset(m_pHiz, 0, m_HizBufferSize * sizeof(uint32_t));
			if (bOccludeeBitScanOp)
			{
				memset(&mAnyDataBlockMask[0], 0, m_blocksY * sizeof(uint64_t));
			}
		}
		return;
	}

	//*****************************************************************************
	//calculate bin scan mask for first 1024 width, block 0 to block 127
	//*****************************************************************************

	if (mWidthIn1024 && bOccludeeBitScanOp) //width must be size of 64
	{
		auto Time1 = std::chrono::high_resolution_clock::now();
		

		int maxXStep = m_blocksX;
		uint32_t y = 0;
        
        //For interleave mode, only Half would be needed to updated.
		//32 * 64 = 2K operations could be saved!!
		uint64_t LastMask = 0;
		if (mInterleave.CurrentFrameInterleaveDrawing == true) {

			int DualBlockMinX = mBlockWidthMin >> 1;
			int DualBlockMaxX = mBlockWidthMax >> 1;
			uint64_t all = -1;
			uint64_t updateMask = (all << DualBlockMinX) & (all >> (63 - DualBlockMaxX));

			LastMask = all ^ updateMask;
		}

		int StartBlock = (mBlockWidthMin >> 3) << 3;
		uint32_t EndBlock = ((mBlockWidthMax + 7) >> 3) << 3;
		if (EndBlock > mBlockWidthMax) EndBlock -= 8;

		uint16_t* hizBufferStart = m_pHiz + StartBlock;
		int EndBlockIdx = EndBlock >> 1;
		int StartBlockIdx = StartBlock >> 1;  //the start update idx of uint64_t
		do
		{
			uint64_t rowMaskData0 = 0; //only care 0~63
			{
				__m128i * CurrentHIZ = (__m128i *)hizBufferStart;
				int x = StartBlockIdx;
				uint16_t th = bOccludeeMinDepthThreshold;
				do {
					__m128i pass = _mm_cmple_epu16_soc(_mm_set1_epi16(th), CurrentHIZ[0]);
					__m128i pass2 = _mm_srai_epi32(pass, 16);
					pass = _mm_and_si128(pass, pass2);
					uint32_t * result = (uint32_t *)& pass;

					uint32_t maskValue = (result[0] & 1) | (result[1] & 2) | (result[2] & 4) | (result[3] & 8);

					rowMaskData0 |= (uint64_t)(maskValue) << x;
					x += 4;
					CurrentHIZ++;// = 8;
				} while (x <= EndBlockIdx);
			}

			mAnyDataBlockMask[y] &= LastMask;
			mAnyDataBlockMask[y] |= rowMaskData0;

			hizBufferStart += maxXStep;

			y++;
		} while (y < m_blocksY);

		auto Time2 = std::chrono::high_resolution_clock::now();


		bool verify = false;
		if (verify) {
			maxXStep = m_width >> 3;
			uint16_t* hizBuffer = m_pHiz;
			y = 0;


			do
			{
				uint64_t rowMaskData0 = 0; //only care 0~63
				{
					uint16_t* CurrentHIZ = (uint16_t*)hizBuffer;
					int x = 0;
					uint16_t th = bOccludeeMinDepthThreshold;
					do {
						int k = x >> 1;
						rowMaskData0 |= ((uint64_t)(CurrentHIZ[0] >= th && CurrentHIZ[1] >= th)) << k; k++;
						rowMaskData0 |= ((uint64_t)(CurrentHIZ[2] >= th && CurrentHIZ[3] >= th)) << k; k++;
						rowMaskData0 |= ((uint64_t)(CurrentHIZ[4] >= th && CurrentHIZ[5] >= th)) << k; k++;
						rowMaskData0 |= ((uint64_t)(CurrentHIZ[6] >= th && CurrentHIZ[7] >= th)) << k;
						x += 8;
						CurrentHIZ += 8;
					} while (x < maxXStep);
				}

				hizBuffer += maxXStep;
				assert(mAnyDataBlockMask[y] == rowMaskData0);
				

				y++;
			} while (y < m_blocksY);


			auto Time3 = std::chrono::high_resolution_clock::now();
			int totalNSNEON32 = (int)std::chrono::duration_cast<std::chrono::nanoseconds>(Time3 - Time2).count();
			int totalNSNEON21 = (int)std::chrono::duration_cast<std::chrono::nanoseconds>(Time2 - Time1).count();
			std::cout << "Time21  " << totalNSNEON21 << "   time32   " << totalNSNEON32 << std::endl;
		}
	}
}



static inline int OnParentNodeQuery(unsigned int i_mesh, bool * results, uint16_t* mOccludeeTreeData)
{
	uint16_t treeSize = mOccludeeTreeData[i_mesh];
	if (treeSize > 1)
	{
		if (results[i_mesh] == false) 
		{
			int childNum = treeSize - 1;
			memset(results + i_mesh + 1, 0, childNum * sizeof(bool));
			return childNum;
		}
	}
	return 0;
}

template <bool bHasTreeData, bool OccludeeWidth1024>
void Rasterizer::batchQueryWithTree(const float * bbox, unsigned int nMesh, bool * results, bool obbMode) {
	const float *min = bbox;
	if (obbMode && bEnableOccludeeOBBQuery) {
		int perOccludeeStride = nOcludeeOBBSize; 
		if (mOccludeeTrueAsCulled == false) {
			if (ShowOccludeeInDepthMap == false)
			{
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += perOccludeeStride)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}
					results[i_mesh] = queryVisibility_OBB<0, OccludeeWidth1024, false>(min);
					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * perOccludeeStride;
					}
				}
			}
			else {
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += perOccludeeStride)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}

					results[i_mesh] = queryVisibility_OBB<0, OccludeeWidth1024, false>(min);

					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * perOccludeeStride;
					}

					if (mCurrentOccludee != -1)
					{
						mOccludeeResults.push_back(mCurrentOccludee);
						mCurrentOccludeeDepth >>= 8;

						mOccludeeResults.push_back(((uint64_t)mCurrentOccludeeDepth << 48) | (uint64_t)results[i_mesh]);
						mCurrentOccludee = -1;
					}
				}
			}



		}
		else {

			if (ShowOccludeeInDepthMap == false)
			{
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += perOccludeeStride)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}
					if (results[i_mesh] == false)
					{
						results[i_mesh] = queryVisibility_OBB<0, OccludeeWidth1024, false>(min);
					}
					else
					{
						results[i_mesh] = false;
					}

					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * perOccludeeStride;
					}
				}
			}
			else {
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += perOccludeeStride)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}

					if (results[i_mesh] == false)
					{
						results[i_mesh] = queryVisibility_OBB<0, OccludeeWidth1024, false>(min);
						if (bHasTreeData)
						{
							int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
							i_mesh += skipNum;
							min += skipNum * perOccludeeStride;
						}


						if (mCurrentOccludee != -1)
						{
							mOccludeeResults.push_back(mCurrentOccludee);
							mCurrentOccludeeDepth >>= 8;

							mOccludeeResults.push_back(((uint64_t)mCurrentOccludeeDepth << 48) | (uint64_t)results[i_mesh]);
							mCurrentOccludee = -1;
						}
					}
					else
					{
						results[i_mesh] = false;
					}
				}
			}
		}
		return;
	}
	if (mOccludeeTrueAsCulled == false) {
		if (ShowOccludeeInDepthMap == false)
		{			
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += BBOX_STRIDE)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}
					results[i_mesh] = queryVisibility<0, OccludeeWidth1024, false>(min, nullptr);
					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * BBOX_STRIDE;
					}
				}
		}
		else {
			for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += BBOX_STRIDE)
			{
				if (DebugOccluderOccludee)
				{
					DebugData[CurrentOccludeeIdx] = i_mesh;
				}
				
				results[i_mesh] = queryVisibility<0, OccludeeWidth1024, false>(min, nullptr);
				
				if (bHasTreeData)
				{
					int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
					i_mesh += skipNum;
					min += skipNum * BBOX_STRIDE;
				}

				if (mCurrentOccludee != -1)
				{
					mOccludeeResults.push_back(mCurrentOccludee);
					
					mCurrentOccludeeDepth >>= 8;

					mOccludeeResults.push_back(((uint64_t)mCurrentOccludeeDepth << 48) | (uint64_t)results[i_mesh]);
					mCurrentOccludee = -1;
				}
			}
		}



	}
	else {

		if (ShowOccludeeInDepthMap == false)
		{
				for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += BBOX_STRIDE)
				{
					if (DebugOccluderOccludee)
					{
						DebugData[CurrentOccludeeIdx] = i_mesh;
					}
					if (results[i_mesh] == false)
					{
						results[i_mesh] = queryVisibility<0, OccludeeWidth1024, false>(min, nullptr);
					}
					else
					{
						results[i_mesh] = false;
					}

					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * BBOX_STRIDE;
					}
				}
		}
		else {
			for (unsigned int i_mesh = 0; i_mesh < nMesh; ++i_mesh, min += BBOX_STRIDE)
			{
				if (DebugOccluderOccludee)
				{
					DebugData[CurrentOccludeeIdx] = i_mesh;
				}

				if (results[i_mesh] == false)
				{
					results[i_mesh] = queryVisibility<0, OccludeeWidth1024, false>(min, nullptr);
					if (bHasTreeData)
					{
						int skipNum = OnParentNodeQuery(i_mesh, results, this->mOccludeeTreeData);
						i_mesh += skipNum;
						min += skipNum * BBOX_STRIDE;
					}


					if (mCurrentOccludee != -1)
					{
						mOccludeeResults.push_back(mCurrentOccludee);
						
						mCurrentOccludeeDepth >>= 8;

						mOccludeeResults.push_back(((uint64_t)mCurrentOccludeeDepth << 48) | (uint64_t)results[i_mesh]);
						mCurrentOccludee = -1;
					}
				}
				else
				{
					results[i_mesh] = false;
				}
			}
		}
	}
}
template <bool OccludeeWidth1024>
void Rasterizer::batchQuery(const float * bbox, unsigned int nMesh, bool * results, bool obbMode)
{
	if (bDebugOccluderOnly) 
	{
		memset(DebugData, 0, DEBUG_DATA_SIZE * sizeof(uint32_t));
		return;
	}

	if (this->mOccludeeTreeData == nullptr) {
		batchQueryWithTree<false, OccludeeWidth1024>(bbox, nMesh, results, obbMode);
	}
	else 
	{
		batchQueryWithTree<true, OccludeeWidth1024>(bbox, nMesh, results, obbMode);
		mOccludeeTreeData = nullptr;
	}

	mOccludeeTrueAsCulled = false;


	if (DebugOccluderOccludee)
	{
		if (PrintOccludeeState)
		{
			bool specialDebug = 0;
			if (specialDebug)
			{


				LOGI(" QuadProcessed:           %d", DebugData[QuadProcessed]);
				LOGI(" QuadToTriangleMerge:           %d", DebugData[QuadToTriangleMerge]);
				LOGI(" QuadToTriangleSplit:           %d", DebugData[QuadToTriangleSplit]);



				LOGI("Query BlockAABBClipToZero:           %d", DebugData[BlockAABBClipToZero]);
				LOGI("Query BlockRenderPartial:           %d", DebugData[BlockRenderPartial]);
				LOGI("Query BlockRenderFull:           %d", DebugData[BlockRenderFull]);
				LOGI("Query BlockMaxLessThanMinCull:           %d", DebugData[BlockMaxLessThanMinCull]);
				LOGI("Query BlockMaskJointZeroCull:           %d", DebugData[BlockMaskJointZeroCull]);
				LOGI("Query BlockTotalPrimitives:           %d", DebugData[BlockTotalPrimitives]);
				

				//LOGI("****Render P4EarlyHizCull                         %d", DebugData[P4EarlyHizCull]);
				//LOGI("****Render P4EarlyHizCullPass                         %d", DebugData[P4EarlyHizCullPass]);
				//	
				//LOGI("****Render OccluderRasterized                         %d", DebugData[OccluderRasterized]);
				//LOGI("****Render RasterizedOccluderTotalTriangles                         %d", DebugData[RasterizedOccluderTotalTriangles]);
				//LOGI("****Render RasterizedOccluderTotalVertices                         %d", DebugData[RasterizedOccluderTotalVertices]);


				
				//LOGI("****Render BlockConvexRow00Cull                         %d", DebugData[BlockConvexRow00Cull]);
				//LOGI("****Render BlockConvexRow10Cull                         %d", DebugData[BlockConvexRow10Cull]);
				//LOGI("****Render BlockConvexEdge24Cull                         %d", DebugData[BlockConvexEdge24Cull]);
				//LOGI("****Render BlockConvexEdge31Cull                         %d", DebugData[BlockConvexEdge31Cull]);
				////LOGI("****Render BlockConvexEdge31CullRowCheck                         %d", DebugData[BlockConvexEdge31CullRowCheck]);
				//LOGI("****Render BlockConvexEdge31CullRowCheckPass                         %d", DebugData[BlockConvexEdge31CullRowCheckPass]);
				//LOGI("****Render BlockConvexAllZeroRow                         %d", DebugData[BlockConvexAllZeroRow]);
				////LOGI("****Render BlockTotal                         %d", DebugData[BlockTotal]);
				//LOGI("****Render BlockConvexRow00CullNextScanRows                         %d", DebugData[BlockConvexRow00CullNextScanRows]);
				//LOGI("****Render BlockConvexEdge24Check                         %d", DebugData[BlockConvexEdge24Check]);
			
				
			}
			else {
				int allOccludeeQuery = 
					DebugData[OccludeeFrustumCull] +
					DebugData[OccludeeNearClipPass] +
					DebugData[OccludeeQuery2d];


				LOGI("*************************************");
				
				
				LOGI("Render P4QuadSplit       %d", DebugData[P4QuadSplit]);
				LOGI("Render P4QuadFrustumCull       %d", DebugData[P4QuadFrustumCull]);
				LOGI("Render P4QuadFrustumPass       %d", DebugData[P4QuadFrustumPass]);
				
				LOGI("Query QueryBlockDoWhileIfSave       %d", DebugData[QueryBlockDoWhileIfSave]);
				LOGI("Query OccludeeFrustumCull       %d", DebugData[OccludeeFrustumCull]);
				LOGI("Query FastHalfPlaneCull         %d", DebugData[FastHalfPlaneCull]);
				LOGI("Query FastBlockHizCull          %d", DebugData[FastBlockHizCull]);
				LOGI("Query BlockPixelCull            %d", DebugData[BlockPixelCull]);
				LOGI("Query OccludeeCull              %d", DebugData[OccludeeCull]);

				LOGI("Query OccludeeQueryMaxPass              %d", DebugData[OccludeeQueryMaxPass]);
				LOGI("Query OccluderQueryMaxPass              %d", DebugData[OccluderQueryMaxPass]);
				
				LOGI("Query OccludeeNearClipPass      %d", DebugData[OccludeeNearClipPass]);
				LOGI("Query FastRowBitCull:           %d", DebugData[FastRowBitCull]);
				LOGI("Query FastBlockEmptyPass        %d", DebugData[FastBlockEmptyPass]);
				LOGI("Query BlockMaskPass             %d", DebugData[BlockMaskPass]);
				LOGI("Query OccludeeQueryMaxPass             %d", DebugData[OccludeeQueryMaxPass]);
				LOGI("Query BlockEmptyPass            %d", DebugData[BlockEmptyPass]);
				LOGI("Query BlockPixelPass            %d", DebugData[BlockPixelPass]);
				LOGI("Query InterleaveQuerySkipPass   %d", DebugData[InterleaveQuerySkipPass]);
				LOGI("Query OccludeeOnePixelExpandCheck -->   %d", DebugData[OccludeeOnePixelExpandCheck]);
				LOGI("Query OccludeeQuery2d:          %d/%d ", DebugData[OccludeeQuery2d], allOccludeeQuery);
				int totalPass = 0;
				totalPass += DebugData[OccludeeNearClipPass];
				totalPass += DebugData[FastRowBitCull];
				totalPass += DebugData[FastBlockEmptyPass];
				totalPass += DebugData[BlockMaskPass];
				totalPass += DebugData[BlockEmptyPass];
				totalPass += DebugData[BlockPixelPass]; 
				totalPass += DebugData[InterleaveQuerySkipPass]; 
				totalPass += DebugData[OccludeeQueryMaxPass];
				LOGI("Query OccludeeTotalPass:       %d/%d ", totalPass, allOccludeeQuery);
				LOGI("Query BlockRowCheck   %d", DebugData[BlockRowCheck]);

				LOGI("Query MaxOccludeeZ   %d", DebugData[MaxOccludeeZ]);
				LOGI("Query FastBlockDepthCompare   %d", DebugData[FastBlockDepthCompare]);

				LOGI("Query FastPlaneBlockDepthCompareSave   %d", DebugData[FastPlaneBlockDepthCompareSave]); 


				LOGI("Render OccluderCulled          %d", DebugData[OccluderCulled]);
				LOGI("Render OccluderRasterized      %d", DebugData[OccluderRasterized]);
				LOGI("Render P4CameraNearPlaneCull   %d", DebugData[P4CameraNearPlaneCull]);
				LOGI("Render P4BackFaceCull          %d", DebugData[P4BackFaceCull]);
				LOGI("Render P4FrustumCull           %d", DebugData[P4FrustumCull]);
				LOGI("Render P4EarlyHizCull          %d", DebugData[P4EarlyHizCull]);
				
				LOGI("Render PrimitiveCameraNearPlaneCull   %d", DebugData[PrimitiveCameraNearPlaneCull]);
				LOGI("Render PrimitiveFrustumCull           %d", DebugData[PrimitiveFrustumCull]);

				LOGI("Render PrimitiveBackFaceCull          %d", DebugData[PrimitiveBackFaceCull]);

				int p4Total = std::max<int>(1, DebugData[P4DrawTriangle]);
				LOGI("Render P4Rasterized              %d   Ratio to Total DrawTriangle %d%%", DebugData[P4Rasterized], 100 * DebugData[P4Rasterized] / p4Total);
				LOGI("Render P4Total                   %d", DebugData[P4Total]);
				LOGI("Render P4PassFrustumCull                   %d", DebugData[P4PassFrustumCull]);
				


				LOGI("Render P4DrawTriangle            %d", DebugData[P4DrawTriangle]);
				LOGI("Render P4PassCull            %d", DebugData[P4PassCull]);
				
				LOGI("Render P4Degenerate(valid0)      %d", DebugData[P4Valid0]);
				if (DebugData[P4Rasterized] > 0) {
					LOGI("Render P4Valid1                  %d    Ratio %d%% ", DebugData[P4Valid1], DebugData[P4Valid1] * 100 / DebugData[P4Rasterized]);
					LOGI("Render P4Valid2                  %d    Ratio %d%% ", DebugData[P4Valid2], DebugData[P4Valid2] * 100 / DebugData[P4Rasterized]);
					LOGI("Render P4Valid3                  %d    Ratio %d%% ", DebugData[P4Valid3], DebugData[P4Valid3] * 100 / DebugData[P4Rasterized]);
					LOGI("Render P4Valid4                  %d    Ratio %d%% ", DebugData[P4Valid4], DebugData[P4Valid4] * 100 / DebugData[P4Rasterized]);
				}
				

				LOGI("Render P4NearClipInput                   %d", DebugData[P4NearClipInput]);
				LOGI("Render PrimitiveNearClipeRasterized      %d", DebugData[PrimitiveNearClipeRasterized]);


				LOGI("Render PrimitiveEarlyHiZCull              %d", DebugData[PrimitiveEarlyHiZCull]);
				LOGI("Render PrimitiveDegenerateCull             %d", DebugData[PrimitiveDegenerateCull]);
				LOGI("Render PrimitiveTotalInput                %d", DebugData[PrimitiveTotalInput]);
				int primitiveTotalInput = std::max<int>(1, DebugData[PrimitiveTotalInput]);
				LOGI("Render PrimitiveRasterizedNum             %d  Ratio %d%%", DebugData[PrimitiveRasterizedNum], 100 * DebugData[PrimitiveRasterizedNum] / primitiveTotalInput);
				
				LOGI("Render Primitive total culled             %d", 
					DebugData[PrimitiveEarlyHiZCull]
					+ DebugData[PrimitiveCameraNearPlaneCull]
					+ DebugData[PrimitiveFrustumCull]
					+ DebugData[PrimitiveBackFaceCull]
					+ DebugData[PrimitiveDegenerateCull]);

				
				
				LOGI("Render BlockDoWhileIfSave                         %d", DebugData[BlockDoWhileIfSave]);
				LOGI("Render BlockTotal                         %d", DebugData[BlockTotal]);
				int blockTotal = std::max<int>(1, DebugData[BlockTotal]);
				LOGI("Render BlockConvexRow10CullOverhead            %d   Overhead Ratio to Total %d%%", DebugData[BlockConvexRow10CullOverhead], DebugData[BlockConvexRow10CullOverhead] * 100 / blockTotal);


				LOGI("****Render BlockConvexRow00Cull                         %d", DebugData[BlockConvexRow00Cull]);
				LOGI("****Render BlockConvexRow10Cull                         %d", DebugData[BlockConvexRow10Cull]);
				LOGI("****Render BlockConvexEdge24Cull                         %d", DebugData[BlockConvexEdge24Cull]);
				LOGI("****Render BlockConvexEdge31Cull                         %d", DebugData[BlockConvexEdge31Cull]);

				
				LOGI("Render  All Convex cull ratio:           ---->     %d%%", 
					(DebugData[BlockConvexRow00Cull]
						+ DebugData[BlockConvexRow10Cull]
					+ DebugData[BlockConvexEdge24Cull]
					+ DebugData[BlockConvexEdge31Cull])
					* 100 / blockTotal);


				LOGI("Render BlockRenderTotal %d%% + BlockMaxLessThanMinCull %d%% + BlockMaskJointZeroCull %d%% + BlockPrimitiveMaxLessThanMinCull %d%%  + BlockOneSureZeroCull %d%%  + BlockConvexRow10Cull %d%%  =  %d",
					100 * DebugData[BlockRenderTotal] / blockTotal,
					100 * DebugData[BlockMaxLessThanMinCull] / blockTotal,
					100 * DebugData[BlockMaskJointZeroCull] / blockTotal,
					100 * DebugData[BlockPrimitiveMaxLessThanMinCull] / blockTotal,
					100 * DebugData[BlockOneSureZeroCull] / blockTotal,
					100 * DebugData[BlockConvexRow10Cull] / blockTotal,
					DebugData[BlockRenderTotal] +
					DebugData[BlockMaxLessThanMinCull] +
					DebugData[BlockMaskJointZeroCull] +
					DebugData[BlockPrimitiveMaxLessThanMinCull] +
					DebugData[BlockOneSureZeroCull] +
					DebugData[BlockConvexRow10Cull] 
				);

				LOGI("Render BlockPrimitiveMaxLessThanMinCull   %d", DebugData[BlockPrimitiveMaxLessThanMinCull]);
				LOGI("Render BlockMaskJointZeroCull             %d", DebugData[BlockMaskJointZeroCull]);
				LOGI("Render BlockMaxLessThanMinCull            %d", DebugData[BlockMaxLessThanMinCull]);

				LOGI("Render BlockRenderTotal                   %d", DebugData[BlockRenderTotal]);
				LOGI("Render BlockRenderPartial                 %d", DebugData[BlockRenderPartial]);
				LOGI("Render BlockAABBClipToZero                %d", DebugData[BlockAABBClipToZero]);
				LOGI("Render BlockRenderFull                    %d", DebugData[BlockRenderFull]);
				LOGI("Render BlockRenderInitial                 %d", DebugData[BlockRenderInitial]);


				LOGI("Render BlockRenderInitialPartial          %d", DebugData[BlockRenderInitialPartial]);
				LOGI("Render BlockRenderInitialFull             %d", DebugData[BlockRenderInitialFull]);



				LOGI("Render BlockMinCompute4                   %d", DebugData[BlockMinCompute4]);
				LOGI("Render BlockMinUseOne                     %d", DebugData[BlockMinUseOne]);






				DebugData[BlockMinValidDepth] = 65535;

				for (uint32_t blockY = 0; blockY < m_blocksY; ++blockY)
				{
					int base = blockY * m_blocksX;
					for (uint32_t blockX = 0; blockX < m_blocksX; ++blockX)
					{
						int hizIdx = blockX + base;
						uint16_t hiz = m_pHiz[hizIdx];
						if (hiz > MIN_UPDATED_BLOCK_DEPTH2)
						{
							DebugData[BlockMinValidDepth] = std::min<uint16_t>(DebugData[BlockMinValidDepth], hiz);
						}

						if (hiz == 0)
						{
							DebugData[BlockEmptyBlock] ++;
							continue;
						}
					}
				}
				LOGI("Render BlockMinValidDepth %d", DebugData[BlockMinValidDepth]);
				LOGI("Render BlockEmptyBlock %d Ratio %d%%", DebugData[BlockEmptyBlock], 100 * DebugData[BlockEmptyBlock] / m_blockSize);



			}

		}

		memset(DebugData, 0, DEBUG_DATA_SIZE * sizeof(uint32_t));

	}
}



size_t Rasterizer::getMemoryUsage() const
{
	size_t memory = m_precomputedRasterTables.capacity() * sizeof(int64_t);
	memory += (m_depthBuffer.capacity() * sizeof(__m128i));
	
	return memory;
}

bool Rasterizer::inFrustum(__m128& center, __m128& extents, __m128* FrustumPlane)
{
	if (bFrustumCullIfClip == false) return true;
	// Bounding box center times 2 - but since W = 2, the plane equations work out correctly

	__m128 minusZero = _mm_set1_ps(-0.0f);

	// Compute distance from each frustum plane
	__m128 offset0 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[0], minusZero)));
	bool dist0 = _mm_dp_ps_float_soc(FrustumPlane[0], offset0) >= 0;

	__m128 offset1 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[1], minusZero)));
	bool dist1 = _mm_dp_ps_float_soc(FrustumPlane[1], offset1) >= 0;

	__m128 offset2 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[2], minusZero)));
	bool dist2 = _mm_dp_ps_float_soc(FrustumPlane[2], offset2) >= 0;

	__m128 offset3 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[3], minusZero)));
	bool dist3 = _mm_dp_ps_float_soc(FrustumPlane[3], offset3) >= 0;

	__m128 offset4 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[4], minusZero)));
	bool dist4 = _mm_dp_ps_float_soc(FrustumPlane[4], offset4) >= 0;

	__m128 offset5 = _mm_add_ps(center, _mm_xor_ps(extents, _mm_and_ps(FrustumPlane[5], minusZero)));
	bool dist5 = _mm_dp_ps_float_soc(FrustumPlane[5], offset5) >= 0;

	// Combine plane distance signs
	bool combined = (dist0 & dist1) & (dist2 & dist3) & (dist4 & dist5);
	return combined;
}
static inline __m128 _mm_rcp_ps_div(__m128 _A) {
	return _mm_div_ps(_mm_set1_ps(1.0), _A);
}
bool Rasterizer::rasterizeOccludee(__m128* corners)
{
	if (bEnableOccludeeOBBQuery == false) return true; //need to active obb query feature first
		corners[2] = _mm_rcp_ps(corners[3]);
		corners[6] = _mm_rcp_ps(corners[7]);

		__m128 lowerBound = _mm_set1_ps(-maxInvW);
		__m128 upperBound = _mm_set1_ps(+maxInvW);
		corners[2] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, corners[2]));
		corners[6] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, corners[6]));


		auto one8th = _mm_set1_ps(0.125f); //in the format of block
		__m128 temp = _mm_mul_ps(corners[2], one8th);
		corners[0] = _mm_mul_ps(corners[0], temp);
		corners[1] = _mm_mul_ps(corners[1], temp);

		temp = _mm_mul_ps(corners[6], one8th);
		corners[4] = _mm_mul_ps(corners[4], temp);
		corners[5] = _mm_mul_ps(corners[5], temp);

		//          2————————3
		//         /|       /|  
		//        / |      / |
		//       /  6-----/--7 
		//      0————————1  / 
		//      | /      | / 
		//      |/       |/ 
		//      4————————5


		__m128 x[3];
		__m128 y[3];
		__m128 W[3];
		__m128 invW[3];
		
		//**********************************************************************************
		float* allData = (float*)corners;
		float* allXf = allData;
		float* allYf = allData+4;
		float* allWInvf = allData+8;
		float* allWf = allData+12;

		OccluderRenderCache cache;
		cache.c0 = _mm_set1_ps(0);
		float* matF = (float*)m_OccludeelocalToClip;
		cache.c1 = _mm_set1_ps(matF[3 * 4 + 2]);

		//batch 1
		{
			//2, 0, 3,
			//0, 1, 3,
			//4, 6, 7,
			//5, 4, 7,
			const int all_index_0 = 2;		const int all_index_1 = 0;		const int all_index_2 = 3;
			const int all_index_3 = 0;		const int all_index_4 = 1;		const int all_index_5 = 3;
			const int all_index_6 = 16;		const int all_index_7 = 18;		const int all_index_8 = 19;
			const int all_index_9 = 17;		const int all_index_10 = 16;	const int all_index_11 = 19;
			x[2] = _mm_setr_ps(allXf[all_index_0], allXf[all_index_3], allXf[all_index_6], allXf[all_index_9]);
			x[1] = _mm_setr_ps(allXf[all_index_1], allXf[all_index_4], allXf[all_index_7], allXf[all_index_10]);
			x[0] = _mm_setr_ps(allXf[all_index_2], allXf[all_index_5], allXf[all_index_8], allXf[all_index_11]);
			y[2] = _mm_setr_ps(allYf[all_index_0], allYf[all_index_3], allYf[all_index_6], allYf[all_index_9]);
			y[1] = _mm_setr_ps(allYf[all_index_1], allYf[all_index_4], allYf[all_index_7], allYf[all_index_10]);
			y[0] = _mm_setr_ps(allYf[all_index_2], allYf[all_index_5], allYf[all_index_8], allYf[all_index_11]);
			invW[2] = _mm_setr_ps(allWInvf[all_index_0], allWInvf[all_index_3], allWInvf[all_index_6], allWInvf[all_index_9]);
			invW[1] = _mm_setr_ps(allWInvf[all_index_1], allWInvf[all_index_4], allWInvf[all_index_7], allWInvf[all_index_10]);
			invW[0] = _mm_setr_ps(allWInvf[all_index_2], allWInvf[all_index_5], allWInvf[all_index_8], allWInvf[all_index_11]);
			W[2] = _mm_setr_ps(allWf[all_index_0], allWf[all_index_3], allWf[all_index_6], allWf[all_index_9]);
			W[1] = _mm_setr_ps(allWf[all_index_1], allWf[all_index_4], allWf[all_index_7], allWf[all_index_10]);
			W[0] = _mm_setr_ps(allWf[all_index_2], allWf[all_index_5], allWf[all_index_8], allWf[all_index_11]);
			__m128 W0W1W2 = _mm_and_ps(_mm_and_ps(W[0], W[1]), (W[2]));
			__m128 primitiveValid = _mm_xor_ps(W0W1W2, _mm_set1_ps(-0.0f));
			drawTriangle<true, false, true>(x, y, invW, W, primitiveValid, &cache);
			if (cache.mRasterizedOccludeeVisible) return true;
		}
		//batch 2
		{
			//0, 2, 6,
			//4, 0, 6,
			//3, 1, 7,
			//1, 5, 7,
			const int all_index_0 = 0;		const int all_index_1 = 2;			const int all_index_2 = 18;
			const int all_index_3 = 16;		const int all_index_4 = 0;  		const int all_index_5 = 18;
			const int all_index_6 = 3;		const int all_index_7 = 1;			const int all_index_8 = 19;
			const int all_index_9 = 1;		const int all_index_10 = 17;		const int all_index_11 = 19;
			x[2] = _mm_setr_ps(allXf[all_index_0], allXf[all_index_3], allXf[all_index_6], allXf[all_index_9]);
			x[1] = _mm_setr_ps(allXf[all_index_1], allXf[all_index_4], allXf[all_index_7], allXf[all_index_10]);
			x[0] = _mm_setr_ps(allXf[all_index_2], allXf[all_index_5], allXf[all_index_8], allXf[all_index_11]);
			y[2] = _mm_setr_ps(allYf[all_index_0], allYf[all_index_3], allYf[all_index_6], allYf[all_index_9]);
			y[1] = _mm_setr_ps(allYf[all_index_1], allYf[all_index_4], allYf[all_index_7], allYf[all_index_10]);
			y[0] = _mm_setr_ps(allYf[all_index_2], allYf[all_index_5], allYf[all_index_8], allYf[all_index_11]);
			invW[2] = _mm_setr_ps(allWInvf[all_index_0], allWInvf[all_index_3], allWInvf[all_index_6], allWInvf[all_index_9]);
			invW[1] = _mm_setr_ps(allWInvf[all_index_1], allWInvf[all_index_4], allWInvf[all_index_7], allWInvf[all_index_10]);
			invW[0] = _mm_setr_ps(allWInvf[all_index_2], allWInvf[all_index_5], allWInvf[all_index_8], allWInvf[all_index_11]);
			W[2] = _mm_setr_ps(allWf[all_index_0], allWf[all_index_3], allWf[all_index_6], allWf[all_index_9]);
			W[1] = _mm_setr_ps(allWf[all_index_1], allWf[all_index_4], allWf[all_index_7], allWf[all_index_10]);
			W[0] = _mm_setr_ps(allWf[all_index_2], allWf[all_index_5], allWf[all_index_8], allWf[all_index_11]);
			__m128 W0W1W2 = _mm_and_ps(_mm_and_ps(W[0], W[1]), (W[2]));
			__m128 primitiveValid = _mm_xor_ps(W0W1W2, _mm_set1_ps(-0.0f));
			drawTriangle<true, false, true>(x, y, invW, W, primitiveValid, &cache);
			if (cache.mRasterizedOccludeeVisible) return true;
		}

		//batch 3
		{
			//0, 4, 5,
			//1, 0, 5,
			//6, 2, 7,
			//2, 3, 7,
			const int all_index_0 = 0;		const int all_index_1 = 16;		const int all_index_2 = 17;
			const int all_index_3 = 1;		const int all_index_4 = 0;		const int all_index_5 = 17;
			const int all_index_6 = 18;		const int all_index_7 = 2;		const int all_index_8 = 19;
			const int all_index_9 = 2;		const int all_index_10 = 3;		const int all_index_11 = 19;
			x[2] = _mm_setr_ps(allXf[all_index_0], allXf[all_index_3], allXf[all_index_6], allXf[all_index_9]);
			x[1] = _mm_setr_ps(allXf[all_index_1], allXf[all_index_4], allXf[all_index_7], allXf[all_index_10]);
			x[0] = _mm_setr_ps(allXf[all_index_2], allXf[all_index_5], allXf[all_index_8], allXf[all_index_11]);
			y[2] = _mm_setr_ps(allYf[all_index_0], allYf[all_index_3], allYf[all_index_6], allYf[all_index_9]);
			y[1] = _mm_setr_ps(allYf[all_index_1], allYf[all_index_4], allYf[all_index_7], allYf[all_index_10]);
			y[0] = _mm_setr_ps(allYf[all_index_2], allYf[all_index_5], allYf[all_index_8], allYf[all_index_11]);
			invW[2] = _mm_setr_ps(allWInvf[all_index_0], allWInvf[all_index_3], allWInvf[all_index_6], allWInvf[all_index_9]);
			invW[1] = _mm_setr_ps(allWInvf[all_index_1], allWInvf[all_index_4], allWInvf[all_index_7], allWInvf[all_index_10]);
			invW[0] = _mm_setr_ps(allWInvf[all_index_2], allWInvf[all_index_5], allWInvf[all_index_8], allWInvf[all_index_11]);
			W[2] = _mm_setr_ps(allWf[all_index_0], allWf[all_index_3], allWf[all_index_6], allWf[all_index_9]);
			W[1] = _mm_setr_ps(allWf[all_index_1], allWf[all_index_4], allWf[all_index_7], allWf[all_index_10]);
			W[0] = _mm_setr_ps(allWf[all_index_2], allWf[all_index_5], allWf[all_index_8], allWf[all_index_11]);
			__m128 W0W1W2 = _mm_and_ps(_mm_and_ps(W[0], W[1]), (W[2]));
			__m128 primitiveValid = _mm_xor_ps(W0W1W2, _mm_set1_ps(-0.0f));
			drawTriangle<true, false, true>(x, y, invW, W, primitiveValid, &cache);
		}
		return cache.mRasterizedOccludeeVisible;
}


bool Rasterizer::rasterizeOccludeeQuad(__m128* corners)
{
	if (bEnableOccludeeOBBQuery == false) return true; //need to active obb query feature first
	corners[2] = _mm_rcp_ps(corners[3]);
	__m128 lowerBound = _mm_set1_ps(-maxInvW);
	__m128 upperBound = _mm_set1_ps(+maxInvW);
	corners[2] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, corners[2]));

	auto one8th = _mm_set1_ps(0.125f); //in the format of block
	__m128 temp = _mm_mul_ps(corners[2], one8th);
	corners[0] = _mm_mul_ps(corners[0], temp);
	corners[1] = _mm_mul_ps(corners[1], temp);

	//          2————————3
	//         /        /  
	//        /        /
	//       /        /
	//      0————————1
	__m128 x[3];
	__m128 y[3];
	__m128 W[3];
	__m128 invW[3];
	//**********************************************************************************
	float* allData = (float*)corners;
	OccluderRenderCache cache;
	cache.c0 = _mm_set1_ps(0);
	float* matF = (float*)m_OccludeelocalToClip;
	cache.c1 = _mm_set1_ps(matF[3 * 4 + 2]);

	//2, 0, 3,
	//0, 1, 3,
	const int all_index_0 = 2;		const int all_index_1 = 0;		const int all_index_2 = 3;
	const int all_index_3 = 0;		const int all_index_4 = 1;		const int all_index_5 = 3;
	x[2] = _mm_setr_ps(allData[all_index_0], allData[all_index_3], 0, 0);
	x[1] = _mm_setr_ps(allData[all_index_1], allData[all_index_4], 0, 0);
	x[0] = _mm_setr_ps(allData[all_index_2], allData[all_index_5], 0, 0);
	y[2] = _mm_setr_ps(allData[all_index_0 + 4], allData[all_index_3 + 4], 0, 0);
	y[1] = _mm_setr_ps(allData[all_index_1 + 4], allData[all_index_4 + 4], 0, 0);
	y[0] = _mm_setr_ps(allData[all_index_2 + 4], allData[all_index_5 + 4], 0, 0);
	invW[2] = _mm_setr_ps(allData[all_index_0 + 8], allData[all_index_3 + 8], 0, 0);
	invW[1] = _mm_setr_ps(allData[all_index_1 + 8], allData[all_index_4 + 8], 0, 0);
	invW[0] = _mm_setr_ps(allData[all_index_2 + 8], allData[all_index_5 + 8], 0, 0);
	W[2] = _mm_setr_ps(allData[all_index_0 + 12], allData[all_index_3 + 12], -0, -0);
	W[1] = _mm_setr_ps(allData[all_index_1 + 12], allData[all_index_4 + 12], -0, -0);
	W[0] = _mm_setr_ps(allData[all_index_2 + 12], allData[all_index_5 + 12], -0, -0);
	__m128 W0W1W2 = _mm_and_ps(_mm_and_ps(W[0], W[1]), (W[2]));
	__m128 primitiveValid = _mm_xor_ps(W0W1W2, _mm_set1_ps(-0.0f));

	drawTriangle<true, false, true>(x, y, invW, W, primitiveValid, &cache);
	return cache.mRasterizedOccludeeVisible;
}


//for occludee obb query only
template <bool bOccludeeWidth1024, bool multiThreadQuery>
bool Rasterizer::queryVisibility_QUAD(const float* obb)
{
	if (bEnableOccludeeOBBQuery == false) return true; //need to active obb query feature first
	// Transform edges
	__m128 corners[4];

	//          2————————3
	//         /        /  
	//        /        /
	//       /        /
	//      0————————1

	__m128* localToClip = this->m_OccludeelocalToClip;
	corners[0] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[0]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[1]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[2]), localToClip[3])));
	corners[1] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[3]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[4]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[5]), localToClip[3])));
	corners[2] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[6]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[7]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[8]), localToClip[3])));
	corners[3] = _mm_add_ps(corners[1], _mm_sub_ps(corners[2], corners[0]));
	

	// Transpose into SoA
	_MM_TRANSPOSE4_PS(corners[0], corners[1], corners[2], corners[3]);


	bool requireClip = false;
	__m128 closeToNearPlane = _mm_cmplt_ps(corners[3], _mm_set1_ps(mNearPlane));
	requireClip = !_mm_same_sign0(closeToNearPlane);


	if (requireClip)
	{
		if (_mm_same_sign1_soc(corners[3]))
		{
			return false;
		}

		//__m128 obbPoints[4];
		//obbPoints[0] = _mm_setr_ps(obb[0], obb[1], obb[2], 1.0);
		//obbPoints[1] = _mm_setr_ps(obb[3], obb[4], obb[5], 1.0);
		//obbPoints[2] = _mm_setr_ps(obb[6], obb[7], obb[8], 1.0);
		//obbPoints[3] = _mm_sub_ps(_mm_add_ps(obbPoints[1], obbPoints[2]), obbPoints[0]);
		//__m128 min = _mm_min_ps(_mm_min_ps(obbPoints[0], obbPoints[1]), _mm_min_ps(obbPoints[2], obbPoints[3]));
		//__m128 minmax_sum = _mm_add_ps(obbPoints[1], obbPoints[2]);
		//__m128 extents = _mm_sub_ps(_mm_sub_ps(minmax_sum, min), min);

		__m128 min = _mm_setr_ps(obb[12], obb[13], obb[14], 1);
		__m128 max = _mm_setr_ps(obb[15], obb[16], obb[17], 1);
		__m128 extents = _mm_sub_ps(max, min);
		__m128 minmax_sum = _mm_add_ps(min, max);

		if (inFrustum(minmax_sum, extents, localToClip+4) == false) {
			return false;
		}

		if (Rasterize_ClippedOccludee_AS_OCCLUDER_OBB) {
			if (obb[10] == 1.0) { //if 10th value set to 1, continue to rasterizeQuad
				return rasterizeOccludeeQuad(corners);
			}
			return true;
		}
	}

	bool visible = requireClip;
	if (requireClip == false)  //visibility still unknown
	{

		// Perspective division
		corners[3] = _mm_rcp_ps(corners[3]);
		corners[0] = _mm_mul_ps(corners[0], corners[3]);
		corners[1] = _mm_mul_ps(corners[1], corners[3]);
		corners[2] = _mm_mul_ps(corners[2], corners[3]);

		// Vertical mins and maxes
		__m128 minsX = corners[0];
		__m128 maxsX = corners[0];

		__m128 minsY = corners[1];
		__m128 maxsY = corners[1];
		__m128i	minsXY = _mm_cvttps_epi32(_mm_min_ps(_mm_unpacklo_ps(minsX, minsY), _mm_unpackhi_ps(minsX, minsY)));
		__m128i	maxsXY = _mm_cvttps_epi32(_mm_max_ps(_mm_unpacklo_ps(maxsX, maxsY), _mm_unpackhi_ps(maxsX, maxsY)));

		__m128i minThreshold = _mm_set1_epi32(0);

		minsXY = _mm_max_epi32(minsXY, minThreshold);
		//already frustum culled. limit the max of minXY
		maxsXY = _mm_min_epi32(maxsXY, m_MaxCoordOccludee_WHWH);


		// Horizontal reduction, step 2
		__m128i minXYMaxXYLo = _mm_unpacklo_epi32(minsXY, maxsXY);
		__m128i minXYMaxXYHi = _mm_unpackhi_epi32(minsXY, maxsXY);

		__m128i boundsI[2];
		boundsI[0] = _mm_min_epi32(minXYMaxXYLo, minXYMaxXYHi);
		boundsI[1] = _mm_max_epi32(minXYMaxXYLo, minXYMaxXYHi);


		int* bounds = (int*)boundsI;

		//frustum culling
		//very useful for interleave mode occluder
		{
			if (bounds[0] > bounds[5] || bounds[2] > bounds[7])
			{
				return false;
			}
		}


		{
			uint32_t minX = bounds[0];
			uint32_t maxX = bounds[5];
			uint32_t minY = bounds[2];
			uint32_t maxY = bounds[7];
			__m128i depth = packQueryDepth(corners[2]);
			uint16_t maxZ = _mm_max_epu16_even(depth);
			if (ShowOccludeeInDepthMap && multiThreadQuery == false)
			{
				mCurrentOccludee = ((uint64_t)minX << 48) | ((uint64_t)maxX << 32) | ((uint64_t)minY << 16) | (uint64_t)maxY;
				mCurrentOccludeeDepth = maxZ;
			}
			visible = query2D<false, bOccludeeWidth1024>(minX, maxX, minY, maxY, maxZ);
		}

	}

	//refactor previous multiple exit to this single exit
	return visible;
}


//for occludee obb query only
template <bool bQueryOccluder, bool bOccludeeWidth1024, bool multiThreadQuery>
bool Rasterizer::queryVisibility_OBB(const float* obb)
{
	if (bEnableOccludeeOBBQuery == false) return true; //need to active obb query feature first
	if (obb[9] == std::numeric_limits<float>::infinity()) {
		return queryVisibility_QUAD<bOccludeeWidth1024, multiThreadQuery>(obb);
	}
	// Transform edges
	__m128 corners[8];

//          2————————3
//         /|       /|  
//        / |      / |
//       /  6-----/--7 
//      0————————1  / 
//      | /      | / 
//      |/       |/ 
//      4————————5

	__m128* localToClip = this->m_OccludeelocalToClip;
	corners[0] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[0]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[1]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[2]), localToClip[3])));
	corners[1] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[3]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[4]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[5]), localToClip[3])));
	corners[2] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[6]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[7]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[8]), localToClip[3])));
	corners[3] = _mm_add_ps(corners[1], _mm_sub_ps(corners[2], corners[0]));
	corners[4] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(obb[9]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(obb[10]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(obb[11]), localToClip[3])));

	__m128 edge40 = _mm_sub_ps(corners[4], corners[0]);
	corners[5] = _mm_add_ps(corners[1], edge40);
	corners[6] = _mm_add_ps(corners[2], edge40);
	corners[7] = _mm_add_ps(corners[3], edge40);

	// Transpose into SoA
	_MM_TRANSPOSE4_PS(corners[0], corners[1], corners[2], corners[3]);
	_MM_TRANSPOSE4_PS(corners[4], corners[5], corners[6], corners[7]);



	bool requireClip = false;
	__m128 closeToNearPlane = _mm_cmplt_ps(_mm_min_ps(corners[3], corners[7]), _mm_set1_ps(mNearPlane));
	requireClip = !_mm_same_sign0(closeToNearPlane);
	

	if (requireClip)
	{
		__m128 depthJoint = _mm_and_ps(corners[3], corners[7]);
		if (_mm_same_sign1_soc(depthJoint))
		{
			return false;
		}

		__m128 min;
		__m128 max;
		if(nOcludeeOBBSize == 12)
		{
			__m128 obbPoints[8];
			obbPoints[0] = _mm_setr_ps(obb[0], obb[1], obb[2], 1.0);
			obbPoints[1] = _mm_setr_ps(obb[3], obb[4], obb[5], 1.0);
			obbPoints[2] = _mm_setr_ps(obb[6], obb[7], obb[8], 1.0);
			obbPoints[3] = _mm_sub_ps(_mm_add_ps(obbPoints[1], obbPoints[2]), obbPoints[0]);

			obbPoints[4] = _mm_setr_ps(obb[9], obb[10], obb[11], 1.0);
			__m128 d40 = _mm_sub_ps(obbPoints[4], obbPoints[0]);
			obbPoints[5] = _mm_add_ps(obbPoints[1], d40);
			obbPoints[6] = _mm_add_ps(obbPoints[2], d40);
			obbPoints[7] = _mm_add_ps(obbPoints[3], d40);


			min = obbPoints[0];
			max = obbPoints[0];
			for (int idx = 1; idx <= 7; idx++) {
				min = _mm_min_ps(min, obbPoints[idx]);
				max = _mm_max_ps(max, obbPoints[idx]);
			}
		}
		else {
			min = _mm_setr_ps(obb[12], obb[13], obb[14], 1);
			max = _mm_setr_ps(obb[15], obb[16], obb[17], 1);
		}		

		__m128 extents = _mm_sub_ps(max, min);

		__m128* FrustumPlane =  localToClip+4;
		__m128 min_max_sum = _mm_add_ps(min, max);
		if (inFrustum(min_max_sum, extents, FrustumPlane) == false) {
			return false;
		}

		if (Rasterize_ClippedOccludee_AS_OCCLUDER_OBB) {
			return rasterizeOccludee(corners);
		}
	}

	bool visible = requireClip;
	if (requireClip == false)  //visibility still unknown
	{

		// Perspective division
		corners[3] = _mm_rcp_ps(corners[3]);
		corners[0] = _mm_mul_ps(corners[0], corners[3]);
		corners[1] = _mm_mul_ps(corners[1], corners[3]);
		corners[2] = _mm_mul_ps(corners[2], corners[3]);

		corners[7] = _mm_rcp_ps(corners[7]);
		corners[4] = _mm_mul_ps(corners[4], corners[7]);
		corners[5] = _mm_mul_ps(corners[5], corners[7]);
		corners[6] = _mm_mul_ps(corners[6], corners[7]);

		// Vertical mins and maxes
		__m128 minsX = _mm_min_ps(corners[0], corners[4]);
		__m128 maxsX = _mm_max_ps(corners[0], corners[4]);

		__m128 minsY = _mm_min_ps(corners[1], corners[5]);
		__m128 maxsY = _mm_max_ps(corners[1], corners[5]);
		__m128i	minsXY = _mm_cvttps_epi32(_mm_min_ps(_mm_unpacklo_ps(minsX, minsY), _mm_unpackhi_ps(minsX, minsY)));
		__m128i	maxsXY = _mm_cvttps_epi32(_mm_max_ps(_mm_unpacklo_ps(maxsX, maxsY), _mm_unpackhi_ps(maxsX, maxsY)));

		__m128i minThreshold = _mm_set1_epi32(0);

		minsXY = _mm_max_epi32(minsXY, minThreshold);
		//already frustum culled. limit the max of minXY
		maxsXY = _mm_min_epi32(maxsXY, m_MaxCoordOccludee_WHWH);
		

		// Horizontal reduction, step 2
		__m128i minXYMaxXYLo = _mm_unpacklo_epi32(minsXY, maxsXY);
		__m128i minXYMaxXYHi = _mm_unpackhi_epi32(minsXY, maxsXY);

		__m128i boundsI[2];
		boundsI[0] = _mm_min_epi32(minXYMaxXYLo, minXYMaxXYHi);
		boundsI[1] = _mm_max_epi32(minXYMaxXYLo, minXYMaxXYHi);


		int* bounds = (int*)boundsI;

		//frustum culling
		//very useful for interleave mode occluder
		{
			if (bounds[0] > bounds[5] || bounds[2] > bounds[7])
			{
				if (DebugOccluderOccludee)
				{
					this->DebugData[OccluderCulled] += bQueryOccluder;
				}
				return false;
			}
		}


		{
			uint32_t minX = bounds[0];
			uint32_t maxX = bounds[5];
			uint32_t minY = bounds[2];
			uint32_t maxY = bounds[7];
			__m128i depth = packQueryDepth(_mm_max_ps(corners[2], corners[6]));
			uint16_t maxZ = _mm_max_epu16_even(depth);			
			if (ShowOccludeeInDepthMap && multiThreadQuery == false)
			{
				mCurrentOccludee = ((uint64_t)minX << 48) | ((uint64_t)maxX << 32) | ((uint64_t)minY << 16) | (uint64_t)maxY;
				mCurrentOccludeeDepth = maxZ;
			}
			visible = query2D<false, bOccludeeWidth1024>(minX, maxX, minY, maxY, maxZ);
		}

	}
	

	if (DebugOccluderOccludee)
	{
		this->DebugData[OccluderCulled] += bQueryOccluder && (visible == false);
	}
	//refactor previous multiple exit to this single exit
	return visible;
}

void Rasterizer::prepareOccludeeRasterization(const float* data, OccluderRenderCache* occ)
{
	const float* minExtents = data + 2; //minExtents
	__m128* localToClip = occ->m_localToClipPointer;

	// Transform edges
	__m128 egde0 = _mm_mul_ps(localToClip[0], _mm_set1_ps(minExtents[3]));
	__m128 egde1 = _mm_mul_ps(localToClip[1], _mm_set1_ps(minExtents[4]));
	__m128 egde2 = _mm_mul_ps(localToClip[2], _mm_set1_ps(minExtents[5]));
	__m128 corners4 =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(minExtents[0]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(minExtents[1]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(minExtents[2]), localToClip[3])));
	occ->mat[3] = corners4; // as clipped occludee might need to go through rasterize path

	float w4 = ((float*)&corners4)[3];
	float e04 = ((float*)&egde0)[3];
	float e14 = ((float*)&egde1)[3];
	float e24 = ((float*)&egde2)[3];

	w4 = std::min(w4, w4 + e04);
	w4 = std::min(w4, w4 + e14);
	w4 = std::min(w4, w4 + e24);


	occ->NeedsClipping = w4 < mNearPlane;
	occ->prepareCache(egde0, egde1, egde2);


	memcpy(occ->FullMeshInvExtents, data + 8, 8 * sizeof(float));
}

template <bool bQueryOccluder, bool bOccludeeWidth1024, bool multiThreadQuery>
bool Rasterizer::queryVisibility(const float* minmaxf, OccluderRenderCache* occ)
{
    // Frustum cull
	__m128 extents;
	if (bQueryOccluder)
	{
		extents = _mm_setr_ps(minmaxf[3], minmaxf[4] , minmaxf[5] , 0);
	}
	else 
	{
		if (bTestQUADByAABB) {
			bool testobb = false;
			float obb[18];
			//p0
			obb[0] = minmaxf[0];
			obb[1] = minmaxf[1];
			obb[2] = minmaxf[5];

			//p1
			obb[3] = minmaxf[3];
			obb[4] = minmaxf[1];
			obb[5] = minmaxf[5];

			//p2
			obb[6] = minmaxf[0];
			obb[7] = minmaxf[4];
			obb[8] = minmaxf[5];

			//p4
			obb[9] = std::numeric_limits<float>::infinity();;
			obb[10] = std::numeric_limits<float>::infinity();;
			obb[11] = std::numeric_limits<float>::infinity();;
			memcpy(obb + 12, minmaxf, 6 * sizeof(float));
			
			testobb = queryVisibility_OBB< bQueryOccluder, bOccludeeWidth1024, multiThreadQuery>(obb);
			return testobb;
		}
		if (bTestObbByAABB) {
			bool testobb = false;
			float obb[18];
			//p0
			obb[0] = minmaxf[0];
			obb[1] = minmaxf[1];
			obb[2] = minmaxf[5];

			//p1
			obb[3] = minmaxf[3];
			obb[4] = minmaxf[1];
			obb[5] = minmaxf[5];

			//p2
			obb[6] = minmaxf[0];
			obb[7] = minmaxf[4];
			obb[8] = minmaxf[5];

			//p4
			obb[9] = minmaxf[0];
			obb[10] = minmaxf[1];
			obb[11] = minmaxf[2];
			memcpy(obb + 12, minmaxf, 6 * sizeof(float));
			testobb = queryVisibility_OBB< bQueryOccluder, bOccludeeWidth1024, multiThreadQuery>(obb);
			return testobb;
		}
		
		extents = _mm_setr_ps(minmaxf[3] - minmaxf[0], minmaxf[4] - minmaxf[1], minmaxf[5] - minmaxf[2], 0);
	}
	
	
	__m128* localToClip = m_OccludeelocalToClip;
	if (bQueryOccluder) localToClip = occ->m_localToClipPointer;

    // Transform edges
    __m128 egde0 = _mm_mul_ps_scalar_soc(localToClip[0], extents, 0);
	__m128 egde1 = _mm_mul_ps_scalar_soc(localToClip[1], extents, 1);
	__m128 egde2 = _mm_mul_ps_scalar_soc(localToClip[2], extents, 2);
	__m128 corners[8];

//          2————————3
//         /|       /|  
//        / |      / |
//       /  6-----/--7 
//      0————————1  / 
//      | /      | / 
//      |/       |/ 
//      4————————5
// 
	// Transform first corner
	corners[4] =
		_mm_fmadd_ps_soc(localToClip[0], _mm_set1_ps(minmaxf[0]),
			_mm_fmadd_ps_soc(localToClip[1], _mm_set1_ps(minmaxf[1]),
				_mm_fmadd_ps_soc(localToClip[2], _mm_set1_ps(minmaxf[2]), localToClip[3])));

	if (bQueryOccluder)
	{
		occ->mat[3] = corners[4]; // as clipped occludee might need to go through rasterize path
	}

	
	// Transform remaining corners by adding edge vectors

	corners[5] = _mm_add_ps(corners[4], egde0);
	corners[6] = _mm_add_ps(corners[4], egde1);
	corners[7] = _mm_add_ps(corners[5], egde1);

	corners[0] = _mm_add_ps(corners[4], egde2);
	corners[1] = _mm_add_ps(corners[5], egde2);
	corners[2] = _mm_add_ps(corners[6], egde2);
	corners[3] = _mm_add_ps(corners[7], egde2);
	// Transpose into SoA
	_MM_TRANSPOSE4_PS(corners[0], corners[1], corners[2], corners[3]);
	_MM_TRANSPOSE4_PS(corners[4], corners[5], corners[6], corners[7]);



	bool requireClip = false;
    // Even if all bounding box corners have W > 0 here, we may end up with some vertices with W < 0 to due floating point differences; so test with some epsilon if any W < 0.
	if (bQueryOccluder)
	{
			//issue: if the model size is large, the previous epsilon would be unnecessary large, which makes the clipping is often true...
			//////__m128 cornerMaxZ = _mm_max_ps(corners[3], corners[7]);
			//////cornerMaxZ = _mm_max4_ps_soc(cornerMaxZ);
			//////__m128 nearPlaneEpsilon = _mm_mul_ps_scalar_soc(cornerMaxZ, 0.001f);

			//////__m128 closeToNearPlane = _mm_cmplt_ps(_mm_min_ps(corners[3], corners[7]), nearPlaneEpsilon);
			////////check occluder.cpp bakePureTriangle related code, it would be sure that points are inside 
			//////the original AABB
			__m128 minW = _mm_min_ps(corners[3], corners[7]);

			__m128 closeToNearPlane = _mm_cmplt_ps(minW, _mm_set1_ps(mNearPlane));
			requireClip = !_mm_same_sign0(closeToNearPlane);

			//clipping is for rasterization use only
			occ->NeedsClipping = requireClip; 


			//VERIFICATION with previous approach
			////__m128 maxExtent = _mm_max_ps(extents, _mm_shuffle_ps(extents, extents, _MM_SHUFFLE(1, 0, 3, 2)));
			////maxExtent = _mm_max_ps(maxExtent, _mm_shuffle_ps(maxExtent, maxExtent, _MM_SHUFFLE(2, 3, 0, 1)));
			////nearPlaneEpsilon = _mm_mul_ps_scalar_soc(maxExtent, 0.001f);
			//////__m128 closeToNearPlane = _mm_or_ps(_mm_cmplt_ps(corners[3], nearPlaneEpsilon), _mm_cmplt_ps(corners[7], nearPlaneEpsilon));
			////closeToNearPlane = _mm_cmplt_ps(_mm_min_ps(corners[3], corners[7]), nearPlaneEpsilon);
			////bool requireClip2 = !_mm_same_sign0(closeToNearPlane);
			////if (requireClip != requireClip2) 
			////{
			////	LOGI("CLIP DIFF  %d  vs old %d", (int)requireClip, (int)requireClip2);
			////}
	}
	else 
	{
		if (OCCLUDEE_NEARCLIP_BBOX_CHECK_IGNORE_OPTIMIZATION) //it might not needed
		{

			__m128 closeToNearPlane = _mm_cmplt_ps(_mm_min_ps(corners[3], corners[7]), _mm_set1_ps(mNearPlane));
			requireClip = !_mm_same_sign0(closeToNearPlane);
		}
		else 
		{
			//the calculation is for occludees only, occluders' nearPlaneEpsilon is pre-calculated
			__m128 maxExtent = _mm_max4_ps_soc(extents);
			__m128 nearPlaneEpsilon = _mm_mul_ps_scalar_soc(maxExtent, 0.001f);
			//__m128 closeToNearPlane = _mm_or_ps(_mm_cmplt_ps(corners[3], nearPlaneEpsilon), _mm_cmplt_ps(corners[7], nearPlaneEpsilon));
			__m128 closeToNearPlane = _mm_cmplt_ps(_mm_min_ps(corners[3], corners[7]), nearPlaneEpsilon);
			requireClip = !_mm_same_sign0(closeToNearPlane);
		}


		if (requireClip && Rasterize_ClippedOccludee_AS_OCCLUDER_AABB) {
			__m128 boundsMin = _mm_setr_ps(minmaxf[0], minmaxf[1], minmaxf[2], 1.0f);
			__m128 boundsMax = _mm_setr_ps(minmaxf[3], minmaxf[4], minmaxf[5], 1.0f);

			__m128* FrustumPlane = localToClip + 4;
			__m128 min_max_sum = _mm_add_ps(boundsMax, boundsMin);
			if (!inFrustum(min_max_sum, extents, FrustumPlane))
			{
				if (DebugOccluderOccludee)
				{
					if (bQueryOccluder == false)
					{
						this->DebugData[OccludeeFrustumCull]++;
						this->DebugData[OccludeeCull]++;
					}
					else {
						this->DebugData[OccluderCulled]++;
						this->DebugData[OccluderFrustumCulled]++;
					}
				}
				return false;
			}

			if (minmaxf[0] <= mCameraPos[0] && minmaxf[3] >= mCameraPos[0]
				&& minmaxf[1] <= mCameraPos[1] && minmaxf[4] >= mCameraPos[1]
				&& minmaxf[2] <= mCameraPos[2] && minmaxf[5] >= mCameraPos[2]) {
				return true;
			}
			else {
				return this->rasterizeOccludee(corners);
			}
		}
	}


	bool visible = requireClip;
	if (requireClip == false)  //visibility still unknown
	{
		
		// Perspective division
		corners[3] = _mm_rcp_ps(corners[3]);
		corners[0] = _mm_mul_ps(corners[0], corners[3]);
		corners[1] = _mm_mul_ps(corners[1], corners[3]);
		corners[2] = _mm_mul_ps(corners[2], corners[3]);

		corners[7] = _mm_rcp_ps(corners[7]);
		corners[4] = _mm_mul_ps(corners[4], corners[7]);
		corners[5] = _mm_mul_ps(corners[5], corners[7]);
		corners[6] = _mm_mul_ps(corners[6], corners[7]);

		// Vertical mins and maxes
		__m128 minsX = _mm_min_ps(corners[0], corners[4]);
		__m128 maxsX = _mm_max_ps(corners[0], corners[4]);

		__m128 minsY = _mm_min_ps(corners[1], corners[5]);
		__m128 maxsY = _mm_max_ps(corners[1], corners[5]);
		__m128i	minsXY = _mm_cvttps_epi32(_mm_min_ps(_mm_unpacklo_ps(minsX, minsY), _mm_unpackhi_ps(minsX, minsY)));
		__m128i	maxsXY = _mm_cvttps_epi32(_mm_max_ps(_mm_unpacklo_ps(maxsX, maxsY), _mm_unpackhi_ps(maxsX, maxsY)));


		//stress test. put in inf number to check whether crash
		//minsX = _mm_castsi128_ps(_mm_set1_epi32(0x7f800000));
		//maxsX = _mm_castsi128_ps(_mm_set1_epi32(0x7f800000));
		//minsY = _mm_castsi128_ps(_mm_set1_epi32(0x7f800000));
		//maxsY = _mm_castsi128_ps(_mm_set1_epi32(0x7f800000));

		
		// Clamp bounds
		
		if (bQueryOccluder == false)
		{
			__m128i minThreshold = _mm_set1_epi32(0);
			
			minsXY = _mm_max_epi32(minsXY, minThreshold);
				//already frustum culled. limit the max of minXY
			maxsXY = _mm_min_epi32(maxsXY, m_MaxCoordOccludee_WHWH);
		}
		else 
		{
			//add safe force clamp to allow safe traversal of blocks
			minsXY = _mm_max_epi32(minsXY, m_MinCoord_WHWHOccluder);
			maxsXY = _mm_min_epi32(maxsXY, m_MaxCoord_WHWHOccluder);
		}


		// Horizontal reduction, step 2
		__m128i minXYMaxXYLo = _mm_unpacklo_epi32(minsXY, maxsXY);
		__m128i minXYMaxXYHi = _mm_unpackhi_epi32(minsXY, maxsXY);

		__m128i boundsI[2];
		boundsI[0] = _mm_min_epi32(minXYMaxXYLo, minXYMaxXYHi);
		boundsI[1] = _mm_max_epi32(minXYMaxXYLo, minXYMaxXYHi);


		int *bounds = (int*)boundsI;

		//frustum culling
		//very useful for interleave mode occluder
		{
			if (bounds[0] > bounds[5] || bounds[2] > bounds[7]) 
			{
				if (DebugOccluderOccludee)
				{
					this->DebugData[OccluderCulled] += bQueryOccluder;
				}
				return false;
			}
		}


		{
			uint32_t minX = bounds[0];
			uint32_t maxX = bounds[5];
			uint32_t minY = bounds[2];
			uint32_t maxY = bounds[7];


			__m128i depth = packQueryDepth(_mm_max_ps(corners[2], corners[6]));
			uint16_t maxZ = _mm_max_epu16_even(depth);

			if (bQueryOccluder == false) //for Occludee
			{
				if (ShowOccludeeInDepthMap && multiThreadQuery == false)
				{
					mCurrentOccludee = ((uint64_t)minX << 48) | ((uint64_t)maxX << 32) | ((uint64_t)minY << 16) | (uint64_t)maxY;

					mCurrentOccludeeDepth = maxZ;
				}

				{			
					visible = query2D<false, bOccludeeWidth1024>(minX, maxX, minY, maxY, maxZ);
					
				}
			}
			else //for occluder
			{
				visible = query2D<bQueryOccluder, bOccludeeWidth1024>(minX, maxX, minY, maxY, maxZ);
			}
		}

		if (bQueryOccluder)
		{
			if (visible)
			{
				occ->prepareCache(egde0, egde1, egde2);
			}
		}
	}
	else
	{

		if (bQueryOccluder == false)
		{
			__m128 depthJoint = _mm_and_ps(corners[3], corners[7]);
			if (_mm_same_sign1_soc(depthJoint))
			{
				if (false)				//test code: to verify that if behind the camera, in frustum must be false
				if (bFrustumCullIfClip) 
				{
					__m128 boundsMin = _mm_setr_ps(minmaxf[0], minmaxf[1], minmaxf[2], 1.0f);
					__m128 boundsMax;

					if (bQueryOccluder)
					{
						boundsMax = _mm_add_ps(boundsMin, extents);
					}
					else
					{
						boundsMax = _mm_setr_ps(minmaxf[3], minmaxf[4], minmaxf[5], 1.0f);
					}

					__m128* FrustumPlane = localToClip + 4;
					auto min_max_sum = _mm_add_ps(boundsMax, boundsMin);
					assert(inFrustum(min_max_sum, extents, FrustumPlane) == false);
				}
				return false;
			}
		}

		if (bFrustumCullIfClip )
		{
			__m128 boundsMin = _mm_setr_ps(minmaxf[0], minmaxf[1], minmaxf[2], 1.0f);
			__m128 boundsMax;

			if (bQueryOccluder)
			{
				boundsMax = _mm_add_ps(boundsMin, extents);
			}
			else
			{
				boundsMax = _mm_setr_ps(minmaxf[3], minmaxf[4], minmaxf[5], 1.0f);
			}

			__m128* FrustumPlane = localToClip + 4;
			__m128 min_max_sum = _mm_add_ps(boundsMax, boundsMin);
			if (!inFrustum(min_max_sum, extents, FrustumPlane))
			{
				if (DebugOccluderOccludee)
				{
					if (bQueryOccluder == false)
					{
						this->DebugData[OccludeeFrustumCull]++;
						this->DebugData[OccludeeCull]++;
					}
					else {
						this->DebugData[OccluderCulled]++; 
						this->DebugData[OccluderFrustumCulled]++; 
					}
				}
				return false;
			}
		}

		if (bQueryOccluder) {
			occ->prepareCache(egde0, egde1, egde2);
		}

		if (DebugOccluderOccludee)
		{
			if (bQueryOccluder) {
				this->DebugData[OccludeeNearClipPass]++;
			}
		}
	}

	if(DebugOccluderOccludee)
	{
		this->DebugData[OccluderCulled] += bQueryOccluder && (visible == false);
	}
	//refactor previous multiple exit to this single exit
	return visible;
}

static inline bool containPattern10(uint64_t left, uint64_t right)
{
	return left > (left & right);
	//return ((left ^ right) & left) > 0;
}
//idx = 0 result = 7
//idx = 1 result = 0
static inline int mask70(int idx) 
{
	return (7 + idx) & 7;
}
//bQueryOccluder means query occluder
template <bool bQueryOccluder, bool bOccludeeWidth1024>
bool Rasterizer::query2D(uint32_t pixelMinX, uint32_t pixelMaxX, uint32_t pixelMinY, uint32_t pixelMaxY, uint16_t maxZ)
{		
	if (bQueryOccluder == false && bTestObbByAABB) {
		static int testlog = 0;
		testlog++;
		testlog = testlog & 1;
		//std::cout << testlog << " minx " << pixelMinX << " maxX " << pixelMaxX << " minY " << pixelMinY << " maxY " << pixelMaxY << " maxZ " << maxZ << std::endl;
	}

	if (DebugOccluderOccludee && bQueryOccluder == false) 
	{
		DebugData[MaxOccludeeZ] = std::max<uint16_t>(maxZ, DebugData[MaxOccludeeZ]);
		DebugData[OccludeeQuery2d]++;
	}

	// X / 8 == X >> 3
	int blockMinX = pixelMinX >> 3;
	int blockMaxX = pixelMaxX >> 3;

	int blockMaxY = pixelMaxY >> 3;
	int blockMinY = pixelMinY >> 3;


	if (bQueryOccluder == false && bOccludeeBitScanOp) 
	{
        //return true;
		if (maxZ < bOccludeeMinDepthThreshold && bOccludeeWidth1024) {
			//1. occludee any block scan				
			uint64_t all = -1;
			int DualBlockMinX = blockMinX >> 1;
			int DualBlockMaxX = blockMaxX >> 1;
            uint64_t rowMask = (all << DualBlockMinX) & (all >> (63 ^ DualBlockMaxX));
          

			int y = blockMaxY;
            uint64_t mask = -1;
			do {
                mask &= mAnyDataBlockMask[y];
				y--;
			} while (y >= blockMinY);

            
            if (rowMask == (rowMask & mask))
            {
                if (DebugOccluderOccludee && !bQueryOccluder)
                {
                    DebugData[FastRowBitCull]++;
                }

                return false;
            }

		}
	}

	//usage of hiz Max to accelerate pass check
	
	int topRow = blockMaxY * m_blocksX;
	if (m_pHizMax[topRow + blockMinX] <= maxZ || m_pHizMax[topRow + blockMaxX] <= maxZ) //top left
	{
		if (DebugOccluderOccludee)
		{
			DebugData[OccludeeQueryMaxPass] += !bQueryOccluder;
			DebugData[OccluderQueryMaxPass] += bQueryOccluder;
		}
		return true;
	}




	uint16_t *pHiZBuffer = m_pHiz;
	uint64_t *pDepthBuffer = m_pDepthBuffer;

	uint32_t pHizOffset = (blockMaxY  * m_blocksX + blockMinX);//top left
	//int blockRangeX = blockMaxX - blockMinX;
	
	
	{
		int startBlockY = -1;
		if (DebugOccluderOccludee) {
			DebugData[QueryBlockDoWhileIfSave]++;
		}
		int blockY = blockMaxY;
		do
		{
			uint16_t *pHiZ = pHiZBuffer + pHizOffset;
			if (DebugOccluderOccludee) {
				DebugData[QueryBlockDoWhileIfSave]++;
			}
			int blockX = blockMinX;
			do
			{
				if (DebugOccluderOccludee && !bQueryOccluder)
				{
					DebugData[FastBlockDepthCompare]++;
				}
				if (maxZ >= pHiZ[0])
				{
					if (pHiZ[0] == 0)
					{
						if (DebugOccluderOccludee && !bQueryOccluder)
						{
							DebugData[FastBlockEmptyPass]++;
						}

						return true;
					}
					startBlockY = blockY;
					blockY = -1;
					break;
				}
				++blockX;
				++pHiZ;
			} while (blockX <= blockMaxX);
			--blockY;
			pHizOffset -= m_blocksX;
		} while (blockY >= blockMinY);

		if (startBlockY == -1)
		{
			if (DebugOccluderOccludee && !bQueryOccluder)
			{
				DebugData[FastBlockHizCull]++;
				DebugData[OccludeeCull]++;
			}
			return false;
		}
		if (startBlockY != blockMaxY) 
		{
			//make pixel Y end map always 7, means fully covered the block
			pixelMaxY = 7;
			blockMaxY = startBlockY;
		}
		pHizOffset += m_blocksX;
	}
    
    if (m_pHizMax[topRow + ((blockMaxX+blockMinX)>>1)] <= maxZ) //top left
    {
        if (DebugOccluderOccludee)
        {
            DebugData[OccludeeQueryMaxPass] += !bQueryOccluder;
            DebugData[OccluderQueryMaxPass] += bQueryOccluder;
        }
        return true;
    }

	//Extend the width in case that only one pixel occludee
	//this should not affect performance much, as
	//1. branchless code
	//2. most cases are already determined
	if (PairBlockNum <= CheckerBoardVizMaskApproach)
	{
		if (DebugOccluderOccludee && !bQueryOccluder)
		{
			DebugData[OccludeeOnePixelExpandCheck]++;
		}

		//branchless approach
		int OnePixelOccludee = (pixelMinX == pixelMaxX) && (pixelMinY == pixelMaxY);
		//case 0: OnePixelOccludee == 0, no impact to pixelMinX, pixelMaxX
		//case 1: 
		// the one pixel last bit is 0:  expand pixelMaxX
		// the one pixel last bit is 1:  reduce pixelMinX
		pixelMaxX |= OnePixelOccludee;
		int lastBit = pixelMinX & 1;
		pixelMinX ^= lastBit & OnePixelOccludee;
		//branch approach
		//if (pixelMinX == pixelMaxX && pixelMinY == pixelMaxY)
		//{
		//	if ((pixelMinX & 7) == 0)
		//	{
		//		pixelMaxX++;
		//	}
		//	else 
		//	{
		//		pixelMinX--;
		//	}
		//}
	}

	//up to here, pixelMinY pixelMaxY pixelMaxX pixelMinX are only used to calculate per block's startY endY startX endX
	pixelMaxY &= 7;
	pixelMaxX &= 7;
	if (DebugOccluderOccludee) {
		DebugData[QueryBlockDoWhileIfSave]++;
	}
	int blockY = blockMaxY;
	do
	{
		uint16_t *pHiZ = pHiZBuffer + (pHizOffset);   //optimize pHizOffset = (blockY * m_blocksX + blockMinX);
        uint64_t *pBlockDepth = pDepthBuffer + (pHizOffset * PairBlockNum);

		//uint16_t startY = 0;
		//if (blockY == blockMinY) startY = pixelMinY & 7;
		//uint16_t startY = (pixelMinY & 7) >> ((int)(blockY != blockMinY) << 2); //branchless version
		uint16_t startY = pixelMinY & mask70((int)(blockY != blockMinY));
		uint16_t endY = (pixelMaxY | mask70((int)blockY == blockMaxY)) ;

		//bool interiorLine = (startY == 0) && (endY == 7);

		for (int blockX = blockMinX; blockX <= blockMaxX; ++blockX, ++pHiZ,
			 pBlockDepth += PairBlockNum)
		{
			uint16_t hiz = pHiZ[0]; 
			// Skip this block if it fully occludes the query box
			if (maxZ < hiz)
			{
				continue;
			}

			if (hiz == 0) {
				if (DebugOccluderOccludee && !bQueryOccluder)
				{
					DebugData[BlockEmptyPass] += hiz == 0;
				}
				return true;
			}



			//uint16_t startX = 0;
			//if (blockX == blockMinX) startX = pixelMinX & 7;
			//uint16_t startX = (pixelMinX & 7) >> ((int)(blockX != blockMinX) << 2); //branchless version
			uint16_t startX = pixelMinX & mask70((int)(blockX != blockMinX));

			//uint16_t endX = 7;
			//if (blockX == blockMaxX) endX = pixelMaxX & 7;
			uint16_t endX = (pixelMaxX | mask70((int)blockX == blockMaxX)) ;
			
			//due to usage of conserve minz calculation. this check is not valid anymore
			////{		
			////	if (DebugOccluderOccludee && !bQueryOccluder)
			////	{
			////		DebugData[BlockCorrectMinPass]++;
			////	}
			////	if (interiorLine)
			////	{
			////		bool interiorBlock = (startX == 0) && (endX == 7);
			////		// No pixels are masked, so there exists one where maxZ > pixelZ, and the query region is visible
			////		if (interiorBlock)
			////		{			
			////			return true;
			////		}
			////	}
			////}
			
			if (PairBlockNum <= PureCheckerBoardApproach+1)
			{
				if (hiz <= MIN_UPDATED_BLOCK_DEPTH2 && PairBlockNum != PureCheckerBoardApproach) //partial updated block
				{
					uint64_t check = mPixelBound->PixelMinXMask[startX] & 
						mPixelBound->PixelMaxXMask[endX] &
						mPixelBound->PixelMinYMask[startY] &
						mPixelBound->PixelMaxYMask[endY];
					uint64_t * maskData = GetMaskData(pBlockDepth, blockX & 1);
					if (containPattern10(check, maskData[0]))
					{
						if (DebugOccluderOccludee && !bQueryOccluder)
						{
							DebugData[BlockMaskPass]++;
						}

						return true;
					}
				}


				__m128i *startBlockDepth;
				if (PairBlockNum != PureCheckerBoardApproach)
					startBlockDepth = GetDepthData(pBlockDepth, (blockX & 1));
				else
					startBlockDepth = (__m128i *)pBlockDepth;

				int rowSelector = (0xFF << startX) & (0xFF >> (7 ^ endX));
				__m128i maxZV = _mm_set1_epi16(maxZ);

				{
					uint8_t* checkerBoardMask = (uint8_t*)(mCheckerBoardQueryMask + (mCheckerBoardQueryOffset[startY] + endY - startY));
					//*******************************************************************


					uint16_t realStartY = startY >> 1;
					uint16_t realEndY = endY >> 1;

					
					if (DebugOccluderOccludee) {
						DebugData[QueryBlockDoWhileIfSave]++;
					}
					uint16_t y = realStartY;
					do
					{
						if (DebugOccluderOccludee && !bQueryOccluder)
							DebugData[BlockRowCheck]++;
						auto dy = startBlockDepth[y];
						__m128i visible = _mm_cmple_epu16_soc(dy, maxZV);
						int visiblePixelMask = _mm_movemask_epi16_soc(visible);

						int currentRowSelector = rowSelector;
						if (currentRowSelector & visiblePixelMask & checkerBoardMask[y])
						{
							//LOGI("query2D : %d, %d, %d", currentRowSelector, visiblePixelMask, checkerBoardMask[y]);
							if (DebugOccluderOccludee && !bQueryOccluder)
							{
								DebugData[BlockPixelPass]++;
							}
							return true;
						}
						++y;
					}while(y <= realEndY);
				}
				
				
			}
			else {
				__m128i maxZV = _mm_set1_epi16(maxZ);
				//int rowSelector = (0xFF << startX) & (0xFF >> (7 ^ endX)); //endX falls in[0,7] 7^endX = 7-endX
				int rowSelector = (0xFF << startX) & (0xFF >> (7 ^ endX));

				__m128i *startBlockDepth = (__m128i *) pBlockDepth;
				for (uint16_t y = startY; y <= endY; ++y)
				{
					if (DebugOccluderOccludee && !bQueryOccluder)
						DebugData[BlockRowCheck]++;
					__m128i visible = _mm_cmplt_epu16_soc(startBlockDepth[y], maxZV);
					int visiblePixelMask = _mm_movemask_epi16_soc(visible);
					if (rowSelector & visiblePixelMask)
					{
						if (DebugOccluderOccludee && !bQueryOccluder)
						{
							DebugData[BlockPixelPass]++;
						}
						return true;
					}
				}
			}
		}
		--blockY;
		pHizOffset -= m_blocksX;
	}
	while (blockY >= blockMinY);

	if (DebugOccluderOccludee && !bQueryOccluder) 
	{
		DebugData[BlockPixelCull]++;
		DebugData[OccludeeCull]++;
	}
	// Not visible
	return false;
}
static __m128i CompareWithExtraBuffer(__m128i blockData, __m128i  extraRoot)
{
	return _mm_or_si128(blockData, extraRoot);
	//__m128i valid = _mm_cmplt_epu16_soc(_mm_setzero_si128(), extraRoot);
	//__m128i match = _mm_cmple_epu16_soc(blockData, extraRoot);
	//match = _mm_and_si128(valid, match);


	//__m128i gray = _mm_srli_epi16(blockData, 1);
	//__m128i gray2 = _mm_srli_epi16(blockData, 2);
	//gray = _mm_adds_epu8(gray, gray2);

	//return _mm_or_si128(gray, match);
}

#if defined( SUPPORT_ALL_FEATURE)
static void SaveCheckBoard(uint64_t mask, uint8_t * target) 
{
	//64 x 64
	for (int y = 0; y < 256; y++) 
	{
		for (int x = 0; x < 256; x++)
		{
			int yu = y >> 5;
			int xu = x >> 5;

			uint32_t bitIndex = 8 * yu +  (7-xu);
			int bit = (mask >> bitIndex) & 1;
			if (bit)
			{
				target[y * 256 + x] = 255;
			}
			else {
				target[y * 256 + x] = 0;
			}
		}
	}
}
#endif


float decompressFloat(uint16_t depth)
{
	const float bias = 3.9623753e+28f; // 1.0f / floatCompressionBias

	union {
		uint32_t u;
		float f;
	} U = { uint32_t(depth) << 12 };
	return  (U.f * bias);
}
uint64_t Rasterizer::applyToneMapping(__m128i a, common::DumpImageMode mode)
{

	if (mode == DumpFullMin)
		return _mm_getUint16Max8_soc(_mm_slli_epi32(_mm_and_si128(a, _mm_set1_epi16(255)), 8));  	//return _mm_getUint16Min8_soc(a);
	if (mode == DumpFullMax)
		return _mm_getUint16Max8_soc(a);

	//project [maxDepth, minDepth] to [55, 255]. Developer can tone the minDepth/maxDepth
	const float minDepth = 1.0f;
	const float maxDepth = 1000.0f;

	uint64_t result = 0;
	__m128i data[2];
	data[0] = _mm_and_si128(a, _mm_set1_epi32(65535));
	data[1] = _mm_srli_epi32(a, 16);
	__m128 toneMin128 = _mm_set1_ps(minDepth);
	__m128 toneScale = _mm_set1_ps(200.0f / (maxDepth - minDepth));
	for (int idx = 0; idx < 2; idx++) {
		__m128 inputf = _mm_castsi128_ps(_mm_slli_epi32(data[idx], 12));
		__m128 mask = _mm_cmpgt_ps(inputf, _mm_set1_ps(0.0f));
		__m128 scale = _mm_set1_ps(3.9623753e+28f);
		inputf = _mm_mul_ps(inputf, scale);
		inputf = _mm_rcp_ps(inputf);
		inputf = _mm_min_ps(_mm_max_ps(inputf, toneMin128), _mm_set1_ps(maxDepth));
		inputf = _mm_sub_ps(inputf, toneMin128);
		inputf = _mm_mul_ps(inputf, toneScale);
		inputf = _mm_sub_ps(_mm_set1_ps(255.0f), inputf);
		inputf = _mm_and_ps(inputf, mask);

		float* fp1 = (float*)&inputf;
		uint64_t output = 0;
		output |= ((uint64_t)fp1[0]);
		output |= ((uint64_t)fp1[1]) << (2 * 8);
		output |= ((uint64_t)fp1[2]) << (4 * 8);
		output |= ((uint64_t)fp1[3]) << (6 * 8);
		result |= output << (idx << 3);
	}
	return result;
}
bool Rasterizer::readBackDepth(unsigned char *target, common::DumpImageMode mode)
{

#if defined( SUPPORT_ALL_FEATURE)
	static constexpr uint64_t blackPattern = 0x55AA55AA55AA55AA;
	static constexpr uint64_t whitePattern = 0xAA55AA55AA55AA55;
	static constexpr uint64_t oddColumn = 0xAAAAAAAAAAAAAAAA;
	static constexpr uint64_t evenColumn = 0x5555555555555555;

	static constexpr uint64_t oddBlack = (oddColumn & blackPattern);
	static constexpr uint64_t evenBlack = (evenColumn & blackPattern);
	static constexpr uint64_t evenWhite = (evenColumn & whitePattern);
	static constexpr uint64_t oddWhite = (oddColumn & whitePattern);
	if (mode == CheckerBoard_blackPattern)
	{
		SaveCheckBoard(blackPattern, target);
		return true;
	}
	if (mode == CheckerBoard_whitePattern)
	{
		SaveCheckBoard(whitePattern, target);
		return true;
	}
	if (mode == CheckerBoard_oddColumn)
	{
		SaveCheckBoard(oddColumn, target);
		return true;
	}
	if (mode == CheckerBoard_evenColumn)
	{
		SaveCheckBoard(evenColumn, target);
		return true;
	}
	if (mode == CheckerBoard_oddBlack)
	{
		SaveCheckBoard(oddBlack, target);
		return true;
	}
	if (mode == CheckerBoard_evenBlack)
	{
		SaveCheckBoard(evenBlack, target);
		return true;
	}

	if (mode == CheckerBoard_evenWhite)
	{
		SaveCheckBoard(evenWhite, target);
		return true;
	}

	if (mode == CheckerBoard_oddWhite)
	{
		SaveCheckBoard(oddWhite, target);
		return true;
	}

	if (mode == DumpBlockMask)
	{
		memset(target, 0, sizeof(unsigned char) * 512 * 1024);
		int numofMinusOne = 0;
		uint64_t minusOne = -1;

		int zeroCount = 0;
		for (int y = 0; y < 64; y++)
		{
			for (int x = 0; x < 64; x++)
			{
				uint64_t mask = 0;
				mask = m_precomputedRasterTables[y * 64 + x];

				if (mask == minusOne) 
				{
					numofMinusOne++;
				}
				if (mask == 0) {
					zeroCount++;
				}
				int blockMin = x * 8 + y * 16 * 512;
				for (int by = 0; by < 8; by++)
				{
					int pixelIdx = blockMin + (by * 512);
					unsigned char *dest = target + pixelIdx;
					for (int bx = 0; bx < 8; bx++)
					{
						int maskIdx = (8 * bx + 7 - by);
						*dest = ((mask >> maskIdx) & 1) << 7;
						dest++;
					}
				}
			}
		}
		//it has been found that for case x <= 1, the mask value is -1. Fully covered
		return true;
	}
#endif


#if defined(SDOC_NATIVE_DEBUG)
	
	if (mode == common::DumpImageMode::DumpHiz) 
	{
		uint16_t * hzPointer = this->m_pHiz;

		
		uint8_t *targetImage = (uint8_t *)target;
		for (uint32_t blockY = 0; blockY < m_blocksY; ++blockY)
		{
			for (uint32_t blockX = 0; blockX < m_blocksX; ++blockX)
			{
				int index = blockY * m_blocksX + blockX;
				uint16_t zValue = hzPointer[index];
				targetImage[index] = zValue >> 8;
			}
		}
		return true;
	}
#endif
	
	if (DebugOccluderOccludee) 
	{
		LOGI("blockY %d blockX %d  m_width %d Height %d m_blockSize %d interleave %d m_HizBufferSize %d OccNum %d", m_blocksY , m_blocksX , m_width , m_height ,  m_blockSize , mInterleave.CurrentFrameInterleaveDrawing , m_HizBufferSize , mCurrValidOccluderNum );
	}


	//bool quickDumpForWindows = true;
	//if (common::IS_ARM_PLATFORM || quickDumpForWindows)
	{
		if (mCurrValidOccluderNum == 0) {
			memset(target, 0, m_totalPixels * sizeof(uint8_t));
		}
		else if (this->mDebugRenderType >= RenderType::Line)
		{
			//copy point line data
			if (m_depthBufferPointLines.size() == m_totalPixels && this->mInterleave.CurrentFrameInterleaveDrawing == false)
			{
				uint64_t * input = (uint64_t*)target;
				__m128i *extraRoot = (__m128i*) &this->m_depthBufferPointLines[0];
				int totalBlocks = (int)( this->m_totalPixels >> 3);
				for (int idx = 0; idx < totalBlocks; idx++) 
				{
					input[idx] = applyToneMapping(extraRoot[idx], mode);
				}
			}
		}
		else
		{
			bool mergeBuffer = mDebugRenderType == RenderType::MeshLine || mDebugRenderType == RenderType::MeshPoint;
			__m128i  TargetBlockData[8];	

			uint64_t * targetRoot = (uint64_t*)target;
			__m128i * extraRoot = nullptr;
			if (mergeBuffer) 
			{
				extraRoot = (__m128i*) &this->m_depthBufferPointLines[0];
			}
			for (uint32_t blockY = 0; blockY < m_blocksY; ++blockY)
			{
				int base = blockY * m_blocksX;
				for (uint32_t blockX = 0; blockX < m_blocksX; ++blockX)
				{
					uint8_t *dest = (uint8_t *)target + (8 * blockX + m_width * (8 * blockY));
					int hizIdx = blockX + base;
					uint64_t * blockPtr =( m_pDepthBuffer + PairBlockNum * hizIdx);
					uint16_t hiz = this->m_pHiz[hizIdx];
					if (hiz == 0)
					{
						for (uint32_t y = 0; y < 8; ++y, dest += m_width)
						{
							uint64_t * dest64 = reinterpret_cast<uint64_t*>(dest);
							if (mergeBuffer)
							{
								int count = (int)(dest64 - targetRoot);

								__m128i valid = _mm_cmplt_epu16_soc(_mm_setzero_si128(), extraRoot[count]);
								*dest64 = applyToneMapping(valid, mode);
							}
							else {
								*dest64 = 0;
							}
						}
						continue;
					}
					else if (PairBlockNum == FullBlockApproach)
					{
						__m128i* fullBlock = (__m128i* )blockPtr;
						for (uint32_t y = 0; y < 8; ++y, dest += m_width)
						{
							uint64_t * dest64 = reinterpret_cast<uint64_t*>(dest);
							__m128i source128 = fullBlock[y];
							*dest64 = applyToneMapping(source128, mode);
						}
						continue;
					}

					if (PairBlockNum <= PureCheckerBoardApproach+1)
					{
						if (hiz <= MIN_UPDATED_BLOCK_DEPTH2)
						{
							uint64_t* maskData = GetMaskData(blockPtr, blockX & 1);
							__m128i* depthData = GetDepthData(blockPtr, (blockX & 1));
							
							RecoverPartialBlockData(TargetBlockData, depthData, maskData);
							__m128i*  fullBlock = TargetBlockData;
							for (uint32_t y = 0; y < 8; ++y, dest += m_width)
							{
								uint64_t * dest64 = reinterpret_cast<uint64_t*>(dest);


								__m128i blockData = fullBlock[y];
								if (mergeBuffer)
								{
									int count = (int)(dest64 - targetRoot);
									blockData = CompareWithExtraBuffer(blockData, extraRoot[count]);
								}
								*dest64 = applyToneMapping(blockData, mode);

							}							
						}
						else
						{
							__m128i* fullBlock = GetDepthData(blockPtr, (blockX & 1));
							
							if (bDumpCheckerboardImage)
							{
								for (int i = 0; i < 8; i++)
								{
									TargetBlockData[i] = fullBlock[i >> 1];
								}
								GrayCheckBoardWhitePixel(TargetBlockData);
								for (uint32_t y = 0; y < 8; ++y, dest += m_width)
								{
									uint64_t * dest64 = reinterpret_cast<uint64_t*>(dest);
									__m128i blockData = TargetBlockData[y];
									if (mergeBuffer)
									{
										int count = (int)(dest64 - targetRoot);
										blockData = CompareWithExtraBuffer(blockData, extraRoot[count]);
									}
									*dest64 = applyToneMapping(blockData, mode);
								}
							}
							else {
								for (uint32_t y = 0; y < 8; ++y, dest += m_width)
								{
									uint64_t * dest64 = reinterpret_cast<uint64_t*>(dest);
									__m128i blockData = fullBlock[y>>1];


									if (mergeBuffer)
									{
										int count = (int)(dest64 - targetRoot);
										blockData = CompareWithExtraBuffer(blockData, extraRoot[count]);
									}
									*dest64 = applyToneMapping(blockData, mode);
								}
							}
											
						}
					}

				}
			}
			
		}

		int totalPixel = m_width * m_height;
		int currentSize = (int)mOccludeeResults.size();
		uint64_t *imgMetaData = reinterpret_cast<uint64_t*>(target + totalPixel);
		if (ShowOccludeeInDepthMap == false || totalPixel / 8 <= mOccludeeResults.size() || currentSize == 0) //too many queries
		{
			imgMetaData[0] = 0;
		}
		else
		{
			imgMetaData[0] = currentSize;
			//sort according to depth value
			std::vector<uint64_t> all;
			mOccludeeResults.reserve(currentSize + (currentSize >> 1));
			for (int idx = 1; idx < currentSize; idx += 2)
			{
				uint64_t merge = mOccludeeResults[idx] | ((uint64_t)idx << 8);
				mOccludeeResults.push_back(merge);
			}
			std::sort(mOccludeeResults.begin()+ currentSize, mOccludeeResults.end());//ascending order
			int metaDataIdx = 1;
			for (int idx = currentSize; idx < mOccludeeResults.size(); idx++)
			{
				int currentIdx = (mOccludeeResults[idx] >> 8) & 0xFFFFFFFF;
				imgMetaData[metaDataIdx++] = mOccludeeResults[currentIdx - 1];
				imgMetaData[metaDataIdx++] = mOccludeeResults[currentIdx];
			}
		}

#if defined(SDOC_WIN)
		//debug data
		if (ShowOccludeeInDepthMap && PrintOccludeeState && mOccludeeResults.size() > 0) 
		{
			std::string developerGoldData = "..\\..\\GOLDEN_DATA\\Output\\";
			mkdir(developerGoldData.c_str());

			std::string file_name = developerGoldData + std::to_string(this->m_width) + "_" + std::to_string(this->m_height) + ".log";
			LOGI("filename %s", file_name.c_str());
			FILE *	mFileWriter = fopen(file_name.c_str(), "w");
			
			for (int idx = 0; idx < currentSize; idx+=2)
			{

				uint64_t value = mOccludeeResults[idx];
				bool visible = mOccludeeResults[idx | 1]  & 1;

				uint16_t minX = value >> 48;
				 
				uint16_t maxX = (value >> 32) & 0xFFFF;
				uint16_t minY = (value >> 16) & 0xFFFF;
				uint16_t maxY = value & 0xFFFF;
				//going to draw this rectangle

				if (PrintOccludeeState) 
				{
					if (PrintOccludeeState) {
						fprintf(mFileWriter, "OccludeeIdx %d minx %d %d %d %d Visible %d depth %d\n ", idx, minX, maxX, minY, maxY, (int)visible, (int)((mOccludeeResults[idx | 1] >> 48)));
					}
				}
				
			}
			fclose(mFileWriter);
		}
#endif
	}
	return true;
}


static void normalizeEdge(__m128 &nx, __m128 &ny, __m128 &invLen)
{
    invLen = _mm_rcp_ps(_mm_add_ps(_mm_abs_ps_soc(nx), _mm_abs_ps_soc(ny)));
    nx = _mm_mul_ps(nx, invLen);
    ny = _mm_mul_ps(ny, invLen);
}
//This function map nx, ny to [0, 63], nx map to [0, 31] << 1, ny map to [0, 1]
//result = nxMap - nyMap


static __m128i quantizeSlopeLookup(__m128 nx, __m128 ny)
{
	__m128i yNeg2 = _mm_castps_si128(ny);
	//auto yNegI2 = _mm_slli_epi32(_mm_srli_epi32(yNeg2, 31), OFFSET_QUANTIZATION_BITS);
	auto yNegI2 = _mm_srli_epi32(yNeg2, 31- OFFSET_QUANTIZATION_BITS);

	if (ARMV7)
	{
		////// Remap [-1, 1] to [0, SLOPE_QUANTIZATION / 2]
		constexpr float mul = (SLOPE_QUANTIZATION_FACTOR / 2 - 1) * 0.5f * 128;  //15.5
		constexpr float add = mul + 0.5f * 128;                                  //16
		__m128i quantizedSlope = _mm_cvttps_epi32(_mm_fmadd_ps_soc(nx, mul, _mm_set1_ps(add)));
		quantizedSlope = _mm_srli_epi32(quantizedSlope, 7);
		quantizedSlope = _mm_slli_epi32(quantizedSlope, 7);
		return _mm_and_si128(_mm_or_si128(quantizedSlope, yNegI2), _mm_set1_epi32(4032));
	}
	else {
		//16.0f / 15.5f = 1.03225806452
		//5.87747175411e-39/5.6938007618e-39  = 1.03225806452
		//2.93873587706e-39/2.8469003809e-39
		__m128i quantizedSlope = _mm_castps_si128(_mm_fmadd_ps_soc(nx, 3.55862547612e-40, _mm_set1_ps(3.67341984632e-40)));
		quantizedSlope = _mm_srli_epi32(quantizedSlope, 14);
		auto q2 = _mm_slli_epi32(quantizedSlope, OFFSET_QUANTIZATION_BITS + 1);
		return _mm_and_si128(_mm_or_si128(q2, yNegI2), _mm_set1_epi32(4032));
	}
}


static void normalizeEdgeAndQuantizeSlope(__m128 &nx, __m128 &ny, __m128 &invLen, __m128i & slope)
{
	invLen = _mm_rcp_ps(_mm_add_ps(_mm_abs_ps_soc(nx), _mm_abs_ps_soc(ny)));
	nx = _mm_mul_ps(nx, invLen);
	ny = _mm_mul_ps(ny, invLen);
	slope = quantizeSlopeLookup(nx, ny);
}

//map [-0.45, 0.45] to [0, 63]
static uint32_t quantizeOffsetLookup(float offset)
{

    float lookup = offset * OFFSET_mul + OFFSET_add;
    return std::min(std::max(int32_t(lookup), 0), OFFSET_QUANTIZATION_FACTOR - 1);
}

void Rasterizer::precomputeRasterizationTable()
{
    static constexpr uint32_t angularResolution = 2000;
	static constexpr uint32_t offsetResolution = 2000;
	
	__m128i * maskTable = nullptr;

	if (this->m_depthBuffer.capacity() >= OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR)
	{
		maskTable =(__m128i *) m_pDepthBuffer;
	}
	else 
	{
		maskTable = new __m128i[OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR];
	}
	uint8_t* offsetLookupTable = nullptr;
	bool offsetLookupTableNewed = false;
	if (this->m_depthBuffer.capacity() >= OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR + offsetResolution / 16 )
	{
		offsetLookupTable = (uint8_t*)(m_pDepthBuffer + OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR);
	}
	else 
	{
		offsetLookupTableNewed = true;
		offsetLookupTable = new uint8_t[offsetResolution];
	}

	if (ReflectBoostBlockMask_Optimization)
		memset(maskTable, 0, OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR/2 * sizeof(__m128i));
	else
		memset(maskTable, 0, OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR * sizeof(__m128i));



	std::chrono::high_resolution_clock::time_point  startTime = std::chrono::high_resolution_clock::now();
	
	//it has been found that 0~261 equals 0
	memset(offsetLookupTable, 0, 262 * sizeof(float));
	for (uint32_t i = 262; i < offsetResolution; ++i)
	{
		float offset = -0.6f + 1.2f * float(i) / (angularResolution - 1);
		offsetLookupTable[i] = quantizeOffsetLookup(offset); //[0, 63]
	}


	float xCache[8];
	float yCache[8];
	__m128i sumJThread[8];
	__m128i sumJThreadByte[8];
	__m128i sumJThreadK8[4];
	//288648 / 17528 = 16.467. current approach is 16 times faster compare with default cpp version

	int totalAngle = angularResolution;
	int fromAngle = 0;
	if (ReflectBoostBlockMask_Optimization) 
	{
		totalAngle = angularResolution / 4 + 21;
		fromAngle = 32;
	}
    for (int angleValue = fromAngle; angleValue <= totalAngle; ++angleValue)
    {
        float angle = -0.1f + 6.4f * float(angleValue) / (angularResolution - 1);

        float nx = std::cos(angle);
        float ny = std::sin(angle);
		float absNxNy = (std::abs(nx) + std::abs(ny));
        float l = 1.0f / absNxNy;

        nx *= l;
        ny *= l;

		auto nxTemp = _mm_set1_ps(nx);
		auto nyTemp = _mm_set1_ps(ny);
        uint32_t slopeLookup = _mm_extract_epi32(quantizeSlopeLookup(nxTemp, nyTemp), 0);
		if (ReflectBoostBlockMask_Optimization) slopeLookup >>= 1;

		if (true) //17481ns 
		{
			auto t0 = _mm_setr_ps((0 - 3.5f) * 0.125f, (1 - 3.5f) * 0.125f, (2 - 3.5f) * 0.125f, (3 - 3.5f) * 0.125f);
			auto t1 = _mm_setr_ps((4 - 3.5f) * 0.125f, (5 - 3.5f) * 0.125f, (6 - 3.5f) * 0.125f, (7 - 3.5f) * 0.125f);
			__m128 xCache128[2];
			__m128 yCache128[2];
			nyTemp = _mm_negate_ps_soc(nyTemp);
			xCache128[0] = _mm_mul_ps(t0, nxTemp);
			xCache128[1] = _mm_mul_ps(t1, nxTemp);
			yCache128[0] = _mm_mul_ps(t0, nyTemp);
			yCache128[1] = _mm_mul_ps(t1, nyTemp);
			_mm_store_ps(xCache, xCache128[0]);
			_mm_store_ps(xCache + 4, xCache128[1]);
			_mm_store_ps(yCache, yCache128[0]);
			_mm_store_ps(yCache + 4, yCache128[1]);
		}
		else  //20068ns
		{
			for (auto j = 0; j < 8; ++j)
			{
				auto t = (j - 3.5f) * 0.125f;
				xCache[j] = t * nx;
				yCache[j] = -t * ny;
			}
		}

		float min = -3.5f * 0.125f * absNxNy;


		//here calculate max offset resolution
		//min > 0.6f - 1.2f * float(i) / (angularResolution - 1);   //ignore
		//i > (0.6 - min) * (angularResolution - 1) / 1.2f; //ignore
		int maxj =(int) ceil((0.6 - min) * (angularResolution - 1) / 1.2f);
		maxj = std::min<int>(maxj, offsetResolution);

		auto y0 = _mm_load_ps(yCache);
		auto y4 = _mm_load_ps(yCache + 4);


		//	//i > (0.6 - sum) * (angularResolution - 1) / 1.2f; //ignore
		//	int maxj = ceil((600 - 1000 *min) * 1.999 / 1.2f);
		//maxj  map to [0, angularResolution + PositiveOffset]
		static constexpr int32_t PositiveOffset = 1;
		auto PositiveOffsetV = _mm_set1_ps(PositiveOffset);
		auto ps06 = _mm_set1_ps(0.6f);
		auto scale = _mm_set1_ps((angularResolution - 1) / 1.2f);
		for (auto j = 0; j < 8; ++j)
		{
			auto xm = _mm_set1_ps(xCache[j]);
			auto sum1 = _mm_add_ps(y0, xm);
			sum1 = _mm_sub_ps(ps06, sum1);
			sum1 = _mm_mul_ps(sum1, scale);

			sum1 = _mm_add_ps(sum1, PositiveOffsetV);
			sum1 = _mm_max_ps(sum1, _mm_set1_ps(0));
			sum1 = _mm_min_ps(sum1, _mm_set1_ps(angularResolution + PositiveOffset));

			__m128i sumInThread1 = _mm_cvttps_epi32(sum1); //this is floor operation


			auto sum2 = _mm_add_ps(y4, xm);
			sum2 = _mm_sub_ps(ps06, sum2);
			sum2 = _mm_mul_ps(sum2, scale);

			sum2 = _mm_add_ps(sum2, PositiveOffsetV);
			sum2 = _mm_max_ps(sum2, _mm_set1_ps(0));
			sum2 = _mm_min_ps(sum2, _mm_set1_ps(angularResolution + PositiveOffset));
			__m128i sumInThread2 = _mm_cvttps_epi32(sum2);  //floor operation

			sumInThread2 = _mm_slli_epi32(sumInThread2, 16);
			sumJThread[j] = _mm_or_si128(sumInThread2, sumInThread1); 
		}

		//maxj = 0;
		int minj = 0;
		if (ReflectBoostBlockMask_Optimization) minj = 256;//hack to eliminate calculation
		
		if (bMaskTableOffsetOptimization) 
		{
			static constexpr float th = 0.45f;
			static constexpr int minj2 =(int)( (-th + 0.6) / 1.2f * (angularResolution - 1) - 2);
			static constexpr int maxj2 = (int)((th + 0.6) / 1.2f * (angularResolution - 1) + 2);
			maxj = std::min<int>(maxj, maxj2);
			minj = std::max<int>(minj, minj2);
		}

        for (int32_t j = minj; j < maxj; ++j)
        {
			int j127 = j & 127; //map j to [0~127]
			if (j127 == 0) 
			{
				int delta =  j - (j&127);
				__m128i deltaV = _mm_set1_epi16(delta);
				//map sumJThread to [0, 200];
				auto Byte128V = _mm_set1_epi16(128);
				for (int k = 0; k < 8; k++)
				{
					auto kd = _mm_sub_epi16(sumJThread[k], deltaV);
					auto k2 = _mm_cmple_epu16_soc(deltaV, sumJThread[k]);
					kd = _mm_and_si128(kd, k2); //[0
					sumJThreadByte[k] = _mm_min_epu16(kd, Byte128V); //[0, 128]
				}
				sumJThreadK8[0] = _mm_or_si128(sumJThreadByte[0], _mm_slli_epi16(sumJThreadByte[1], 8));
				sumJThreadK8[1] = _mm_or_si128(sumJThreadByte[2], _mm_slli_epi16(sumJThreadByte[3], 8));
				sumJThreadK8[2] = _mm_or_si128(sumJThreadByte[4], _mm_slli_epi16(sumJThreadByte[5], 8));
				sumJThreadK8[3] = _mm_or_si128(sumJThreadByte[6], _mm_slli_epi16(sumJThreadByte[7], 8));
			}

			__m128i j8 = _mm_set1_epi8(j127);

			__m128i mask = _mm_cmplt_epu8_soc(j8, sumJThreadK8[0]);		__m128i result = _mm_srli_epi8(mask, 7);
					mask = _mm_cmplt_epu8_soc(j8, sumJThreadK8[1]); result = _mm_or_si128(result, _mm_slli_epi8(_mm_srli_epi8(mask, 7), 1));
					mask = _mm_cmplt_epu8_soc(j8, sumJThreadK8[2]); result = _mm_or_si128(result, _mm_slli_epi8(_mm_srli_epi8(mask, 7), 2));
					mask = _mm_cmplt_epu8_soc(j8, sumJThreadK8[3]); result = _mm_or_si128(result, _mm_slli_epi8(_mm_srli_epi8(mask, 7), 3));


			uint32_t offsetLookup = offsetLookupTable[j];
			uint32_t lookup = slopeLookup | offsetLookup;
			maskTable[lookup] = _mm_or_si128(maskTable[lookup], result);// block;
		}

		socAssert(_mm_test_all_zeros(_mm_sub_epi32(maskTable[slopeLookup], _mm_set1_epi8(0xF)), _mm_set1_epi32(0xFFFFFFFF) ) == 1);		

		//triangle block update request the last index mask to be zero
		//otherwise, for thin vertical triangle, the convex optimization might cause top block not updated
		socAssert( _mm_test_all_zeros(maskTable[slopeLookup + 63], _mm_set1_epi32(0xFFFFFFFF) ) == 1);
		
        //// For each slope, the first block should be all ones, the last all zeroes
    }

	uint64_t rows[4];

	int startY = 0;
	if (ReflectBoostBlockMask_Optimization) startY = 32;
	uint64_t * pMask = &m_precomputedRasterTables[0] + startY * 64;
	//calculate row 32 to row 62
	int yOffset = 1 + (int)ReflectBoostBlockMask_Optimization;
	int pYOffset = (int)ReflectBoostBlockMask_Optimization * 64;
	for (int y = startY; y < 64; y+= yOffset, pMask+= pYOffset)
	{	
		int yBase = y << 6; // y * 64
		if (ReflectBoostBlockMask_Optimization) yBase >>= 1;
		//it has been found that for case x <= 1, the mask value is -1. Fully covered
		//*pMask = -1; 
		//*pMask = -1; 
		pMask += 1;
		for (int x = 1; x <= 63; x++)
		{
			int idx = yBase | x;			
			auto & lookup = maskTable[idx];
#if !defined(SDOC_ARM)
			rows[0] = uint64_t(_mm_extract_epi32(lookup, 0));
			rows[1] = uint64_t(_mm_extract_epi32(lookup, 1));
			rows[2] = uint64_t(_mm_extract_epi32(lookup, 2));
			rows[3] = uint64_t(_mm_extract_epi32(lookup, 3));
#else
            // For iOS & Android
			uint32x4_t &lookupArray = *(uint32x4_t *)&lookup;
			rows[0] = lookupArray[0];
			rows[1] = lookupArray[1];
			rows[2] = lookupArray[2];
			rows[3] = lookupArray[3];
#endif
			uint64_t block = 0;
			for (int j = 0; j < 4; j++) 
			{
				//15 7 11 3      14 6 10 2      13 5  9 1         12 4  8 0    //each contain 8 bit, the first 4 is the mask...
				uint64_t rowBlock = (rows[0] & 1); //extra pos 0
						rowBlock |= (rows[1] & 1)<<1; //extra pos 1
						rowBlock |= (rows[2] & 1)<<2; //extra pos 2
						rowBlock |= (rows[3] & 1)<<3; //extra pos 3


				rowBlock |= (rows[0] & (1 << 16) ) >> 12; //extra pos 4
				rowBlock |= (rows[1] & (1 << 16)) >> 11; //extra pos 5
				rowBlock |= (rows[2] & (1 << 16)) >> 10; //extra pos 6
				rowBlock |= (rows[3] & (1 << 16)) >> 9; //extra pos 7


				rowBlock |= (rows[0] & (1 << 8) ) ; //extra pos 8
				rowBlock |= (rows[1] & (1 << 8) ) << 1; //extra pos 9
				rowBlock |= (rows[2] & (1 << 8) ) << 2; //extra pos 10
				rowBlock |= (rows[3] & (1 << 8) ) << 3; //extra pos 11


				rowBlock |= (rows[0] & (1 << 24)) >> 12; //extra pos 12
				rowBlock |= (rows[1] & (1 << 24)) >> 11; //extra pos 13
				rowBlock |= (rows[2] & (1 << 24)) >> 10; //extra pos 14
				rowBlock |= (rows[3] & (1 << 24)) >> 9; //extra pos 15

				uint64_t perRow = rowBlock << (j << 4);
				if (block != 0 && perRow == 0) 
				{
					break;
				}
				block |= perRow;

				rows[0] >>= 1;
				rows[1] >>= 1;
				rows[2] >>= 1;
				rows[3] >>= 1;
			}

			*pMask = block; pMask++;
		}

	}


	if (ReflectBoostBlockMask_Optimization) 
	{
		

		uint8_t verticalReflectMap[256];
		for (int i = 0; i < 128; i++)
		{
			int n = i;
			//n = (128 & n) >> 7 | (64 & n) >> 5 | (32 & n) >> 3 | (16 & n) >> 1 | (8 & n) << 1 | (4 & n) << 3 | (2 & n) << 5 | (1 & n) << 7;
			n = (n & 0xaa) >> 1 | (n & 0x55) << 1;
			n = (n & 0xcc) >> 2 | (n & 0x33) << 2;
			n = (n & 0xf0) >> 4 | (n & 0x0f) << 4;
			verticalReflectMap[i] = n;
			verticalReflectMap[i | 128] = n | 1;
		}

		pMask = &m_precomputedRasterTables[0];
		pMask += 33 * 64;
		

		//time to calculate all the odd rows 
		for (int y = 33; y < 64; y+=2, pMask += 128)
		{
			//for case x = 0, the mask value is -1. Fully covered
			for (int x = 1; x <= 63; x++)
			{
				uint64_t mirror = *(pMask + x - 64);
				uint8_t * mirrow8 = (uint8_t *)(&mirror);
				uint64_t current = 0;
				uint8_t * current8 = (uint8_t *)(&current);
				for (int bx = 0; bx < 8; bx++)
				{
					current8[bx] = verticalReflectMap[mirrow8[bx]];
				}
				pMask[x] = current;
			}
		}


		//x mirror reflection to calculate 0~31 
		pMask = &m_precomputedRasterTables[0];
		for (int y = 0; y < 32; y += 1, pMask+=64)
		{
			//it has been found that for case x <= 1, the mask value is -1. Fully covered
			//*pMask = -1; pMask++;
			//*pMask = -1; pMask++;

			uint64_t* targetRow = &m_precomputedRasterTables[(62 - y) * 64];
			targetRow += (y & 1) << 7; //for odd,   change to 64 - y 
			for (int x = 1; x <= 63; x++)
			{
				pMask[x] = sdoc_bswap_64(targetRow[x]);
			}
		}
		pMask = &m_precomputedRasterTables[0];
		for (int y = 0; y < 64; y++, pMask+= 64)
		{
			pMask[0] = -1;
		}


		//here check the completeness
		bool completenessCheck = false;
		if (completenessCheck) {
			pMask = &m_precomputedRasterTables[0];
			for (int y = 0; y < 64; y++)
			{
				for (int x = 0; x < 64; x++)
				{
					int start = x + y * 64;

					int y0 = y ^ 1;
					if (y0 & 1) y0 = 64 - y0;
					else y0 = 62 - y0;

					int end = 63 - x + y0 * 64;
					uint64_t result = pMask[start] | pMask[end];
					if (result != -1)
					{
						LOGI( "Fail completeness check ");
					}
				}
			}
		}



		//9~54 difference mask elements
		//if (false) {
		//	pMask = &m_precomputedRasterTables[0];
		//	for (int y = 0; y < 64; y++, pMask += 64)
		//	{
		//		int same = 0;
		//		for (int y = 0; y < 63; y++)
		//		{
		//			same += pMask[y] == pMask[y + 1];
		//		}
		//		LOGI("Row %d diff %d", y, 64 - same);
		//	}
		//}
		pMask = &m_precomputedRasterTables[0];


		//////grant more visible pixels by comparing with cpp version
		////m_precomputedRasterTables[151] |= 18446744073692774400U;
		////m_precomputedRasterTables[166] |= 18446743523953737728U;
		////m_precomputedRasterTables[215] |= 18446744073692774400U;
		////m_precomputedRasterTables[400] |= 18446744073709547520U;
		////m_precomputedRasterTables[422] |= 18446743936270598144U;
		////m_precomputedRasterTables[464] |= 18446744073709489920U;
		////m_precomputedRasterTables[486] |= 18446743004262694912U;
		////m_precomputedRasterTables[784] |= 18446744073709353184U;
		////m_precomputedRasterTables[848] |= 18446744073696911111U;
		////m_precomputedRasterTables[923] |= 18446742961194582144U;
		////m_precomputedRasterTables[936] |= 18372699527942504448U;
		////m_precomputedRasterTables[987] |= 18446602507813585665U;
		////m_precomputedRasterTables[1000] |= 9160056689884397568U;
		////m_precomputedRasterTables[1201] |= 16204163117414875136U;
		////m_precomputedRasterTables[1265] |= 506376785949097984U;
		////m_precomputedRasterTables[1305] |= 18374401693156700400U;
		////m_precomputedRasterTables[1307] |= 18373838726023409888U;
		////m_precomputedRasterTables[1369] |= 9187131305195671311U;
		////m_precomputedRasterTables[1371] |= 9169116769247235847U;
		////m_precomputedRasterTables[1457] |= 16195156193046429696U;
		////m_precomputedRasterTables[1521] |= 505250894632255488U;
		////m_precomputedRasterTables[1711] |= 16195156194124415168U;
		////m_precomputedRasterTables[1722] |= 9259542123265392640U;
		////m_precomputedRasterTables[1775] |= 505250894665941763U;
		////m_precomputedRasterTables[1786] |= 72340172838010880U;
		////m_precomputedRasterTables[1923] |= 18446744073709485822U;
		////m_precomputedRasterTables[1958] |= 17357120220336021728U;
		////m_precomputedRasterTables[1965] |= 16204198715729174752U;
		////m_precomputedRasterTables[1987] |= 18446744073701130111U;
		////m_precomputedRasterTables[2022] |= 1082841962169960199U;
		////m_precomputedRasterTables[2029] |= 506381209866536711U;
		////m_precomputedRasterTables[4035] |= 288230376151711743U;

	}
	//initialize the Primitive Boundary Mask


	if (PairBlockNum <= PureCheckerBoardApproach+1)
	{
		int total = 64 * 64;

		pMask = &m_precomputedRasterTables[0];

		for (int idx = 0; idx < total; idx++)
		{
			pMask[idx] = CheckerBoardTransform(pMask[idx]);
		}
	}

	//clear possible memory
	if (offsetLookupTableNewed) 
	{
		delete[]offsetLookupTable;
	}
	if (this->m_depthBuffer.capacity() < OFFSET_QUANTIZATION_FACTOR * SLOPE_QUANTIZATION_FACTOR)
	{
		delete[]maskTable;
	}

	if (false) //neon version is around 200 times faster
	{
		auto endTime = std::chrono::high_resolution_clock::now();
		int totalNSNEON = (int)std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        LOGI("PrecomputeRasterizationTable time %d", (int)std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count());

		{
			std::vector<uint64_t> m_precomputedRasterTablesNEON;
			m_precomputedRasterTablesNEON.resize(m_precomputedRasterTables.size(), 0);
			std::swap(m_precomputedRasterTablesNEON, m_precomputedRasterTables);
			startTime = std::chrono::high_resolution_clock::now();
			for (uint32_t i = 0; i < angularResolution; ++i)
			{
				float angle = -0.1f + 6.4f * float(i) / (angularResolution - 1);

				float nx = std::cos(angle);
				float ny = std::sin(angle);
				float l = 1.0f / (std::abs(nx) + std::abs(ny));

				float nxo = nx; //original
				float nyo = ny; //original
				nx *= l;
				ny *= l;

				uint32_t slopeLookup = _mm_extract_epi32(quantizeSlopeLookup(_mm_set1_ps(nx), _mm_set1_ps(ny)), 0);

				for (uint32_t j = 0; j < offsetResolution; ++j)
				{
					float offset = -0.6f + 1.2f * float(j) / (angularResolution - 1);

					uint32_t offsetLookup = quantizeOffsetLookup(offset);

					uint32_t lookup = slopeLookup | offsetLookup;

					uint64_t block = 0;

					for (auto x = 0; x < 8; ++x)
					{
						for (auto y = 0; y < 8; ++y)
						{
							//groundtruth equation
							//float edgeDistance = offset + (x - 3.5f) / 8.0f * nx + (y - 3.5f) / 8.0f * ny;
							//rasterizer uses (0.5, 0.5).
							float edgeDistance = offset/l + (x - 3.5f) / 8.0f * nxo + (y - 3.5f) / 8.0f * nyo;
							if (edgeDistance <= 0.0f)
							{
								uint32_t bitIndex = 8 * x + (7 - y);
								block |= uint64_t(1) << bitIndex;
							}
						}
					}

					m_precomputedRasterTables[lookup] |= block;
				}
				// For each slope, the first block should be all ones, the last all zeroes
				socAssert(m_precomputedRasterTables[slopeLookup] == -1);
			// current row idx 1 is not -1 and row idx 63 is not 0
			//	socAssert(m_precomputedRasterTables[slopeLookup + OFFSET_QUANTIZATION_FACTOR_MINUS_ONE] == 0);
			}
			
			endTime = std::chrono::high_resolution_clock::now();
			int totalCPP = (int)std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();


			int diffCount = 0;
			int largeCount = 0;
			for (int idx = 0; idx < m_precomputedRasterTablesNEON.size(); idx++)
			{
				if (m_precomputedRasterTablesNEON[idx] != m_precomputedRasterTables[idx])
				{
					int idx64 = idx & 63;
					if(idx64 == 0 || idx64 == 1 || idx64 == 63) continue;

#if defined(SDOC_NATIVE)
					std::cout << "m_precomputedRasterTables[" << idx << "]" << " = " << m_precomputedRasterTables[idx]<<"U;" << std::endl;
#endif
					diffCount++;
				}
				if (m_precomputedRasterTablesNEON[idx] > m_precomputedRasterTables[idx]) 
				{
					largeCount++;
				}
			}

            // Use NEON input
			std::swap(m_precomputedRasterTablesNEON, m_precomputedRasterTables);

            LOGI("CPP vs NEON  %lf   ResultDiffCount %d largeCount %d totalCPP %d totalNSNEON %d", totalCPP * 1.0f / totalNSNEON, diffCount, largeCount, totalCPP, totalNSNEON);
		}

	}
}


void Rasterizer::doRasterize(common::OccluderMesh& raw, OccluderRenderCache* occluderCache)
{
		int key = (int)occluderCache->NeedsClipping << 1;  //2
		key |= (int)(raw.Indices == nullptr);                    //1
		key |=  raw.EnableBackface << 3; //8
		key |= raw.SuperCompress << 2;  // raw.Indices must be false  //4,   remove case of 4 6 12 14
		key |= raw.IsOccludee << 4;

		occluderCache->OccludeeMesh = (bool)raw.IsOccludee;
		switch (key)
		{
		case 0: rasterize<0>(raw, occluderCache); break;
		case 1: rasterize<1>(raw, occluderCache); break;
		case 2: rasterize<2>(raw, occluderCache); break;
		case 3: rasterize<3>(raw, occluderCache); break;
		//case 4: rasterize<4>(raw); break;
		case 5: rasterize<5>(raw, occluderCache); break;
		//case 6: rasterize<6>(raw); break;
		case 7: rasterize<7>(raw, occluderCache); break;
		case 8: rasterize<8>(raw, occluderCache); break;
		case 9: rasterize<9>(raw, occluderCache); break;
		case 10: rasterize<10>(raw, occluderCache); break;
		case 11: rasterize<11>(raw, occluderCache); break;
		//case 12: rasterize<12>(raw); break;
		case 13: rasterize<13>(raw, occluderCache); break;
		//case 14: rasterize<14>(raw); break;
		case 15: rasterize<15>(raw, occluderCache); break;



		case 16: rasterize<16>(raw, occluderCache); break;
		case 17: rasterize<17>(raw, occluderCache); break;
		case 18: rasterize<18>(raw, occluderCache); break;
		case 19: rasterize<19>(raw, occluderCache); break;
			//case 4: rasterize<4>(raw); break;
		case 21: rasterize<21>(raw, occluderCache); break;
			//case 6: rasterize<6>(raw); break;
		case 23: rasterize<23>(raw, occluderCache); break;
		case 24: rasterize<24>(raw, occluderCache); break;
		case 25: rasterize<25>(raw, occluderCache); break;
		case 26: rasterize<26>(raw, occluderCache); break;
		case 27: rasterize<27>(raw, occluderCache); break;
			//case 12: rasterize<12>(raw); break;
		case 29: rasterize<29>(raw, occluderCache); break;
			//case 14: rasterize<14>(raw); break;
		case 31: rasterize<31>(raw, occluderCache); break;

		default:
			break;
		}

	
}

inline static void Transpose(const float ** triangleData, float *dataf)
{
	//_MM_TRANSPOSE4_PS_43(dataArray[0], dataArray[3], dataArray[6], dataArray[9]);

	//0 12 24
	const float* f = triangleData[0];
	dataf[0] = f[0];
	dataf[12] = f[1];
	dataf[24] = f[2];

	f = triangleData[3];
	dataf[1] = f[0];
	dataf[13] = f[1];
	dataf[25] =f[2];


	f = triangleData[6];
	dataf[2] = f[0];
	dataf[14] = f[1];
	dataf[26] = f[2];

	f = triangleData[9];
	dataf[3] = f[0];
	dataf[15] = f[1];
	dataf[27] = f[2];
}

static int GetValidPrimitiveNum(__m128 primitiveValid) 
{
	uint32_t * v = (uint32_t *)&primitiveValid;
	return (v[0] >> 31) + (v[1] >> 31) + (v[2] >> 31) + (v[3] >> 31);
}


//draw convex quad, bPixelAABBClippingQuad is always TRUE for correctness
template <bool bPixelAABBClippingQuad, bool bDrawOccludee>
void Rasterizer::drawQuad(__m128* x, __m128* y, __m128* invW, __m128* W,  __m128 primitiveValid, __m128* edgeNormalsX, __m128* edgeNormalsY, __m128* areas, OccluderRenderCache* occluderCache)
{
	if (bDrawOccludee && bSplitQuadToTriangleForOccludee) return;
	PrimitiveBoundaryClipCache* PrimitivePixelClip = &occluderCache->mPixelClip;
	__m128 minFx, minFy, maxFx, maxFy;

	
	// Standard bounding box inclusion
	minFx = _mm_min_ps(_mm_min_ps(x[0], x[1]), _mm_min_ps(x[2], x[3]));
	maxFx = _mm_max_ps(_mm_max_ps(x[0], x[1]), _mm_max_ps(x[2], x[3]));

	minFy = _mm_min_ps(_mm_min_ps(y[0], y[1]), _mm_min_ps(y[2], y[3]));
	maxFy = _mm_max_ps(_mm_max_ps(y[0], y[1]), _mm_max_ps(y[2], y[3]));
	

	// Clamp upper bound and unpack
	__m128i bounds[4];

	bounds[0] = _mm_max_epi32(_mm_cvttps_epi32(minFx), _mm_set1_epi32(mBlockWidthMin));
	bounds[1] = _mm_min_epi32(_mm_cvttps_epi32(maxFx), _mm_set1_epi32(mBlockWidthMax));
	bounds[2] = _mm_max_epi32(_mm_cvttps_epi32(minFy), _mm_setzero_si128());
	bounds[3] = _mm_min_epi32(_mm_cvttps_epi32(maxFy), _mm_set1_epi32(m_blocksYMinusOne));

	
	// Check overlap between bounding box and frustum
	__m128 isInFrustum = _mm_castsi128_ps(_mm_and_si128(_mm_cmple_epi32_soc(bounds[0], bounds[1]), _mm_cmple_epi32_soc(bounds[2], bounds[3])));
	__m128 one16 = _mm_set1_ps(1.0 / 16.0);
	isInFrustum = _mm_and_ps(isInFrustum, _mm_cmpge_ps(maxFx, one16));
	isInFrustum = _mm_and_ps(isInFrustum, _mm_cmpge_ps(maxFy, one16));

	primitiveValid = _mm_and_ps(isInFrustum, primitiveValid);

	if (DebugOccluderOccludee)
	{
		int temp = GetValidPrimitiveNum(primitiveValid);
		this->DebugData[PrimitiveFrustumCull] += (DebugData[PrimitiveValidNum] - temp) * 2;
		DebugData[PrimitiveValidNum] = temp;
	}
	uint32_t validMask = _mm_movemask_ps(primitiveValid);
	if (validMask == 0)
	{
		if (DebugOccluderOccludee)
		{
			this->DebugData[P4FrustumCull]++;
			this->DebugData[P4QuadFrustumCull]++;
		}
		return;
	}
	if (DebugOccluderOccludee)
	{
		this->DebugData[P4QuadFrustumPass]++;
	}


	// Compute Z from linear relation with 1/W
	__m128 z[4];
	__m128 c0 = occluderCache->c0;
	__m128 c1 = occluderCache->c1;
	z[0] = _mm_fmadd_ps_soc(invW[0], c1, c0);
	z[1] = _mm_fmadd_ps_soc(invW[1], c1, c0);
	z[2] = _mm_fmadd_ps_soc(invW[2], c1, c0);
	z[3] = _mm_fmadd_ps_soc(invW[3], c1, c0);

	__m128 maxZ = _mm_max_ps(_mm_max_ps(z[0], z[1]), _mm_max_ps(z[2], z[3]));

	__m128i maxZi = _mm_srai_epi32(_mm_castps_si128(maxZ), 12); 

	__m128 maxArea = _mm_max_ps(areas[1], areas[0]);

	maxZi = _mm_min_epu32(maxZi, _mm_set1_epi32(65535));
	uint32_t* depthBounds = (uint32_t*)&maxZi;

	//we could use hiZ to do a quick culling. 

	//quick cull check
	if (CULL_FEATURE_HizPrimitiveCull)
	{
		uint32_t alivePrimitive = 0;
		uint16_t *pHiZBuffer = m_pHiz;
		uint32_t pValidIdx = mAliveIdxMask[validMask];
		do
		{
			// Move index and mask to next set bit
			uint32_t primitiveIdx = pValidIdx & 3;
			pValidIdx >>= 2;

			// Extract and prepare per-primitive data
			uint16_t primitiveMaxZ = depthBounds[primitiveIdx];
			//do a quick check here to reject occluded primitive
			{
				uint32_t * boundData = ((uint32_t*)bounds) + primitiveIdx;
				uint32_t blockMinX = boundData[0];
				uint32_t blockMaxX = boundData[4];
				const uint32_t blockMinY = boundData[8];
				uint32_t blockMaxY = boundData[12];


				uint16_t *pOffsetHiZ = pHiZBuffer + (m_blocksX * blockMinY + blockMinX);

				//reduce 1 and then blockRangeY = BlockMax-BlockMin, then loop  [0, BlockMax - blockMin]
				//blockRangeY--; //bounds[1] = _mm_add_epi32(_mm_sub_epi32(bounds[1], bounds[0]), _mm_set1_epi32(1));
				//blockRangeX--; //bounds[3] = _mm_add_epi32(_mm_sub_epi32(bounds[3], bounds[2]), _mm_set1_epi32(1));
				int blockRangeX = blockMaxX - blockMinX;
				__m128i primitiveMaxZVi = _mm_set1_epi16(primitiveMaxZ);
				if (DebugOccluderOccludee)
				{
					DebugData[BlockDoWhileIfSave]++;
				}
				uint32_t NextBlockY = blockMinY;
				do
				{
					uint32_t blockY = NextBlockY++;
					uint16_t *pBlockRowHiZ = pOffsetHiZ;
					pOffsetHiZ += m_blocksX;


					if (DebugOccluderOccludee)
					{
						DebugData[BlockDoWhileIfSave]++;
					}
					int32_t NextRowRangeX = blockRangeX;
					do
					{
						int32_t rowRangeX = NextRowRangeX;
						NextRowRangeX -= 8;

						// Load HiZ for 8 blocks at once - note we're possibly reading out-of-bounds here; but it doesn't affect correctness if we test more blocks than actually covered
						__m128i hiZblob = _mm_loadu_si128(reinterpret_cast<const __m128i *>(pBlockRowHiZ));
						__m128i cmpResult;
						if (bDrawOccludee)	cmpResult = _mm_cmple_epu16_soc(hiZblob, primitiveMaxZVi);
						else 				cmpResult = _mm_cmplt_epu16_soc(hiZblob, primitiveMaxZVi);
						uint64_t * r64 = (uint64_t *)& cmpResult;
						uint64_t result = r64[0] & r64[1];

						if (result != -1)
						{
							int32_t byteBlockCheck = std::min(7, rowRangeX);
							int32_t bit = byteBlockCheck >> 2;
							int32_t offset = byteBlockCheck & 3;
							r64[bit] <<= (3 ^ offset) << 4;

							r64[1] &= (uint64_t)(-bit);


							result = r64[0] | r64[1];

							if (result == 0)
							{
								pBlockRowHiZ += 8;
								continue;
							}
						}

						boundData[8] = blockY;

						alivePrimitive |= 1 << primitiveIdx;
						blockMaxY = 0; //reset blockMaxY  to 0 to force do while exit

						break;
					} while (NextRowRangeX >= 0);
					} while (NextBlockY <= blockMaxY);

					if (DebugOccluderOccludee)
					{
						uint32_t checkMask = 1 << primitiveIdx;
						if ((alivePrimitive & checkMask) == 0)
						{
							DebugData[PrimitiveEarlyHiZCull]++;
							DebugData[PrimitiveValidNum] --;
						}
					}

				}
			} while (pValidIdx != 0);

			if (alivePrimitive == 0)
			{
				if (DebugOccluderOccludee)
				{
					DebugData[P4EarlyHizCull] ++;
					assert(DebugData[PrimitiveValidNum] == 0);
				}
				return;
			}

			validMask = mAliveIdxMask[alivePrimitive];
	}


	__m128i minZi;
	uint32_t* depthBoundsMin;
	{
		__m128 minZ = _mm_min_ps(_mm_min_ps(z[0], z[1]), _mm_min_ps(z[2], z[3]));// _mm_fmadd_ps(_mm_min_ps(_mm_min_ps(_mm_min_ps(invW[0], invW[1]), invW[2]), invW[3]), occluderCache->c1, occluderCache->c0);
		minZ = _mm_and_ps(minZ, _mm_cmpgt_ps(minZ, _mm_setzero_ps()));
		minZi = _mm_castps_si128(minZ);
		depthBoundsMin = (uint32_t*)&minZi;
	}


	if (bPixelAABBClippingQuad)
	{
		auto scale8 = _mm_set1_ps(8);
		__m128i mask = _mm_set1_epi32((uint32_t(-1) << 16) | 7);

		__m128i pixel = _mm_cvttps_epi32(_mm_mul_ps(minFx, scale8));
		//make bit 16~31 store block value for each int component
		//make bit 0~15 store block pixel remainder(0~7)
		PrimitivePixelClip->PrimitivePixelBounds[0] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(maxFx, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[1] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(minFy, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[2] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(maxFy, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[3] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
	}

	if (this->mDebugRenderMode && bDrawOccludee == false){
		HandleDrawMode<4>(x, y, z, validMask, occluderCache);
	}

	if (DebugOccluderOccludee)
	{
		DebugData[P4EarlyHizCullPass] ++;
		this->DebugData[BatchQuad4Rasterized]++;
	}


	// Compute screen space depth plane
	__m128 depthPlane[3];

	__m128 greaterArea = _mm_cmpeq_ps(maxArea, areas[1]);

	__m128 invArea = _mm_rcp_ps_div(maxArea);


	__m128 z12 = _mm_sub_ps(z[1], z[2]);
	__m128 z20 = _mm_sub_ps(z[2], z[0]);
	__m128 z30 = _mm_sub_ps(z[3], z[0]);





	//delay the calculation of edgeNormalX4, edgeNormalY4
	__m128 edgeNormalsX4 = _mm_sub_ps(y[0], y[2]);
	__m128 edgeNormalsY4 = _mm_sub_ps(x[2], x[0]);

	if (bDrawOccludee) {
		auto a1 = _mm_mul_ps(areas[0], _mm_fmsub_ps_soc(z20, edgeNormalsX[1], _mm_mul_ps(z12, edgeNormalsX4)));
		auto a2 = _mm_mul_ps(areas[0], _mm_fmsub_ps_soc(z20, edgeNormalsY[1], _mm_mul_ps(z12, edgeNormalsY4)));
		auto d1 = _mm_mul_ps(areas[1], _mm_fnmadd_ps_soc(z20, edgeNormalsX[3], _mm_mul_ps(z30, edgeNormalsX4)));
		auto d2 = _mm_mul_ps(areas[1], _mm_fnmadd_ps_soc(z20, edgeNormalsY[3], _mm_mul_ps(z30, edgeNormalsY4)));


		auto a = _mm_sub_ps(z[0], _mm_fmadd_ps_soc(x[0], a1, _mm_mul_ps(y[0], a2)));
		auto b = _mm_sub_ps(z[1], _mm_fmadd_ps_soc(x[1], a1, _mm_mul_ps(y[1], a2)));
		auto c = _mm_sub_ps(z[2], _mm_fmadd_ps_soc(x[2], d1, _mm_mul_ps(y[2], d2)));
		auto d = _mm_sub_ps(z[3], _mm_fmadd_ps_soc(x[3], d1, _mm_mul_ps(y[3], d2)));

		depthPlane[0] = _mm_max_ps(_mm_max_ps(a, b), _mm_max_ps(c, d));

		depthPlane[1] = _mm_max_ps(a1, d1);
		depthPlane[2] = _mm_max_ps(a2, d2);
	}
	else
	{
		// Depth delta X/Y - select the derivatives from the triangle with the greater area, which is numerically more stable
		depthPlane[1] = _mm_mul_ps(invArea,
			_mm_blendv_ps(
				_mm_fmsub_ps_soc(z20, edgeNormalsX[1], _mm_mul_ps(z12, edgeNormalsX4)),
				_mm_fnmadd_ps_soc(z20, edgeNormalsX[3], _mm_mul_ps(z30, edgeNormalsX4)),
				greaterArea));
		depthPlane[2] = _mm_mul_ps(invArea,
			_mm_blendv_ps(
				_mm_fmsub_ps_soc(z20, edgeNormalsY[1], _mm_mul_ps(z12, edgeNormalsY4)),
				_mm_fnmadd_ps_soc(z20, edgeNormalsY[3], _mm_mul_ps(z30, edgeNormalsY4)),
				greaterArea));




		
		// Depth at center of first pixel
		__m128 refX = _mm_sub_ps(_mm_set1_ps(1.0f / 16.0f), x[0]);
		__m128 refY = _mm_sub_ps(_mm_set1_ps(1.0f / 16.0f), y[0]);
		depthPlane[0] = _mm_fmadd_ps_soc(refX, depthPlane[1], _mm_fmadd_ps_soc(refY, depthPlane[2], z[0]));
	}



	// Normalize edge equations for lookup
	__m128 invLen[4];
	// Quantize slopes
	__m128i slopeLookups[4];
	normalizeEdgeAndQuantizeSlope(edgeNormalsX[0], edgeNormalsY[0], invLen[0], slopeLookups[0]);
	normalizeEdgeAndQuantizeSlope(edgeNormalsX[1], edgeNormalsY[1], invLen[1], slopeLookups[1]);
	normalizeEdgeAndQuantizeSlope(edgeNormalsX[2], edgeNormalsY[2], invLen[2], slopeLookups[2]);
	normalizeEdgeAndQuantizeSlope(edgeNormalsX[3], edgeNormalsY[3], invLen[3], slopeLookups[3]);

	__m128 edgeOffsets[4];
	// Important not to use FMA here to ensure identical results between neighboring edges
	edgeOffsets[0] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[1], y[0]), _mm_mul_ps(y[1], x[0])), invLen[0]);
	edgeOffsets[1] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[2], y[1]), _mm_mul_ps(y[2], x[1])), invLen[1]);
	edgeOffsets[2] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[3], y[2]), _mm_mul_ps(y[3], x[2])), invLen[2]);
	edgeOffsets[3] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[0], y[3]), _mm_mul_ps(y[0], x[3])), invLen[3]);
	
	// Fetch data pointers since we'll manually strength-reduce memory arithmetic
	uint16_t *pHiZBuffer = m_pHiz;

	do
	{
		uint32_t primitiveIdx = validMask & 3;
		validMask >>= 2;

		if (DebugOccluderOccludee)
		{
			this->DebugData[PrimitiveRasterizedQuadNum] ++;
		}

		uint32_t* slopeLookup = ((uint32_t*)&slopeLookups) + primitiveIdx;	
		const uint64_t* pRow0 = m_pMaskTable + slopeLookup[0];
		const uint64_t* pRow1 = m_pMaskTable + slopeLookup[4];
		const uint64_t* pRow2 = m_pMaskTable + slopeLookup[8];
		const uint64_t* pRow3 = m_pMaskTable + slopeLookup[12];
		

		// Extract and prepare per-primitive data
		uint32_t primitiveMaxZf = depthBounds[primitiveIdx];
		uint16_t primitiveMaxZ = (uint16_t)primitiveMaxZf;
		uint32_t primitiveMinZf = depthBoundsMin[primitiveIdx];

		
		primitiveMaxZf <<= 12;
		float* depthPlaneData = ((float*)depthPlane) + primitiveIdx;




		float slope = depthPlaneData[8];

		int xIncrease = (int)(depthPlaneData[4] > 0);
		int yIncrease = (int)(slope > 0);
		//data: btmLeft 0 btmRight 0 topleft 0 topright 0
		int maxBlockIdx = (xIncrease + yIncrease * 2) << 1;
		int minBlockIdx = 6 ^ maxBlockIdx;

		//***********************************************************

		__m128 depthBlockDelta = _mm_set1_ps(slope);

		// aggressive approach
		__m128 depthRowDeltaBtm = _mm_setzero_ps();
		depthRowDeltaBtm = _mm_min_ps(_mm_set1_ps(slope* 0.375f), depthRowDeltaBtm);


		__m128 depthDx = _mm_set1_ps(depthPlaneData[4]);
		__m128 depthLeftBase;
		if (VRS_X4Y4_Optimzation)
		{
			depthLeftBase = _mm_fmadd_ps_soc(depthDx, xFactors[xIncrease], _mm_set1_ps(depthPlaneData[0]));
			float halfSlope = slope * 0.5f;
			depthLeftBase = _mm_add_ps(depthLeftBase, _mm_setr_ps(0, 0, halfSlope, halfSlope));
		}
		else
		{
			depthLeftBase = _mm_fmadd_ps_soc(depthDx, _mm_setr_ps(0.0f, 0.125f, 0.25f, 0.375f), _mm_set1_ps(depthPlaneData[0]));
		}


		uint32_t * boundData = ((uint32_t*)bounds) + primitiveIdx;
		uint32_t blockMinX = boundData[0];
		uint32_t blockMaxX = boundData[4];
		const uint32_t blockMinY = boundData[8];
		uint32_t blockMaxY = boundData[12];



		float * edgeNormalsXf = (float*)edgeNormalsX + primitiveIdx;
		float * edgeNormalsYf = (float*)edgeNormalsY + primitiveIdx;
		float * edgeOffsetsf = (float*)edgeOffsets + primitiveIdx;

		
		__m128 edgeNormalsXP = _mm_setr_ps(edgeNormalsXf[0], edgeNormalsXf[4], edgeNormalsXf[8], edgeNormalsXf[12] );
		__m128 edgeNormalsYP = _mm_setr_ps(edgeNormalsYf[0], edgeNormalsYf[4], edgeNormalsYf[8], edgeNormalsYf[12]);
		__m128 edgeOffsetsP = _mm_setr_ps(edgeOffsetsf[0], edgeOffsetsf[4], edgeOffsetsf[8], edgeOffsetsf[12]);

		//delay calculation of edgeNormalsX edgeNormalsY
		//_mm_add_ps(edgeNormalsX[primitiveIdx], edgeNormalsY[primitiveIdx]), _mm_set1_ps(0.5f) is the central point of 8x8 block
		__m128 edgeOffset = _mm_fmadd_ps_soc(_mm_add_ps(edgeNormalsXP, edgeNormalsYP), _mm_set1_ps(0.5f), edgeOffsetsP);
		__m128 edge_mul = _mm_set1_ps(OFFSET_mul);
		edgeOffset = _mm_fmadd_ps_soc(edgeOffset, edge_mul, _mm_set1_ps(OFFSET_add));

		__m128 edgeNormalX = _mm_mul_ps(edgeNormalsXP, edge_mul);
		__m128 edgeNormalY = _mm_mul_ps(edgeNormalsYP, edge_mul);


		const uint32_t blocksX = m_blocksX;
				
		uint32_t startYBlocks = blocksX * blockMinY;
		uint16_t *pOffsetHiZ = pHiZBuffer + startYBlocks;


		uint64_t *outblockRowData = m_pDepthBuffer + startYBlocks * PairBlockNum;

		__m128 rowDepthLeftBtmOffset = _mm_add_ps(depthLeftBase, depthRowDeltaBtm);

		
		if (bPixelAABBClippingQuad)
		{
			PrimitivePixelClip->UpdatePixelAABBData(primitiveIdx, mPixelBound);
		}
		if (DebugOccluderOccludee) {
			DebugData[BlockTotalPrimitives]++;
		}

		uint32_t blockY = blockMinY;
		uint32_t anythingDraw = 0;
		__m128 edgeOffsetMin = _mm_fmadd_ps_soc(edgeNormalX, _mm_set1_ps(float(blockMinX)), edgeOffset);
		while(true)
		{

			__m128 blockYf = _mm_set1_ps(float(blockY));
			__m128 rowDepthLeftBtm = _mm_fmadd_ps_soc(depthBlockDelta, blockYf, rowDepthLeftBtmOffset);

			__m128 offset = _mm_fmadd_ps_soc(edgeNormalY, blockYf, edgeOffsetMin);

			
			uint32_t blockRowOffset = 0;
			for (uint32_t blockX = blockMinX; blockX <= blockMaxX; blockX++,  offset = _mm_add_ps(edgeNormalX, offset))
			{
				__m128i lookup = _mm_cvttps_epi32(offset);

				lookup = _mm_max_epi32(lookup, _mm_setzero_si128());
				uint32_t* lookIdx = (uint32_t*)&lookup;
				uint32_t idxOr = lookIdx[0] | lookIdx[1] | lookIdx[2] | lookIdx[3];

				if (idxOr > 63)
				{	
					//Convex Optimization 0: YesNo optimization. Stop if Block state from see to not see
					if(bConvexOptimization)
					{
						blockX |= blockRowOffset;
					}
					continue;
				}
				if (bConvexOptimization) 
				{
					blockRowOffset = 65536;
				}
				uint16_t *pBlockRowHiZ = pOffsetHiZ + blockX;
				if (pBlockRowHiZ[0] >= primitiveMaxZ)
				{
					if (DebugOccluderOccludee) {
						DebugData[BlockPrimitiveMaxLessThanMinCull]++;
					}
					continue;
				}

				uint64_t blockMask = -1;
				if (idxOr != 0) 
				{
					blockMask  = pRow0[lookIdx[0]];
					blockMask &= pRow1[lookIdx[1]];
					blockMask &= pRow2[lookIdx[2]];
					blockMask &= pRow3[lookIdx[3]];

					// No pixels covered => skip block
					if (!blockMask)
					{
						if (DebugOccluderOccludee) 
						{
							DebugData[BlockMaskJointZeroCull]++;
						}
						continue;
					}
				}

				if (bDumpBlockColumnImage)
				{
					if (blockX == DebugDumpBlockX && blockY == DebugDumpBlockY) {}
					else	continue;
				}
				

				//drawQuad routine
				if (VRS_X4Y4_Optimzation)
				{
					__m128i rowDepthLeft = _mm_castps_si128(_mm_fmadd_ps_soc(depthDx, _mm_set1_ps((float)blockX), rowDepthLeftBtm));

					rowDepthLeft = _mm_max_epi32(rowDepthLeft, _mm_set1_epi32(primitiveMinZf));
					rowDepthLeft = _mm_min_epi32(rowDepthLeft, _mm_set1_epi32(primitiveMaxZf));


					__m128i depthData = packDepthPremultipliedVRS12Fast(rowDepthLeft);
					
					uint16_t * depth16 = (uint16_t*)&depthData;

					uint16_t maxBlockDepth = depth16[maxBlockIdx];
					if (bDebug_ValidateMinMaxBlockIdx) {
						uint16_t minDepth = depth16[minBlockIdx];
						for (int xx = 0; xx < 8; xx++) {
							assert(depth16[xx] <= maxBlockDepth);
							assert(depth16[xx] >= minDepth);
						}
					}
					if (maxBlockDepth > pBlockRowHiZ[0])
					{
						uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;

						uint32_t * depth32 = (uint32_t*)depth16;
						if (PairBlockNum <= CheckerBoardVizMaskApproach)
						{
							if (blockMask != -1)
							{

								if (bPixelAABBClippingQuad)
								{
									blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
									if (blockMask == 0)
									{
										if (DebugOccluderOccludee)
										{
											DebugData[BlockAABBClipToZero]++;
										}
										continue;
									}
								}

								if (DebugOccluderOccludee)
								{
									DebugData[BlockRenderPartial]++;
									DebugData[BlockRenderTotal]++;
								}

								if (PairBlockNum == PureCheckerBoardApproach)
								{
									__m128i* out = (__m128i*)outBlockData;
									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										updateBlockMSCBPartial_Occludee(depth32, blockMask, out, occluderCache);
										if (occluderCache->mRasterizedOccludeeVisible)
											return;
									}
									else {
										updateBlockMSCBPartial(depth32, blockMask, out, nullptr, pBlockRowHiZ, maxBlockDepth);
									}
								}
								else {
									int bit = blockX & 1;
									__m128i* out = GetDepthData(outBlockData, bit);
									uint64_t* maskData = GetMaskData(outBlockData, bit);

									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										updateBlockMSCBPartial_Occludee(depth32, blockMask, out, occluderCache);
										if (occluderCache->mRasterizedOccludeeVisible)
											return;
									}
									else {
										updateBlockMSCBPartial(depth32, blockMask, out, maskData, pBlockRowHiZ, maxBlockDepth);
									}
								}
							}
							else //full block update
							{
								mUpdateAnyBlock = true;
								if (DebugOccluderOccludee)
								{
									DebugData[BlockRenderFull]++;
									DebugData[BlockRenderTotal]++;
									DebugData[BlockMinUseOne]++;
								}



								__m128i* out = nullptr;
								if (PairBlockNum == CheckerBoardVizMaskApproach)
								{
									int bit = blockX & 1;
									out = GetDepthData(outBlockData, bit);
									uint64_t* maskData = GetMaskData(outBlockData, bit);
									maskData[0] = -1;
								}
								else {
									out = (__m128i*) outBlockData;
								}

								uint16_t minBlockDepth = depth16[minBlockIdx];
								// All pixels covered => skip edge tests

								uint16_t * pBlockRowHiZMax = pBlockRowHiZ + m_HizBufferSize;
								if (minBlockDepth >= pBlockRowHiZMax[0]) //full block update, min is larger than exist max
								{
									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										occluderCache->mRasterizedOccludeeVisible = true;
										return;
									}

									if (DebugOccluderOccludee) {
										DebugData[BlockRenderInitial]++;
										DebugData[BlockRenderInitialFull]++;
									}

									__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
									__m128i depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
									out[0] = depthBottom;
									out[1] = depthBottom;
									out[2] = depthTop;
									out[3] = depthTop;
									pBlockRowHiZ[0] = minBlockDepth;

									pBlockRowHiZ[m_HizBufferSize] = maxBlockDepth; //update max hiz
								}
								else
								{
									__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
									__m128i depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										__m128i visible = _mm_cmple_epu16_soc(out[0], depthBottom);
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[1], depthBottom));
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[2], depthTop));
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[3], depthTop));
										if (_mm_anybit_one_soc(visible)) {
											occluderCache->mRasterizedOccludeeVisible = true;
											return;
										}
									}
									else {
										out[0] = _mm_max_epu16(out[0], depthBottom);
										out[1] = _mm_max_epu16(out[1], depthBottom);
										out[2] = _mm_max_epu16(out[2], depthTop);
										out[3] = _mm_max_epu16(out[3], depthTop);


										pBlockRowHiZ[0] = std::max<uint16_t>(minBlockDepth, pBlockRowHiZ[0]);
										pBlockRowHiZMax[0] = std::max<uint16_t>(maxBlockDepth, pBlockRowHiZMax[0]);
									}
								}
							}
						}
						else if (PairBlockNum == FullBlockApproach) 
						{
#if defined( SUPPORT_ALL_FEATURE)
							if (bPixelAABBClippingQuad) {
								if (blockMask != -1 && bPixelAABBClipping)
								{
									blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
								}
							}
							__m128i depthRows[2];
							depthRows[0] = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
							depthRows[1] = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
							if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
								occluderCache->mRasterizedOccludeeVisible = updateBlock_Occludee(depthRows, blockMask, (__m128i*)(outBlockData), pBlockRowHiZ);
								if (occluderCache->mRasterizedOccludeeVisible)
								{
									return;
								}
							}
							else {
								updateBlock(depthRows, blockMask, (__m128i*)(outBlockData), pBlockRowHiZ);
							}
#endif
						}
					}
					else
					{
						if (DebugOccluderOccludee) {
							DebugData[BlockMaxLessThanMinCull]++;
						}
					}

#if defined( SUPPORT_ALL_FEATURE)
					if (bDumpBlockColumnImage)
					{
						uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;
						uint64_t* maskData = GetMaskData(outBlockData, blockX & 1);

						uint64_t t0 = pRow0[lookIdx[0]];
						uint64_t t1 = pRow1[lookIdx[1]];
						uint64_t t2 = pRow2[lookIdx[2]];
						DumpColumnBlock(t0, t1, t2, primitiveMaxZ, maskData[0]);
					}
#endif
				}
				else {
#if defined( SUPPORT_ALL_FEATURE)
					__m128 depthDxHalf = _mm_set1_ps(depthPlaneData[4] * 0.5f);
					__m128 lineDepthLeft = _mm_fmadd_ps_soc(depthBlockDelta, _mm_set1_ps(float(blockY)), depthLeftBase);
					__m128 rowDepthLeft = _mm_fmadd_ps_soc(depthDx, _mm_set1_ps((float)blockX), lineDepthLeft);
					__m128 rowDepthRight = _mm_add_ps(depthDxHalf, rowDepthLeft);

					if (blockMask != -1 && bPixelAABBClippingQuad)
					{
						blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
					}

					__m128 depthRowDelta = _mm_set1_ps(slope * 0.125f);

					uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;
					updateBlockWithMaxZ(rowDepthLeft, rowDepthRight, blockMask, depthRowDelta, _mm_set1_epi32(primitiveMaxZf), (__m128i*) outBlockData, pBlockRowHiZ);
#endif
				}
			}
			if (blockY >= blockMaxY)
			{
				break;
			}
			//from draw to undraw
			if (anythingDraw > blockRowOffset) 
			{
				break;
			}
			anythingDraw = blockRowOffset;
			blockY++;
			pOffsetHiZ += blocksX;
			outblockRowData += m_blocksXFullDataRows;
		}
	}
	while (validMask > 0);
}


template <int RASTERIZE_CONFIG>
void Rasterizer::rasterize(common::OccluderMesh& raw, OccluderRenderCache* occluderCache)
{
	constexpr bool PrimitiveDataCompressed = RASTERIZE_CONFIG & 1;
	constexpr bool possiblyNearClipped = (bool)(RASTERIZE_CONFIG & 2);
	constexpr bool bBackFaceCulling =  (bool)(RASTERIZE_CONFIG & 8) && !bDebugForceBackfaceCullingOFF;
	constexpr bool bSuperCompressed = (bool)(RASTERIZE_CONFIG & 4);


	constexpr bool bDrawOccludee = (bool)(RASTERIZE_CONFIG & 16);

	if (bDrawOccludee && !Rasterize_ClippedOccludee_AS_OCCLUDER) {
		return;
	}
	if (!bDrawOccludee && bDrawOccludeeToDepthMap) return;

	if (DebugOccluderOccludee) {
		DebugData[OccluderRasterized] ++;
		////use to select terrain to debug
		//if (DebugData[OccluderRasterized] != 72)
		//	return;

		DebugData[RasterizedOccluderTotalTriangles] += raw.TriangleBatchIdxNum / 3;
		DebugData[RasterizedOccluderTotalVertices] += raw.VerticesNum;
	}

	__m128 * mat = occluderCache->mat;


	float* matF = (float*)mat;

	if (bFastSetUpMatOp == false) {
		// *****_MM_TRANSPOSE4_PS*****
		_MM_TRANSPOSE4_PS(mat[0], mat[1], mat[2], mat[3]);
		
		__m128 Za = _mm_shuffle_ps_single_index(mat[2], 3);
		__m128 Zb = _mm_sum4_ps_soc(mat[2]);

		__m128 Wa = _mm_shuffle_ps_single_index(mat[3], 3);
		__m128 Wb = _mm_sum4_ps_soc(mat[3]);

		__m128 c0 = _mm_div_ps(_mm_sub_ps(Za, Zb), _mm_sub_ps(Wa, Wb));
		//DC2: degenerate case 2
		//add zero check to remove case of Wa = Wb. This would eliminate degenerate case..
		__m128 zero_mask = _mm_cmpneq_ps(Wa, Wb);
		c0 = _mm_and_ps(c0, zero_mask);

		occluderCache->c1 = _mm_fnmadd_ps_soc(c0, Wa, Za);
		occluderCache->c0 = c0;


		auto one8th = _mm_set1_ps(0.125f);
		mat[0] = _mm_mul_ps(mat[0], one8th); // scale down by 8


		mat[1] = _mm_mul_ps(mat[1], one8th); // scale down by 8
	}
	else 
	{
		__m128 matT0 = _mm_setr_ps(matF[0], matF[4], matF[8], matF[12]);
		__m128 matT1 = _mm_setr_ps(matF[1], matF[4 + 1], matF[8 + 1], matF[12 + 1]);
		__m128 matT3 = _mm_setr_ps(matF[3], matF[4 + 3], matF[8 + 3], matF[12 + 3]);

		float za_bf = matF[0 * 4 + 2] + matF[1 * 4 + 2] + matF[2 * 4 + 2];
		if (za_bf == 0) //most of Occluders should have no scale which means only R|T
		{
			occluderCache->c1 = _mm_set1_ps(matF[3 * 4 + 2]);// _mm_shuffle_ps_single_index(mat[2], 3);
			occluderCache->c0 = _mm_set1_ps(0);
		}
		else
		{
			//_MM_TRANSPOSE4_PS(mat[0], mat[1], mat[2], mat[3]);
			__m128 Wa = _mm_set1_ps(matF[3 * 4 + 3]);// _mm_shuffle_ps_single_index(mat[3], 3);
			__m128 Za = _mm_set1_ps(matF[3 * 4 + 2]); //_mm_shuffle_ps_single_index(mat[2], 3);
			//__m128 Zb = _mm_set1_ps(matF[0 * 4 + 2] + matF[1 * 4 + 2] + matF[2 * 4 + 2] + matF[3 * 4 + 2]); //_mm_sum4_ps_soc(mat[2]);
			__m128 Za_b = _mm_set1_ps(za_bf); //_mm_sum4_ps_soc(mat[2]);

			//__m128 Wb = _mm_set1_ps(matF[0 * 4 + 3] + matF[1 * 4 + 3] + matF[2 * 4 + 3] + matF[3 * 4 + 3]); //_mm_sum4_ps_soc(mat[3]);
			__m128 Wa_b = _mm_set1_ps(matF[0 * 4 + 3] + matF[1 * 4 + 3] + matF[2 * 4 + 3]); //_mm_sum4_ps_soc(mat[3]);
			__m128 c0 = _mm_div_ps(Za_b, Wa_b);
			//DC2: degenerate case 2
			//add zero check to remove case of Wa = Wb. This would eliminate degenerate case..
			__m128 zero_mask = _mm_cmpneq_ps(Wa_b, _mm_setzero_ps());
			c0 = _mm_and_ps(c0, zero_mask);

			occluderCache->c1 = _mm_fnmadd_ps_soc(c0, Wa, Za);
			occluderCache->c0 = c0;
		}



		auto one8th = _mm_set1_ps(0.125f);
		mat[0] = _mm_mul_ps(matT0, one8th); // scale down by 8
		mat[1] = _mm_mul_ps(matT1, one8th); // scale down by 8
		mat[3] = matT3;
	}

	// *****_MM_SHUFFLE*****
	__m128 mat33;// = _mm_shuffle_ps_single_index(occluderCache->mat[3], 3);
	__m128 mat03;// = _mm_shuffle_ps_single_index(occluderCache->mat[0], 3);
	__m128 mat13;// = _mm_shuffle_ps_single_index(occluderCache->mat[1], 3);


	{
		auto c = occluderCache->FullMeshInvExtents[1];
		mat03 = _mm_sum4_ps_soc(_mm_mul_ps(c, mat[0]));
		mat13 = _mm_sum4_ps_soc(_mm_mul_ps(c, mat[1]));
		mat33 = _mm_sum4_ps_soc(_mm_mul_ps(c, mat[3]));
		
		///Xf0 = _mm_fmadd_ps(dataArray[0], b, c);
		__m128 b = occluderCache->FullMeshInvExtents[0];
		mat[0] = _mm_mul_ps(mat[0], b);
		mat[1] = _mm_mul_ps(mat[1], b);
		mat[3] = _mm_mul_ps(mat[3], b);				
	}


	__m128  dataArray[12];

	__m128* paabbArray = (__m128*)mAABBUnitArray;

	uint8_t * pCompressIndices8 = nullptr;
	uint16_t * pCompressIndices = nullptr;
	float * pCompressVertices = nullptr; //No vertices position compression: to improve accuracy of vertices position, compression float to uint16_t is removed. 
	if (PrimitiveDataCompressed == true)
	{
		if (bSuperCompressed)
		{
			if (raw.AABBMode) {
				pCompressVertices = (float*)raw.Vertices - 14;
			}
			else {
				pCompressIndices8 = (uint8_t*)raw.Vertices;
				pCompressVertices = (float*)(pCompressIndices8 + raw.QuadSafeBatchNum * 16 + raw.TriangleBatchIdxNum * 12);
			}
		}
		else
		{
			pCompressIndices = (uint16_t *)raw.Vertices;
			pCompressVertices = (float*)(pCompressIndices +  raw.QuadSafeBatchNum * 16 + raw.TriangleBatchIdxNum * 12);
		}
	}
	
	int flip = this->mClockWise != occluderCache->FlipOccluderFace;
	if (bDrawOccludee) {
		if (occluderCache->OccludeeMesh == false) {
			flip = !this->mClockWise;
		}
	}

	//swap 0 and 2 in case of flipping
	int faceIdx0 = flip << 1;
	int faceIdx2 = 2 ^ faceIdx0;


	if (PrimitiveDataCompressed  &&raw.QuadSafeBatchNum  > 0)
	{
		//***************************
		int packetsLeft = raw.QuadSafeBatchNum;
		float* pData = (float*)dataArray;
		do {
			if (bSuperCompressed) {
				//prepare the 16 tempV
				if (raw.AABBMode) {
					__m128 extend = _mm_set1_ps(pCompressVertices[3]);
					__m128 min = _mm_set1_ps(pCompressVertices[0]);
					dataArray[0] = _mm_fmadd_ps_soc(paabbArray[0], extend, min);
					dataArray[1] = _mm_fmadd_ps_soc(paabbArray[1], extend, min);
					dataArray[2] = dataArray[1];// _mm_fmadd_ps(paabbArray[2], extend, min);
					dataArray[3] = _mm_fmadd_ps_soc(paabbArray[2], extend, min);


					extend = _mm_set1_ps(pCompressVertices[4]);
					min = _mm_set1_ps(pCompressVertices[1]);
					dataArray[4] = _mm_fmadd_ps_soc(paabbArray[3], extend, min);
					dataArray[5] = _mm_fmadd_ps_soc(paabbArray[4], extend, min);
					dataArray[6] = _mm_fmadd_ps_soc(paabbArray[5], extend, min);
					dataArray[7] = _mm_fmadd_ps_soc(paabbArray[6], extend, min);

					extend = _mm_set1_ps(pCompressVertices[5]);
					min = _mm_set1_ps(pCompressVertices[2]);
					dataArray[8] = _mm_fmadd_ps_soc(paabbArray[7], extend, min);
					dataArray[9] = dataArray[8];// _mm_fmadd_ps(paabbArray[9], extend, min);
					dataArray[10] = _mm_fmadd_ps_soc(paabbArray[8], extend, min);
					dataArray[11] = dataArray[10];// _mm_fmadd_ps(paabbArray[11], extend, min);
					paabbArray += 8;

				}
				else {
					float* a = pCompressVertices + pCompressIndices8[0];
					float* b = pCompressVertices + pCompressIndices8[4];
					float* c = pCompressVertices + pCompressIndices8[8];
					float* d = pCompressVertices + pCompressIndices8[12];

					//dataArray[0] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
					pData[0] = a[0]; pData[1] = b[0]; pData[2] = c[0]; pData[3] = d[0];
					//dataArray[4] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
					pData[16] = a[1]; pData[17] = b[1]; pData[18] = c[1]; pData[19] = d[1];
					//dataArray[8] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
					pData[32] = a[2]; pData[33] = b[2]; pData[34] = c[2]; pData[35] = d[2];



					a = pCompressVertices + pCompressIndices8[0 + 1];
					b = pCompressVertices + pCompressIndices8[4 + 1];
					c = pCompressVertices + pCompressIndices8[8 + 1];
					d = pCompressVertices + pCompressIndices8[12 + 1];

					// dataArray[3] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
					pData[0 + 12] = a[0]; pData[1 + 12] = b[0]; pData[2 + 12] = c[0]; pData[3 + 12] = d[0];
					//dataArray[7] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
					pData[16 + 12] = a[1]; pData[17 + 12] = b[1]; pData[18 + 12] = c[1]; pData[19 + 12] = d[1];
					//dataArray[11] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
					pData[32 + 12] = a[2]; pData[33 + 12] = b[2]; pData[34 + 12] = c[2]; pData[35 + 12] = d[2];


					a = pCompressVertices + pCompressIndices8[0 + 2];
					b = pCompressVertices + pCompressIndices8[4 + 2];
					c = pCompressVertices + pCompressIndices8[8 + 2];
					d = pCompressVertices + pCompressIndices8[12 + 2];

					//dataArray[2] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
					pData[0 + 8] = a[0]; pData[1 + 8] = b[0]; pData[2 + 8] = c[0]; pData[3 + 8] = d[0];
					//dataArray[6] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
					pData[16 + 8] = a[1]; pData[17 + 8] = b[1]; pData[18 + 8] = c[1]; pData[19 + 8] = d[1];
					//dataArray[10] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
					pData[32 + 8] = a[2]; pData[33 + 8] = b[2]; pData[34 + 8] = c[2]; pData[35 + 8] = d[2];

					a = pCompressVertices + pCompressIndices8[0 + 3];
					b = pCompressVertices + pCompressIndices8[4 + 3];
					c = pCompressVertices + pCompressIndices8[8 + 3];
					d = pCompressVertices + pCompressIndices8[12 + 3];

					//dataArray[1] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
					pData[0 + 4] = a[0]; pData[1 + 4] = b[0]; pData[2 + 4] = c[0]; pData[3 + 4] = d[0];
					//dataArray[5] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
					pData[16 + 4] = a[1]; pData[17 + 4] = b[1]; pData[18 + 4] = c[1]; pData[19 + 4] = d[1];
					//dataArray[9] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
					pData[32 + 4] = a[2]; pData[33 + 4] = b[2]; pData[34 + 4] = c[2]; pData[35 + 4] = d[2];


					pCompressIndices8 += 16;
				}
			}
			else {

				float* a = pCompressVertices + pCompressIndices[0];
				float* b = pCompressVertices + pCompressIndices[4];
				float* c = pCompressVertices + pCompressIndices[8];
				float* d = pCompressVertices + pCompressIndices[12];

				//dataArray[0] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
				pData[0] = a[0]; pData[1] = b[0]; pData[2] = c[0]; pData[3] = d[0];
				//dataArray[4] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
				pData[16] = a[1]; pData[17] = b[1]; pData[18] = c[1]; pData[19] = d[1];
				//dataArray[8] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
				pData[32] = a[2]; pData[33] = b[2]; pData[34] = c[2]; pData[35] = d[2];



				 a = pCompressVertices + pCompressIndices[0+1];
				 b = pCompressVertices + pCompressIndices[4 + 1];
				 c = pCompressVertices + pCompressIndices[8 + 1];
				 d = pCompressVertices + pCompressIndices[12 + 1];

				// dataArray[3] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
				 pData[0+12] = a[0]; pData[1 + 12] = b[0]; pData[2 + 12] = c[0]; pData[3 + 12] = d[0];
				 //dataArray[7] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
				 pData[16 + 12] = a[1]; pData[17 + 12] = b[1]; pData[18 + 12] = c[1]; pData[19 + 12] = d[1];
				 //dataArray[11] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
				 pData[32 + 12] = a[2]; pData[33 + 12] = b[2]; pData[34 + 12] = c[2]; pData[35 + 12] = d[2];


				 a = pCompressVertices + pCompressIndices[0 + 2];
				 b = pCompressVertices + pCompressIndices[4 + 2];
				 c = pCompressVertices + pCompressIndices[8 + 2];
				 d = pCompressVertices + pCompressIndices[12 + 2];

				 //dataArray[2] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
				 pData[0 + 8] = a[0]; pData[1 + 8] = b[0]; pData[2 + 8] = c[0]; pData[3 + 8] = d[0];
				 //dataArray[6] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
				 pData[16 + 8] = a[1]; pData[17 + 8] = b[1]; pData[18 + 8] = c[1]; pData[19 + 8] = d[1];
				 //dataArray[10] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
				 pData[32 + 8] = a[2]; pData[33 + 8] = b[2]; pData[34 + 8] = c[2]; pData[35 + 8] = d[2];

				 a = pCompressVertices + pCompressIndices[0 + 3];
				 b = pCompressVertices + pCompressIndices[4 + 3];
				 c = pCompressVertices + pCompressIndices[8 + 3];
				 d = pCompressVertices + pCompressIndices[12 + 3];

				 //dataArray[1] = _mm_setr_ps(a[0], b[0], c[0], d[0]);
				 pData[0 + 4] = a[0]; pData[1 + 4] = b[0]; pData[2 + 4] = c[0]; pData[3 + 4] = d[0];
				 //dataArray[5] = _mm_setr_ps(a[1], b[1], c[1], d[1]);
				 pData[16 + 4] = a[1]; pData[17 + 4] = b[1]; pData[18 + 4] = c[1]; pData[19 + 4] = d[1];
				 //dataArray[9] = _mm_setr_ps(a[2], b[2], c[2], d[2]);
				 pData[32 + 4] = a[2]; pData[33 + 4] = b[2]; pData[34 + 4] = c[2]; pData[35 + 4] = d[2];



				pCompressIndices += 16;
			}

			packetsLeft--;


			__m128 mat30 = _mm_set1_ps(matF[12]);//  _mm_shuffle_ps_single_index(occluderCache->mat[3], 0);
			__m128 mat31 = _mm_set1_ps(matF[13]);//_mm_shuffle_ps_single_index(occluderCache->mat[3], 1);
			__m128 mat32 = _mm_set1_ps(matF[14]);//_mm_shuffle_ps_single_index(occluderCache->mat[3], 2);

			__m128 W[4];
			W[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat30, _mm_fmadd_ps_soc(dataArray[4], mat31, _mm_fmadd_ps_soc(dataArray[8], mat32, mat33)));
			W[1] = _mm_fmadd_ps_soc(dataArray[1], mat30, _mm_fmadd_ps_soc(dataArray[5], mat31, _mm_fmadd_ps_soc(dataArray[9], mat32, mat33)));
			W[faceIdx2] = _mm_fmadd_ps_soc(dataArray[2], mat30, _mm_fmadd_ps_soc(dataArray[6], mat31, _mm_fmadd_ps_soc(dataArray[10], mat32, mat33)));
			W[3] = _mm_fmadd_ps_soc(dataArray[3], mat30, _mm_fmadd_ps_soc(dataArray[7], mat31, _mm_fmadd_ps_soc(dataArray[11], mat32, mat33)));

			__m128 primitiveValid = _mm_set1_ps(-0.0f);
			if (possiblyNearClipped)
			{
				// All W < 0 means fully culled by camera plane
				__m128 W0123 = _mm_and_ps(_mm_and_ps(_mm_and_ps(W[0], W[1]), W[2]), W[3]);
				if (_mm_same_sign1_soc(W0123))
				{
					if (DebugOccluderOccludee)
					{
						this->DebugData[P4CameraNearPlaneCull]++;
						this->DebugData[PrimitiveCameraNearPlaneCull] += 4;

						primitiveValid = _mm_xor_ps(W0123, _mm_set1_ps(-0.0f));
						DebugData[PrimitiveValidNum] = GetValidPrimitiveNum(primitiveValid);
						assert(DebugData[PrimitiveValidNum] == 0);
					}
					continue;
				}
				primitiveValid = _mm_xor_ps(W0123, primitiveValid);

				if (DebugOccluderOccludee)
				{
					DebugData[PrimitiveValidNum] = GetValidPrimitiveNum(primitiveValid);
					this->DebugData[PrimitiveCameraNearPlaneCull] += 4 - DebugData[PrimitiveValidNum];
				}
			}
			else {

				if (DebugOccluderOccludee)
				{
					DebugData[PrimitiveValidNum] = 4;
				}
			}

			__m128 X[4], Y[4];

			__m128 mat00 = _mm_set1_ps(matF[0]); //_mm_shuffle_ps_single_index(occluderCache->mat[0], 0);
			__m128 mat01 = _mm_set1_ps(matF[1]); //_mm_shuffle_ps_single_index(occluderCache->mat[0], 1);
			__m128 mat02 = _mm_set1_ps(matF[2]); //_mm_shuffle_ps_single_index(occluderCache->mat[0], 2);
			X[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat00, _mm_fmadd_ps_soc(dataArray[4], mat01, _mm_fmadd_ps_soc(dataArray[8], mat02, mat03)));
			X[1] = _mm_fmadd_ps_soc(dataArray[1], mat00, _mm_fmadd_ps_soc(dataArray[5], mat01, _mm_fmadd_ps_soc(dataArray[9], mat02, mat03)));
			X[faceIdx2] = _mm_fmadd_ps_soc(dataArray[2], mat00, _mm_fmadd_ps_soc(dataArray[6], mat01, _mm_fmadd_ps_soc(dataArray[10], mat02, mat03)));
			X[3] = _mm_fmadd_ps_soc(dataArray[3], mat00, _mm_fmadd_ps_soc(dataArray[7], mat01, _mm_fmadd_ps_soc(dataArray[11], mat02, mat03)));

			__m128 mat10 = _mm_set1_ps(matF[4]); //_mm_shuffle_ps_single_index(occluderCache->mat[1], 0);
			__m128 mat11 = _mm_set1_ps(matF[5]); //_mm_shuffle_ps_single_index(occluderCache->mat[1], 1);
			__m128 mat12 = _mm_set1_ps(matF[6]); //_mm_shuffle_ps_single_index(occluderCache->mat[1], 2);
			Y[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat10, _mm_fmadd_ps_soc(dataArray[4], mat11, _mm_fmadd_ps_soc(dataArray[8], mat12, mat13)));
			Y[1] = _mm_fmadd_ps_soc(dataArray[1], mat10, _mm_fmadd_ps_soc(dataArray[5], mat11, _mm_fmadd_ps_soc(dataArray[9], mat12, mat13)));
			Y[faceIdx2] = _mm_fmadd_ps_soc(dataArray[2], mat10, _mm_fmadd_ps_soc(dataArray[6], mat11, _mm_fmadd_ps_soc(dataArray[10], mat12, mat13)));
			Y[3] = _mm_fmadd_ps_soc(dataArray[3], mat10, _mm_fmadd_ps_soc(dataArray[7], mat11, _mm_fmadd_ps_soc(dataArray[11], mat12, mat13)));



			// Clamp W and invert
			__m128 invW[4];
			//this error might up to 1 pixel for x, and y
			invW[0] = _mm_rcp_ps_div(W[0]);
			invW[1] = _mm_rcp_ps_div(W[1]);
			invW[2] = _mm_rcp_ps_div(W[2]);
			invW[3] = _mm_rcp_ps_div(W[3]);

			bool realNearClipped = false;
			if (possiblyNearClipped)
			{
				__m128 allInfront = _mm_min_ps(_mm_min_ps(_mm_min_ps(W[0], W[1]), W[2]), W[3]);
				allInfront = _mm_cmplt_ps(allInfront, _mm_set1_ps(0.01f));

				if (!_mm_same_sign0(allInfront)) //near plane clipped
				{
					__m128 lowerBound = _mm_set1_ps(-maxInvW);
					__m128 upperBound = _mm_set1_ps(+maxInvW);

					invW[0] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[0]));
					invW[1] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[1]));
					invW[2] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[2]));
					invW[3] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[3]));

					realNearClipped = true;				
				}
			}


			X[0] = _mm_mul_ps(X[0], invW[0]);
			X[1] = _mm_mul_ps(X[1], invW[1]);
			X[2] = _mm_mul_ps(X[2], invW[2]);
			X[3] = _mm_mul_ps(X[3], invW[3]);

			Y[0] = _mm_mul_ps(Y[0], invW[0]);
			Y[1] = _mm_mul_ps(Y[1], invW[1]);
			Y[2] = _mm_mul_ps(Y[2], invW[2]);
			Y[3] = _mm_mul_ps(Y[3], invW[3]);


			if (realNearClipped) //near plane clipped
			{
				int validMask = _mm_movemask_ps(primitiveValid);
				SplitToTwoTriangles<true, bBackFaceCulling && !bDrawOccludee, bDrawOccludee>(X, Y, W, invW, primitiveValid, validMask, occluderCache);
				if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
					return;
				continue;
			}


			if (DebugOccluderOccludee)
			{
				this->DebugData[P4DrawTriangle] ++;
			}

			__m128 edgeNormalsX[4], edgeNormalsY[4];

			edgeNormalsX[0] = _mm_sub_ps(Y[1], Y[0]);
			edgeNormalsX[1] = _mm_sub_ps(Y[2], Y[1]);
			edgeNormalsX[2] = _mm_sub_ps(Y[3], Y[2]);
			edgeNormalsX[3] = _mm_sub_ps(Y[0], Y[3]);


			edgeNormalsY[0] = _mm_sub_ps(X[0], X[1]);
			edgeNormalsY[1] = _mm_sub_ps(X[1], X[2]);
			edgeNormalsY[2] = _mm_sub_ps(X[2], X[3]);
			edgeNormalsY[3] = _mm_sub_ps(X[3], X[0]);




			__m128 areas[2];
			areas[0] = _mm_fmsub_ps_soc(edgeNormalsX[0], edgeNormalsY[1], _mm_mul_ps(edgeNormalsX[1], edgeNormalsY[0]));
			areas[1] = _mm_fmsub_ps_soc(edgeNormalsX[2], edgeNormalsY[3], _mm_mul_ps(edgeNormalsX[3], edgeNormalsY[2]));

			__m128 minArea = _mm_min_ps(areas[0], areas[1]);
			{

				//query occludee, force split to increase accuracy
				if (bDrawOccludee && bSplitQuadToTriangleForOccludee)
				{
					if (bBackFaceCulling == true)
					{
						//Apply backface culling, reject the quad if both triangles' area < 0
						__m128 anyPositive = _mm_cmpgt_ps(_mm_max_ps(areas[0], areas[1]), _mm_set1_ps(0.0f));
						primitiveValid = _mm_and_ps(anyPositive, primitiveValid);

						
					}
					int validMask = _mm_movemask_ps(primitiveValid);
					if (bBackFaceCulling && validMask == 0) //all negative
					{
						continue;
					}
					SplitToTwoTriangles<false, bBackFaceCulling, bDrawOccludee>(X, Y, W, invW, primitiveValid, validMask, occluderCache);
					if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
						return;
					continue;
				}
				//must go through convex check even in planar case due to numeric error
				//the convex check would guarantee that points does not overlap
				if (bBackFaceCulling == true)
				{
					//Apply backface culling, reject the quad if both triangles' area < 0
					__m128 anyPositive = _mm_cmpgt_ps(_mm_max_ps(areas[0], areas[1]), _mm_set1_ps(0.0f));
					primitiveValid = _mm_and_ps(anyPositive, primitiveValid);


					int validMask = _mm_movemask_ps(primitiveValid);
					if (validMask == 0) //all negative
					{
						continue;
					}
					//need to sync primitive valid state here
					if (DebugOccluderOccludee)
					{
						int active = GetValidPrimitiveNum(primitiveValid);
						this->DebugData[PrimitiveBackFaceCull] += (DebugData[PrimitiveValidNum] - active) * 2;
						DebugData[PrimitiveValidNum] = active;
					}

					{
						//check concave scenario: if any triangle of four triangles formed by the four points has negative area
						//the convex property is violated. going for safe way, Triangle Approach
						__m128 area3 = _mm_fmsub_ps_soc(edgeNormalsX[1], edgeNormalsY[2], _mm_mul_ps(edgeNormalsX[2], edgeNormalsY[1]));
						__m128 area4 = _mm_sub_ps(_mm_add_ps(areas[0], areas[1]), area3);

						//DC3 degenerate case 3: for the case of any of area1/area2 is zero,
						//normalizeEdge would trigger divide by zero degeneracy
						//fix: treat as concave if any of four triangle is non-positive
						__m128 minArea4 = _mm_min_ps(_mm_min_ps(area4, area3), minArea);
						__m128 concaveQuad = _mm_cmple_ps(minArea4, _mm_set1_ps(1e-10f));

						concaveQuad = _mm_and_ps(concaveQuad, primitiveValid);//any active primitive is concave
						if (_mm_same_sign0(concaveQuad) == false)  //at least one active primitive is concave
						{
							SplitToTwoTriangles<false, bBackFaceCulling, bDrawOccludee>(X, Y, W, invW, primitiveValid, validMask, occluderCache);
							if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
								return;
							continue;
						}
					}
				}
				else
				{
					//even in backface cull off state, if any area is negative, go triangle approach
					__m128 area3 = _mm_fmsub_ps_soc(edgeNormalsX[1], edgeNormalsY[2], _mm_mul_ps(edgeNormalsX[2], edgeNormalsY[1]));
					__m128 area4 = _mm_sub_ps(_mm_add_ps(areas[0], areas[1]), area3);

					__m128 minArea4 = _mm_min_ps(_mm_min_ps(area4, area3), minArea);
					__m128 concaveQuad = _mm_cmple_ps(minArea4, _mm_set1_ps(1e-10f));

					//in backface cull off mode
					//todo: possible optimization, in case all valid negative, swap could still continue the quad approach
					concaveQuad = _mm_and_ps(concaveQuad, primitiveValid);//any active primitive is concave
					if (_mm_same_sign0(concaveQuad) == false)  //at least one active primitive is concave
					{
						int validMask = _mm_movemask_ps(primitiveValid);
						SplitToTwoTriangles<false, bBackFaceCulling, bDrawOccludee>(X, Y, W, invW, primitiveValid, validMask, occluderCache);
						if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
							return;
						continue;
					}
				}
				//all positive
			}



			if (DebugOccluderOccludee) {
				DebugData[QuadProcessed] ++;
			}

			////must be positive faces here now
			drawQuad<true, bDrawOccludee>(X, Y, invW, W, primitiveValid, edgeNormalsX, edgeNormalsY, areas, occluderCache);
			
			if (bDrawOccludee) {
				if (occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap) {
					return;
				}
			}
		} while (packetsLeft > 0);
	}
#pragma region TrisProcessing

	const float * triangleData[12];
	if(raw.TriangleBatchIdxNum > 0)
	{
		int triPacketCount = 0;


		const uint16_t* pIndexCurrent = nullptr;
		int faceNum = 0; //total triangle num for the next round
		if (PrimitiveDataCompressed == true)
		{
			triPacketCount = raw.TriangleBatchIdxNum;
		}
		else {
			faceNum = raw.TriangleBatchIdxNum / 3;
			triPacketCount = faceNum >> 2; //first round P4 number
			pIndexCurrent = raw.Indices;
		}

		do
		{
			if (PrimitiveDataCompressed == false)
			{
				if (faceNum >= 4)
				{
					faceNum = faceNum & 3;
				}
				else //the end round
				{
					mIndexBuffer[0] = mIndexBuffer[1] = mIndexBuffer[2] = 0;

					uint16_t* temp = (uint16_t*)mIndexBuffer;
					memcpy(temp, pIndexCurrent, faceNum * (3 * sizeof(uint16_t)));

					pIndexCurrent = temp;
					triPacketCount = 0; //0 or 1 both work, set to zero as zero compare is faster
					faceNum = 0;
				}
			}

			int packetIdx = 0;
			float* pData = (float*)dataArray;
			do{
				//__m128 Xf0, Xf1, Xf2;
				//__m128 Yf0, Yf1, Yf2;
				//__m128 Zf0, Zf1, Zf2;
				if (PrimitiveDataCompressed == false)
				{	
					bool normalMesh = true;
					if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER) {
						normalMesh = occluderCache->OccludeeMesh;
						if (normalMesh == false) {
							//prepare the 16 tempV
							triangleData[0] = raw.Vertices + pIndexCurrent[0];
							triangleData[1] = raw.Vertices + pIndexCurrent[2];
							triangleData[2] = raw.Vertices + pIndexCurrent[1];
							triangleData[3] = raw.Vertices + pIndexCurrent[3];
							triangleData[3 + 1] = raw.Vertices + pIndexCurrent[5];
							triangleData[3 + 2] = raw.Vertices + pIndexCurrent[4];
							triangleData[6] = raw.Vertices + pIndexCurrent[6];
							triangleData[6 + 1] = raw.Vertices + pIndexCurrent[8];
							triangleData[6 + 2] = raw.Vertices + pIndexCurrent[7];
							triangleData[9] = raw.Vertices + pIndexCurrent[9];
							triangleData[9 + 1] = raw.Vertices + pIndexCurrent[11];
							triangleData[9 + 2] = raw.Vertices + pIndexCurrent[10];
						}
					}
					if(normalMesh){
						//prepare the 16 tempV
						triangleData[0] = raw.Vertices + pIndexCurrent[0] * 3;
						triangleData[1] = raw.Vertices + pIndexCurrent[2] * 3;
						triangleData[2] = raw.Vertices + pIndexCurrent[1] * 3;
						triangleData[3] = raw.Vertices + pIndexCurrent[3] * 3;
						triangleData[3 + 1] = raw.Vertices + pIndexCurrent[5] * 3;
						triangleData[3 + 2] = raw.Vertices + pIndexCurrent[4] * 3;
						triangleData[6] = raw.Vertices + pIndexCurrent[6] * 3;
						triangleData[6 + 1] = raw.Vertices + pIndexCurrent[8] * 3;
						triangleData[6 + 2] = raw.Vertices + pIndexCurrent[7] * 3;
						triangleData[9] = raw.Vertices + pIndexCurrent[9] * 3;
						triangleData[9 + 1] = raw.Vertices + pIndexCurrent[11] * 3;
						triangleData[9 + 2] = raw.Vertices + pIndexCurrent[10] * 3;
					}
					pIndexCurrent += 12;

						float * dataf = (float*)dataArray;
						Transpose(triangleData, dataf);
						Transpose(triangleData + 2, dataf + 8);
						Transpose(triangleData + 1, dataf + 4);
				}
				else if (bSuperCompressed) {

					float* a = pCompressVertices + pCompressIndices8[0];
					float* b = pCompressVertices + pCompressIndices8[3];
					float* c = pCompressVertices + pCompressIndices8[6];
					float* d = pCompressVertices + pCompressIndices8[9];

					pData[0] = a[0]; pData[1] = b[0]; pData[2] = c[0]; pData[3] = d[0];
					pData[12] = a[1]; pData[13] = b[1]; pData[14] = c[1]; pData[15] = d[1];
					pData[24] = a[2]; pData[25] = b[2]; pData[26] = c[2]; pData[27] = d[2];

					a = pCompressVertices + pCompressIndices8[1];
					b = pCompressVertices + pCompressIndices8[4];
					c = pCompressVertices + pCompressIndices8[7];
					d = pCompressVertices + pCompressIndices8[10];

					pData[8] = a[0]; pData[8 + 1] = b[0]; pData[8 + 2] = c[0]; pData[8 + 3] = d[0];
					pData[20] = a[1]; pData[20 + 1] = b[1]; pData[20 + 2] = c[1]; pData[20 + 3] = d[1];
					pData[32] = a[2]; pData[32 + 1] = b[2]; pData[32 + 2] = c[2]; pData[32 + 3] = d[2];

					a = pCompressVertices + pCompressIndices8[2];
					b = pCompressVertices + pCompressIndices8[5];
					c = pCompressVertices + pCompressIndices8[8];
					d = pCompressVertices + pCompressIndices8[11];

					pData[4] = a[0]; pData[4 + 1] = b[0]; pData[4 + 2] = c[0]; pData[4 + 3] = d[0];
					pData[16] = a[1]; pData[16 + 1] = b[1]; pData[16 + 2] = c[1]; pData[16 + 3] = d[1];
					pData[28] = a[2]; pData[28 + 1] = b[2]; pData[28 + 2] = c[2]; pData[28 + 3] = d[2];

					pCompressIndices8 += 12;
				}
				else
				{
					float* a = pCompressVertices + pCompressIndices[0];
					float* b = pCompressVertices + pCompressIndices[3];
					float* c = pCompressVertices + pCompressIndices[6];
					float* d = pCompressVertices + pCompressIndices[9];

					pData[0] = a[0]; pData[1] = b[0]; pData[2] = c[0]; pData[3] = d[0];
					pData[12] = a[1]; pData[13] = b[1]; pData[14] = c[1]; pData[15] = d[1];
					pData[24] = a[2]; pData[25] = b[2]; pData[26] = c[2]; pData[27] = d[2];

					a = pCompressVertices + pCompressIndices[1];
					b = pCompressVertices + pCompressIndices[4];
					c = pCompressVertices + pCompressIndices[7];
					d = pCompressVertices + pCompressIndices[10];

					pData[8] = a[0]; pData[8+1] = b[0]; pData[8 + 2] = c[0]; pData[8 + 3] = d[0];
					pData[20] = a[1]; pData[20 + 1] = b[1]; pData[20 + 2] = c[1]; pData[20 + 3] = d[1];
					pData[32] = a[2]; pData[32 + 1] = b[2]; pData[32 + 2] = c[2]; pData[32 + 3] = d[2];

					a = pCompressVertices + pCompressIndices[2];
					b = pCompressVertices + pCompressIndices[5];
					c = pCompressVertices + pCompressIndices[8];
					d = pCompressVertices + pCompressIndices[11];

					pData[4] = a[0]; pData[4 + 1] = b[0]; pData[4 + 2] = c[0]; pData[4 + 3] = d[0];
					pData[16] = a[1]; pData[16 + 1] = b[1]; pData[16 + 2] = c[1]; pData[16 + 3] = d[1];
					pData[28] = a[2]; pData[28 + 1] = b[2]; pData[28 + 2] = c[2]; pData[28 + 3] = d[2];

					pCompressIndices += 12;
				}
				

				if (DebugOccluderOccludee)
				{
					this->DebugData[P4Total]++;
				}

				packetIdx++;

				__m128 mat30 = _mm_set1_ps(matF[12]);// _mm_shuffle_ps_single_index(occluderCache->mat[3], 0);
				__m128 mat31 = _mm_set1_ps(matF[13]);// _mm_shuffle_ps_single_index(occluderCache->mat[3], 1);
				__m128 mat32 = _mm_set1_ps(matF[14]);// _mm_shuffle_ps_single_index(occluderCache->mat[3], 2);

				__m128 W[3];
				W[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat30, _mm_fmadd_ps_soc(dataArray[3], mat31, _mm_fmadd_ps_soc(dataArray[6], mat32, mat33)));
				W[1] = _mm_fmadd_ps_soc(dataArray[2], mat30, _mm_fmadd_ps_soc(dataArray[5], mat31, _mm_fmadd_ps_soc(dataArray[8], mat32, mat33)));
				W[faceIdx2] = _mm_fmadd_ps_soc(dataArray[1], mat30, _mm_fmadd_ps_soc(dataArray[4], mat31, _mm_fmadd_ps_soc(dataArray[7], mat32, mat33)));


				if (DebugOccluderOccludee)
				{
					this->DebugData[PrimitiveTotalInput]+=4;
					this->DebugData[P4NearClipInput] += (int)possiblyNearClipped << 2;
				}
				__m128 W0W1W2;
				if (possiblyNearClipped)
				{
					// All W < 0 means fully culled by camera plane
					W0W1W2 = _mm_and_ps(_mm_and_ps(W[0], W[1]), (W[2]));
					if (_mm_same_sign1_soc(W0W1W2))
					{
						if (DebugOccluderOccludee)
						{
							this->DebugData[P4CameraNearPlaneCull]++;
							this->DebugData[PrimitiveCameraNearPlaneCull] += 4;	

						}
						continue;
					}
				}
				else {
					if (DebugOccluderOccludee)
					{
						DebugData[PrimitiveValidNum] = 4;
					}
				}

				__m128 primitiveValid = _mm_set1_ps(-0.0f);
				if (possiblyNearClipped)
				{
					primitiveValid = _mm_xor_ps(W0W1W2, _mm_set1_ps(-0.0f));

					if (DebugOccluderOccludee)
					{
						DebugData[PrimitiveValidNum] = GetValidPrimitiveNum(primitiveValid);
						this->DebugData[PrimitiveCameraNearPlaneCull] += 4 - DebugData[PrimitiveValidNum];
					}
				}

				__m128 X[3], Y[3];

				__m128 mat00 = _mm_set1_ps(matF[0]);// _mm_shuffle_ps_single_index(occluderCache->mat[0], 0);
				__m128 mat01 = _mm_set1_ps(matF[1]);//_mm_shuffle_ps_single_index(occluderCache->mat[0], 1);
				__m128 mat02 = _mm_set1_ps(matF[2]);//_mm_shuffle_ps_single_index(occluderCache->mat[0], 2);
				X[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat00, _mm_fmadd_ps_soc(dataArray[3], mat01, _mm_fmadd_ps_soc(dataArray[6], mat02, mat03)));
				X[1] = _mm_fmadd_ps_soc(dataArray[2], mat00, _mm_fmadd_ps_soc(dataArray[5], mat01, _mm_fmadd_ps_soc(dataArray[8], mat02, mat03)));
				X[faceIdx2] = _mm_fmadd_ps_soc(dataArray[1], mat00, _mm_fmadd_ps_soc(dataArray[4], mat01, _mm_fmadd_ps_soc(dataArray[7], mat02, mat03)));


				__m128 mat10 = _mm_set1_ps(matF[4]);//_mm_shuffle_ps_single_index(occluderCache->mat[1], 0);
				__m128 mat11 = _mm_set1_ps(matF[5]);//_mm_shuffle_ps_single_index(occluderCache->mat[1], 1);
				__m128 mat12 = _mm_set1_ps(matF[6]);//_mm_shuffle_ps_single_index(occluderCache->mat[1], 2);
				Y[faceIdx0] = _mm_fmadd_ps_soc(dataArray[0], mat10, _mm_fmadd_ps_soc(dataArray[3], mat11, _mm_fmadd_ps_soc(dataArray[6], mat12, mat13)));
				Y[1] = _mm_fmadd_ps_soc(dataArray[2], mat10, _mm_fmadd_ps_soc(dataArray[5], mat11, _mm_fmadd_ps_soc(dataArray[8], mat12, mat13)));
				Y[faceIdx2] = _mm_fmadd_ps_soc(dataArray[1], mat10, _mm_fmadd_ps_soc(dataArray[4], mat11, _mm_fmadd_ps_soc(dataArray[7], mat12, mat13)));



				// Clamp W and invert
				__m128 invW[3];

				{
					//StressTest. Input junk value to make it does not crash
					//if (possiblyNearClipped)
					//{
					//	W[0] = _mm_set1_ps( 1.17549435082e-38);
					//	W[1] = _mm_set1_ps(1);
					//	W[2] = _mm_set1_ps(1);
					//}

					//this error might up to 1 pixel for x, and y
					invW[0] = _mm_rcp_ps_div(W[0]);
					invW[1] = _mm_rcp_ps_div(W[1]);
					invW[2] = _mm_rcp_ps_div(W[2]);
				}
				bool treatNearClip = false;
				if (possiblyNearClipped) 
				{
					__m128 allInfront = _mm_min_ps(_mm_min_ps(W[0], W[1]), W[2]);
					allInfront = _mm_cmplt_ps(allInfront, _mm_set1_ps(0.01f));

					if (!_mm_same_sign0(allInfront)) //near plane clipped
					{
						__m128 lowerBound = _mm_set1_ps(-maxInvW);
						__m128 upperBound = _mm_set1_ps(+maxInvW);

						invW[0] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[0]));
						invW[1] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[1]));
						invW[2] = _mm_min_ps(upperBound, _mm_max_ps(lowerBound, invW[2]));
						treatNearClip = true;
					}
				}

				X[0] = _mm_mul_ps(X[0], invW[0]);
				X[1] = _mm_mul_ps(X[1], invW[1]);
				X[2] = _mm_mul_ps(X[2], invW[2]);

				Y[0] = _mm_mul_ps(Y[0], invW[0]);
				Y[1] = _mm_mul_ps(Y[1], invW[1]);
				Y[2] = _mm_mul_ps(Y[2], invW[2]);

			

				if (DebugOccluderOccludee)
				{
					this->DebugData[P4DrawTriangle] ++;
					this->DebugData[BlockPacketID] = packetIdx;
				}

				if (possiblyNearClipped)
				{
				
					if (treatNearClip) {
						drawTriangle< true, bBackFaceCulling && !bDrawOccludee, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);
					}
					else {
						drawTriangle< false, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);
					}
				}
				else {
					drawTriangle< false, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);
				}
				if (bDrawOccludee) {
					if (occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap) {
						return ;
					}
				}


			}while (packetIdx < triPacketCount);

			if (PrimitiveDataCompressed == true) 
			{
				return;
			}
		}while (faceNum != 0);
	}
#pragma endregion TrisProcessing
}

#if defined( SUPPORT_ALL_FEATURE)
static void StoreBlock(uint64_t t0, int blockIdx, uint8_t * target, uint8_t value)
{
	if (bDumpBlockColumnImage == false) return;
	int startX = blockIdx * 16  ;

	for (int by = 0; by < 8; by++)
	{
		int pixelIdx = startX + (by * 80);
		unsigned char *dest = target + pixelIdx;
		for (int bx = 0; bx < 8; bx++)
		{
			//mapping pixel (x, y) to 64 bit
			uint64_t check = (t0 >> (8 * bx + 7 - by)) & 1;
			if (check > 0) {
				dest[0] = std::max<uint8_t>(value, dest[0]);
			}
			else 
			{
				dest[0] = std::max<uint8_t>(127, dest[0]); 
			}
			dest++;
		}
	}
}
#endif

#if defined( SUPPORT_ALL_FEATURE)
static int mCurrentDebugMaskCount = 0;
static int mMaxBlockRowDump = 100;
void Rasterizer::DumpColumnBlock(uint64_t t0, uint64_t t1, uint64_t t2, uint16_t maxDepth, uint64_t blockMask)
{
	if (bDumpBlockColumnImage == false) return;
	int maskDumpWith = 80;
	int maskDumpHeight = 16 * mMaxBlockRowDump; //store max 100 row data //one row data + one row black space region
	static std::vector<uint8_t> buffer;
	if (buffer.size() == 0) {
		buffer.resize(maskDumpWith * maskDumpHeight); //only save r,g channel and b = r&g
		memset(&buffer[0], 0, buffer.size());
	}
	uint8_t *target = &buffer[0];

	target += mCurrentDebugMaskCount * 16 * maskDumpWith;
	mCurrentDebugMaskCount++;
	if (mCurrentDebugMaskCount >= mMaxBlockRowDump)
	{
		return;
	}


	StoreBlock(t0, 0, target, maxDepth >> 8);
	StoreBlock(t1, 1, target, maxDepth >> 8);
	StoreBlock(t2, 2, target, maxDepth >> 8);
	StoreBlock(t0 & t1&t2, 3, target, maxDepth >> 8);
	StoreBlock(blockMask, 4, target, 255);


	target = &buffer[0];
	std::string str = std::to_string(mCurrentDebugMaskCount);
	while (str.length() < 3) str = "0" + str;
	const std::string filename = "../../GOLDEN_DATA/all///Output/triangleNumber_" + str + "_Mask.pgm";
	{
		std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
		ofs << "P5\n" << maskDumpWith << " " << maskDumpHeight << "\n255\n";
		for (int ny = 0; ny < maskDumpHeight; ny++)
		{
			for (int nx = 0; nx < maskDumpWith; nx++)
			{
				int idx = nx + (maskDumpHeight - 1 - ny) * maskDumpWith;
				ofs << target[idx];
			}
		}
		ofs.close();
		LOGI("dumpGrayImage %s", filename.c_str());
	}


	uint8_t * img = new uint8_t[this->m_totalPixels * 2];
	const std::string filenameImage = "../../GOLDEN_DATA/all///Output/triangleNumber_" + str + "_Full.pgm";
	{
		readBackDepth(img, DumpFull);
		std::ofstream ofs(filenameImage, std::ios_base::out | std::ios_base::binary);
		ofs << "P5\n" << this->m_width << " " << this->m_height << "\n255\n";
		for (uint32_t ny = 0; ny < this->m_height; ny++)
		{
			for (uint32_t nx = 0; nx < this->m_width; nx++)
			{
				int idx = nx + (this->m_height - 1 - ny) * this->m_width;
				ofs << img[idx];
			}
		}
		ofs.close();
		LOGI("dumpFullImage %s", filenameImage.c_str());
	}
	delete[] img;
}
#endif



#if defined( SUPPORT_ALL_FEATURE)
void Rasterizer::updateBlockWithMaxZ(__m128 rowDepthLeft, __m128 rowDepthRight, uint64_t blockMask, __m128 depthRowDelta, __m128i primitiveMaxZV, __m128i * out, uint16_t * pBlockRowHiZ)
{
	if (VRS_X4Y4_Optimzation)
	{
		return;
	}


	if (blockMask != -1)
	{
		__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);

		if (pBlockRowHiZ[0] == 0)
		{
			pBlockRowHiZ[0] = MIN_UPDATED_BLOCK_DEPTH;

			for (uint32_t i = 0; i < 7; ++i)
			{
				auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);

				__m128i rowMask = _mm_srai_epi16(interleavedBlockMask, 15);
				out[i] =  _mm_and_si128(rowMask, current);


				rowDepthLeft = _mm_add_ps(rowDepthLeft, depthRowDelta);
				rowDepthRight = _mm_add_ps(rowDepthRight, depthRowDelta);
				interleavedBlockMask = _mm_add_epi16(interleavedBlockMask, interleavedBlockMask);
			}
			__m128i rowMask = _mm_srai_epi16(interleavedBlockMask, 15);


			auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);
			out[7] = _mm_and_si128(rowMask, current);


			return; // init-partial update, hizMin is surely zero. No need to calculate
		}
		else
		{
			for (uint32_t i = 0; i < 7; ++i)
			{
				auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);

				__m128i rowMask = _mm_srai_epi16(interleavedBlockMask, 15);
				out[i] = _mm_max_epu16(out[i], _mm_and_si128(rowMask, current));


				rowDepthLeft = _mm_add_ps(rowDepthLeft, depthRowDelta);
				rowDepthRight = _mm_add_ps(rowDepthRight, depthRowDelta);
				interleavedBlockMask = _mm_add_epi16(interleavedBlockMask, interleavedBlockMask);
			}
			__m128i rowMask = _mm_srai_epi16(interleavedBlockMask, 15);


			auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);
			out[7] = _mm_max_epu16(out[7], _mm_and_si128(rowMask, current));
		}
	}
	else
	{
		if (pBlockRowHiZ[0] == 0)
		{
			for (uint32_t i = 0; i < 7; ++i)
			{
				out[i] = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);

				rowDepthLeft = _mm_add_ps(rowDepthLeft, depthRowDelta);
				rowDepthRight = _mm_add_ps(rowDepthRight, depthRowDelta);
			}

			out[7] = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);

			pBlockRowHiZ[0] = _mm_min_epu16(_mm_min_epu16(out[0], out[7])); //initial full block, check first & last row
			return;
		}
		else {
			////keep original for reference									
			for (uint32_t i = 0; i < 7; ++i)
			{
				auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);

				out[i] = _mm_max_epu16(out[i], current);

				rowDepthLeft = _mm_add_ps(rowDepthLeft, depthRowDelta);
				rowDepthRight = _mm_add_ps(rowDepthRight, depthRowDelta);
			}

			auto current = packDepthPremultiplied(rowDepthLeft, rowDepthRight, primitiveMaxZV);
			out[7] = _mm_max_epu16(out[7], current);
		}
	}


		__m128i newMinZ0 = _mm_min_epu16(out[0], out[1]);
		__m128i newMinZ2 = _mm_min_epu16(out[2], out[3]);
		__m128i newMinZ4 = _mm_min_epu16(out[4], out[5]);
		__m128i newMinZ6 = _mm_min_epu16(out[6], out[7]);
		__m128i newMinZ = _mm_min_epu16(_mm_min_epu16(newMinZ0, newMinZ2), _mm_min_epu16(newMinZ4, newMinZ6));
		uint16_t result = _mm_min_epu16(newMinZ);
		pBlockRowHiZ[0] = std::max<uint16_t>(MIN_UPDATED_BLOCK_DEPTH, result);
}

#endif
void Rasterizer::updateBlockMSCBPartial_Occludee(uint32_t* depth32, uint64_t blockMask, __m128i* out, OccluderRenderCache * cache)
{
#if defined(SDOC_NATIVE)
	if (false) {
		__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
		__m128i	depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
		__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);
		__m128i out0 = out[0];
		__m128i new0 = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom);
		__m128i visible = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[0], depthBottom));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		bool A = _mm_anybit_one_soc(visible);
		__m128i out1 = out[1];
		__m128i new1 = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom);
		visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[1], depthBottom)));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		bool B = _mm_anybit_one_soc(visible);
		__m128i out2 = out[2];
		__m128i new2 = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom);
		visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[2], depthTop)));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		bool C = _mm_anybit_one_soc(visible);
		__m128i out3 = out[3];
		__m128i new3 = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom);
		visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[3], depthTop)));
		cache->mRasterizedOccludeeVisible = _mm_anybit_one_soc(visible);
		if (cache->mRasterizedOccludeeVisible) {
			return;
		}
		return;
	}
#endif

	__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
	__m128i	depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
	__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);
	__m128i visible = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[0], depthBottom));
	interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
	visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[1], depthBottom)));
	interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
	visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[2], depthTop)));
	interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
	visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[3], depthTop)));
	cache->mRasterizedOccludeeVisible = _mm_anybit_one_soc(visible);
}

void Rasterizer::updateBlockMSCBPartial(uint32_t * depth32, uint64_t blockMask, __m128i* out, uint64_t* maskData, uint16_t * pBlockRowHiZ, uint16_t maxBlockDepth)
{
	mUpdateAnyBlock = true;
	__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
	__m128i	depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);

	if (pBlockRowHiZ[0] != 0) 
	{
		__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);

		out[0] = _mm_max_epu16(out[0], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[1] = _mm_max_epu16(out[1], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[2] = _mm_max_epu16(out[2], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop));
		interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[3] = _mm_max_epu16(out[3], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop));

		if (PairBlockNum == CheckerBoardVizMaskApproach) 
		{
			maskData[0] |= blockMask;
			if (maskData[0] == -1)
			{
				__m128i newMinZ0 = _mm_min_epu16(out[0], out[1]);
				__m128i newMinZ2 = _mm_min_epu16(out[2], out[3]);
				pBlockRowHiZ[0] = _mm_min_epu16(_mm_min_epu16(newMinZ0, newMinZ2));
				if (DebugOccluderOccludee) {
					DebugData[BlockMinCompute4]++;
				}
			}
		}
		else {
			__m128i newMinZ0 = _mm_min_epu16(out[0], out[1]);
			__m128i newMinZ2 = _mm_min_epu16(out[2], out[3]);
			pBlockRowHiZ[0] = _mm_min_epu16(_mm_min_epu16(newMinZ0, newMinZ2)) | MIN_UPDATED_BLOCK_DEPTH;
			if (DebugOccluderOccludee) {
				DebugData[BlockMinCompute4]++;
			}
		}
		uint16_t * pBlockRowHiZMax = pBlockRowHiZ + m_HizBufferSize;
		pBlockRowHiZMax[0] = std::max<uint16_t>(maxBlockDepth, pBlockRowHiZMax[0]);
	}
	else 
	{
		if (DebugOccluderOccludee) {
			DebugData[BlockRenderInitial]++;
			DebugData[BlockRenderInitialPartial]++;
		}
		if (PairBlockNum == CheckerBoardVizMaskApproach) 
		{
			maskData[0] = blockMask;
		}

		pBlockRowHiZ[m_HizBufferSize] = maxBlockDepth;
		pBlockRowHiZ[0] = MIN_UPDATED_BLOCK_DEPTH;
		__m128i interleavedBlockMask;
		interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);

		out[0] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[1] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[2] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 2);
		out[3] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop);
		return; // init-partial update, hizMin is surely zero. No need to calculate				
	}

}

#if defined( SUPPORT_ALL_FEATURE)
void Rasterizer::updateBlock(__m128i* depthRows, uint64_t blockMask,   __m128i * out, uint16_t * pBlockRowHiZ)
{
	if (PairBlockNum != FullBlockApproach) {
		return;
	}
	this->mUpdateAnyBlock = true;
	{
		__m128i depthBottom = depthRows[0];
		__m128i depthTop = depthRows[1];
		if (blockMask != -1)
		{
			__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);
			if (pBlockRowHiZ[0] == 0) {

				pBlockRowHiZ[0] = MIN_UPDATED_BLOCK_DEPTH;


				out[0] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[1] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[2] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[3] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[4] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[5] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[6] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[7] = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop);
				return; // init-partial update, hizMin is surely zero. No need to calculate
			}
			else {

				out[0] = _mm_max_epu16(out[0], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[1] = _mm_max_epu16(out[1], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[2] = _mm_max_epu16(out[2], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[3] = _mm_max_epu16(out[3], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthBottom)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[4] = _mm_max_epu16(out[4], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[5] = _mm_max_epu16(out[5], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[6] = _mm_max_epu16(out[6], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				out[7] = _mm_max_epu16(out[7], _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), depthTop));
			}



		}
		else
		{

			////keep original for reference

			// All pixels covered => skip edge tests
			if (pBlockRowHiZ[0] == 0)
			{
				out[0] = depthBottom;
				out[1] = depthBottom;
				out[2] = depthBottom;
				out[3] = depthBottom;
				out[4] = depthTop;
				out[5] = depthTop;
				out[6] = depthTop;
				out[7] = depthTop;
				pBlockRowHiZ[0] = _mm_min_epu16(_mm_min_epu16(depthBottom, depthTop));
				return;
			}
			else
			{
				out[0] = _mm_max_epu16(out[0], depthBottom);
				out[1] = _mm_max_epu16(out[1], depthBottom);
				out[2] = _mm_max_epu16(out[2], depthBottom);
				out[3] = _mm_max_epu16(out[3], depthBottom);
				out[4] = _mm_max_epu16(out[4], depthTop);
				out[5] = _mm_max_epu16(out[5], depthTop);
				out[6] = _mm_max_epu16(out[6], depthTop);
				out[7] = _mm_max_epu16(out[7], depthTop);
			}
		}
		{
			__m128i newMinZ0 = _mm_min_epu16(out[0], out[1]);
			__m128i newMinZ2 = _mm_min_epu16(out[2], out[3]);
			__m128i newMinZ4 = _mm_min_epu16(out[4], out[5]);
			__m128i newMinZ6 = _mm_min_epu16(out[6], out[7]);
			__m128i newMinZ = _mm_min_epu16(_mm_min_epu16(newMinZ0, newMinZ2), _mm_min_epu16(newMinZ4, newMinZ6));
			uint16_t result = _mm_min_epu16(newMinZ);
			pBlockRowHiZ[0] = std::max<uint16_t>(MIN_UPDATED_BLOCK_DEPTH, result);

		}
	}
	
}
bool Rasterizer::updateBlock_Occludee(__m128i* depthRows, uint64_t blockMask, __m128i* out, uint16_t* pBlockRowHiZ)
{
	if (PairBlockNum != FullBlockApproach) {
		return false;
	}
	{
		__m128i depthBottom = depthRows[0];
		__m128i depthTop = depthRows[1];
		if (blockMask != -1)
		{
			__m128i interleavedBlockMask = _mm_unpacklo_epi8_soc(blockMask);
			if (pBlockRowHiZ[0] == 0) {
				return true; // init-partial update, hizMin is surely zero. No need to calculate
			}
			else {
				__m128i visible = _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[0],  depthBottom)); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[1], depthBottom))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[2], depthBottom))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[3], depthBottom))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[4], depthTop))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[5], depthTop))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[6], depthTop))); interleavedBlockMask = _mm_slli_epi16(interleavedBlockMask, 1);
				visible = _mm_or_si128(visible, _mm_and_si128(_mm_srai_epi16(interleavedBlockMask, 15), _mm_cmple_epu16_soc(out[7], depthTop)));
				if (_mm_anybit_one_soc(visible)) {
					return true;
				}
			}
		}
		else
		{
			////keep original for reference
			// All pixels covered => skip edge tests
			if (pBlockRowHiZ[0] == 0)
			{
				return true;
			}
			else
			{
				__m128i visible = _mm_cmple_epu16_soc(out[0], depthBottom);
				visible = _mm_cmple_epu16_soc(out[1], depthBottom);
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[2], depthBottom));
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[3], depthBottom));
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[4], depthTop));
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[5], depthTop));
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[6], depthTop));
				visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[7], depthTop));
				if (_mm_anybit_one_soc(visible)) {
					return true;
				}
			}
		}
	}
	return false;
}

#endif

template <bool possiblyNearClipped, bool bBackFaceCulling, bool bDrawOccludee>
void Rasterizer::drawTriangle( __m128* x, __m128* y, __m128* invW, __m128* W,  __m128 primitiveValid, OccluderRenderCache* occluderCache)
{


	PrimitiveBoundaryClipCache* PrimitivePixelClip = &occluderCache->mPixelClip;

	__m128 edgeNormalsX[3], edgeNormalsY[3];
	edgeNormalsX[0] = _mm_sub_ps(y[1], y[0]);
	edgeNormalsX[1] = _mm_sub_ps(y[2], y[1]);


	edgeNormalsY[0] = _mm_sub_ps(x[0], x[1]);
	edgeNormalsY[1] = _mm_sub_ps(x[1], x[2]);


	// Area and backface culling
	__m128 negativeArea = _mm_fmsub_ps_soc(edgeNormalsX[1], edgeNormalsY[0], _mm_mul_ps(edgeNormalsX[0], edgeNormalsY[1])); //negative negativeArea



	if (bBackFaceCulling == false)
	{
		__m128 swapMask;
		bool needRearrangeTriangle = false;
		// Need to flip back faceNum test for each W < 0
		if (possiblyNearClipped)
		{
			__m128i areaCheckMask = _mm_set1_epi32(0x7fffffff);

			//area threshold change to 0x7f000000
			//previous setting is 0x53800000 1.09951162778e+12
			__m128i areaThreshold = _mm_set1_epi32(0x7f000000);
			__m128i checkArea = _mm_and_si128(_mm_castps_si128(negativeArea), areaCheckMask); //after this, valid Mask is positive Area
			__m128i validMask = _mm_cmplt_epi32(checkArea, areaThreshold);

			primitiveValid = _mm_and_ps(_mm_castsi128_ps(validMask), primitiveValid);

			//flip negativeArea sign if any W is negative
			__m128 FlipW = _mm_xor_ps(W[0], _mm_xor_ps(W[1], W[2]));
			FlipW = _mm_and_ps(FlipW, _mm_set1_ps(-0.0f));
			__m128 validArea = _mm_xor_ps(negativeArea, FlipW);

			swapMask = _mm_cmpge_ps(validArea, _mm_setzero_ps());
			__m128 validSwapMask = _mm_and_ps(swapMask, primitiveValid); //if triangle behind camera, ignore
			needRearrangeTriangle = _mm_same_sign0(validSwapMask) == false;
		}
		else
		{
			swapMask = _mm_cmpge_ps(negativeArea, _mm_setzero_ps());
			primitiveValid = _mm_cmpneq_ps(negativeArea, _mm_setzero_ps());
			needRearrangeTriangle = _mm_same_sign0(swapMask) == false;
		}

		if (needRearrangeTriangle)
		{
			//time to re-order the vertex indices
			//mainly swap vertex 1 and vertex 2
			__m128 temp;

			temp = _mm_blendv_ps(x[1], x[2], swapMask);
			x[2] = _mm_blendv_ps(x[2], x[1], swapMask);
			x[1] = temp;

			temp = _mm_blendv_ps(y[1], y[2], swapMask);
			y[2] = _mm_blendv_ps(y[2], y[1], swapMask);
			y[1] = temp;

			temp = _mm_blendv_ps(W[1], W[2], swapMask);
			W[2] = _mm_blendv_ps(W[2], W[1], swapMask);
			W[1] = temp;

			temp = _mm_blendv_ps(invW[1], invW[2], swapMask);
			invW[2] = _mm_blendv_ps(invW[2], invW[1], swapMask);
			invW[1] = temp;


			edgeNormalsX[0] = _mm_sub_ps(y[1], y[0]);
			edgeNormalsX[1] = _mm_sub_ps(y[2], y[1]);


			edgeNormalsY[0] = _mm_sub_ps(x[0], x[1]);
			edgeNormalsY[1] = _mm_sub_ps(x[1], x[2]);

			swapMask = _mm_and_ps(swapMask, _mm_set1_ps(-0.0f));
			negativeArea = _mm_xor_ps(negativeArea, swapMask);
		}

	}
	else {

		// Need to flip back faceNum test for each W < 0
		if (possiblyNearClipped)
		{
			__m128i areaCheckMask = _mm_set1_epi32(0x7fffffff);

			//area threshold change to 0x7f000000
			//previous setting is 0x53800000 1.09951162778e+12
			__m128i areaThreshold = _mm_set1_epi32(0x7f000000);
			__m128i checkArea = _mm_and_si128(_mm_castps_si128(negativeArea), areaCheckMask); //after this, valid Mask is positive Area
			__m128i validMask = _mm_cmplt_epi32(checkArea, areaThreshold);

			primitiveValid = _mm_and_ps(_mm_castsi128_ps(validMask), primitiveValid);

			//flip negativeArea sign if any W is negative
			__m128 validArea = _mm_xor_ps(_mm_xor_ps(negativeArea, W[1]), _mm_xor_ps(W[0], W[2]));
			primitiveValid = _mm_and_ps(validArea, primitiveValid);
		}
		else
		{
			__m128 validLarge = _mm_cmplt_ps(negativeArea, _mm_set1_ps(-0.00002));  //if primitive too small, just ignore!!!
			primitiveValid = _mm_and_ps(validLarge, primitiveValid);
		}


		if (DebugOccluderOccludee)
		{
			int temp = GetValidPrimitiveNum(primitiveValid);
			this->DebugData[PrimitiveBackFaceCull] += DebugData[PrimitiveValidNum] - temp;
			DebugData[PrimitiveValidNum] = temp;
		}
	}


	if (_mm_same_sign0(primitiveValid) == true)
	{
		if (DebugOccluderOccludee)
		{
			this->DebugData[P4BackFaceCull]++;
		}
		return;
	}

	__m128 minFx, minFy, maxFx, maxFy;
	__m128 NearClipMaskMin;
	if (possiblyNearClipped)
	{
		// Clipless bounding box computation
		__m128 infP = _mm_set1_ps(+10000.0f);
		//__m128 infN = _mm_set1_ps(-10000.0f);

		// Find  interval of points with W > 0
		//__m128 zero = _mm_setzero_ps();
		__m128 infN = _mm_setzero_ps();
		__m128 maskMin0 = _mm_cmplt_ps(W[0], infN);
		__m128 maskMin1 = _mm_cmplt_ps(W[1], infN);
		__m128 maskMin2 = _mm_cmplt_ps(W[2], infN);
		NearClipMaskMin = _mm_or_ps(_mm_or_ps(maskMin0, maskMin1), maskMin2);


		// have some slight improvement
		__m128 minPx = _mm_min_ps(
			_mm_min_ps(_mm_blendv_ps(x[0], infP, maskMin0), _mm_blendv_ps(x[1], infP, maskMin1)),
			_mm_blendv_ps(x[2], infP, maskMin2));

		__m128 minPy = _mm_min_ps(
			_mm_min_ps(_mm_blendv_ps(y[0], infP, maskMin0), _mm_blendv_ps(y[1], infP, maskMin1)),
			_mm_blendv_ps(y[2], infP, maskMin2));

		__m128 maxPx = _mm_max_ps(
			_mm_max_ps(_mm_blendv_ps(x[0], infN, maskMin0), _mm_blendv_ps(x[1], infN, maskMin1)),
			_mm_blendv_ps(x[2], infN, maskMin2));

		__m128 maxPy = _mm_max_ps(
			_mm_max_ps(_mm_blendv_ps(y[0], infN, maskMin0), _mm_blendv_ps(y[1], infN, maskMin1)),
			_mm_blendv_ps(y[2], infN, maskMin2));

		__m128 minNx = _mm_min_ps(
			_mm_min_ps(_mm_blendv_ps(infP, x[0], maskMin0), _mm_blendv_ps(infP, x[1], maskMin1)),
			_mm_blendv_ps(infP, x[2], maskMin2));

		__m128 minNy = _mm_min_ps(
			_mm_min_ps(_mm_blendv_ps(infP, y[0], maskMin0), _mm_blendv_ps(infP, y[1], maskMin1)),
			_mm_blendv_ps(infP, y[2], maskMin2));

		__m128 maxNx = _mm_max_ps(
			_mm_max_ps(_mm_and_ps(x[0], maskMin0), _mm_and_ps(x[1], maskMin1)),
			_mm_and_ps(x[2], maskMin2));

		__m128 maxNy = _mm_max_ps(
			_mm_max_ps(_mm_and_ps(y[0], maskMin0), _mm_and_ps(y[1], maskMin1)),
			_mm_and_ps(y[2], maskMin2));

		__m128 incAx = _mm_and_ps(minPx,  _mm_cmple_ps(maxNx, minPx));
		__m128 incAy = _mm_and_ps(minPy,  _mm_cmple_ps(maxNy, minPy));
		__m128 incBx = _mm_blendv_ps(maxPx, infP, _mm_cmpgt_ps(maxPx, minNx));
		__m128 incBy = _mm_blendv_ps(maxPy, infP, _mm_cmpgt_ps(maxPy, minNy));

		minFx = _mm_min_ps(incAx, incBx);
		minFy = _mm_min_ps(incAy, incBy);
		maxFx = _mm_max_ps(incAx, incBx);
		maxFy = _mm_max_ps(incAy, incBy);
	}
	else
	{
		// Standard bounding box inclusion
		minFx = _mm_min_ps(_mm_min_ps(x[0], x[1]), x[2]);
		maxFx = _mm_max_ps(_mm_max_ps(x[0], x[1]), x[2]);

		minFy = _mm_min_ps(_mm_min_ps(y[0], y[1]), y[2]);

		maxFy = _mm_max_ps(_mm_max_ps(y[0], y[1]), y[2]);
	}

	// Clamp upper bound and unpack
	__m128i bounds[4];
	if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER) {
		bounds[0] = _mm_max_epi32(_mm_cvttps_epi32(minFx), _mm_set1_epi32(0));
		bounds[1] = _mm_min_epi32(_mm_cvttps_epi32(maxFx), _mm_set1_epi32(m_blocksXMinusOne));
	}
	else {
		bounds[0] = _mm_max_epi32(_mm_cvttps_epi32(minFx), _mm_set1_epi32(mBlockWidthMin));
		bounds[1] = _mm_min_epi32(_mm_cvttps_epi32(maxFx), _mm_set1_epi32(mBlockWidthMax));
	}
	bounds[2] = _mm_max_epi32(_mm_cvttps_epi32(minFy), _mm_setzero_si128());
	bounds[3] = _mm_min_epi32(_mm_cvttps_epi32(maxFy), _mm_set1_epi32(m_blocksYMinusOne));

	// Check overlap between bounding box and frustum
	__m128 isInFrustum = _mm_castsi128_ps(_mm_and_si128(_mm_cmple_epi32_soc(bounds[0], bounds[1]), _mm_cmple_epi32_soc(bounds[2], bounds[3])));
	__m128 one16 = _mm_set1_ps(1.0/16.0);
	isInFrustum = _mm_and_ps(isInFrustum, _mm_cmpge_ps(maxFx, one16));
	isInFrustum = _mm_and_ps(isInFrustum, _mm_cmpge_ps(maxFy, one16));

	primitiveValid = _mm_and_ps(isInFrustum, primitiveValid);
	uint32_t validMask = _mm_movemask_ps(primitiveValid);

	if (DebugOccluderOccludee)
	{
		int temp = GetValidPrimitiveNum(primitiveValid);
		this->DebugData[PrimitiveFrustumCull] += DebugData[PrimitiveValidNum] - temp;
		DebugData[PrimitiveValidNum] = temp;
	}

	if (validMask == 0)
	{
		if (DebugOccluderOccludee)
		{
			this->DebugData[P4FrustumCull]++;	
			assert(DebugData[PrimitiveValidNum] == 0);
		}
		return;
	}
	if (DebugOccluderOccludee)
	{
		this->DebugData[P4PassFrustumCull] ++;
	}


	// Compute Z from linear relation with 1/W
	__m128 maxZ = _mm_fmadd_ps_soc(_mm_max_ps(_mm_max_ps(invW[0], invW[1]), invW[2]), occluderCache->c1, occluderCache->c0);
	// If any W < 0, assume maxZ = 1 (effectively disabling Hi-Z)
	if (possiblyNearClipped)
	{
		maxZ = _mm_or_ps(maxZ, NearClipMaskMin);  //save one load and _mm_blendv_ps
	}
	else
	{
		//maxZ = _mm_min_ps(maxZ, _mm_castsi128_ps(_mm_set1_epi32(0x0ffff000)));
	}

	__m128i maxZi = _mm_srai_epi32(_mm_castps_si128(maxZ), 12);

	maxZi = _mm_min_epu32(maxZi, _mm_set1_epi32(65535));
	uint32_t* depthBounds = (uint32_t*)&maxZi;
	

	__m128i minZi;
	uint32_t* depthBoundsMin;

	if (bDrawOccludee && possiblyNearClipped == false)
	{
		__m128 minZ;
		if (occluderCache->SuperFlatOccludee == false) {
			minZ = _mm_fmadd_ps_soc(_mm_min_ps(_mm_min_ps(invW[0], invW[1]), invW[2]), occluderCache->c1, occluderCache->c0);
			minZ = _mm_and_ps(minZ, _mm_cmpgt_ps(minZ, _mm_setzero_ps()));
		}
		else {
			minZ = maxZ;
		}

		minZi = _mm_srai_epi32(_mm_castps_si128(minZ), 12);
		depthBoundsMin = (uint32_t*)&minZi;
	}

	//quick cull check
	if (CULL_FEATURE_HizPrimitiveCull)
	{
		uint32_t alivePrimitive = 0;
		uint16_t* pHiZBuffer = m_pHiz;
		uint32_t pValidIdx = mAliveIdxMask[validMask];
		if (bDrawOccludee && possiblyNearClipped == false && !bDrawOccludeeToDepthMap)
		{
			uint32_t pValidIdx2 = pValidIdx;

			__m128 cx = _mm_add_ps(_mm_add_ps(x[0], x[1]), x[2]);
			__m128 cy = _mm_add_ps(_mm_add_ps(y[0], y[1]), y[2]);
			__m128 scaleC = _mm_set1_ps(1.0f/3.0f);
			cx = _mm_mul_ps(cx, scaleC);
			cy = _mm_mul_ps(cy, scaleC);

			float* cxf = (float*)&cx;
			float* cyf = (float*)&cy;

			do{
				uint32_t primitiveIdx = pValidIdx2 & 3;
				pValidIdx2 >>= 2;
				uint16_t primitiveMinZ = depthBoundsMin[primitiveIdx];
				uint16_t* pOffsetHiZMax = pHiZBuffer + m_HizBufferSize;
				uint32_t xLoc = (uint32_t)cxf[primitiveIdx];
				uint32_t yLoc = (uint32_t)cyf[primitiveIdx];
				if (cxf[primitiveIdx] >= 0.0f && xLoc <= m_blocksXMinusOne && cyf[primitiveIdx] >= 0.0f && yLoc <= m_blocksYMinusOne) {
					uint32_t xyLoc = (m_blocksX * yLoc + xLoc);
					uint16_t max = pOffsetHiZMax[xyLoc] - 1;
					if (primitiveMinZ > max ) {
						occluderCache->mRasterizedOccludeeVisible = true;
						return;
					}
				}
			} while (pValidIdx2 != 0);
		}

		do
		{
			// Move index and mask to next set bit
			uint32_t primitiveIdx = pValidIdx & 3;
			pValidIdx >>= 2;

			// Extract and prepare per-primitive data
			uint16_t primitiveMaxZ = depthBounds[primitiveIdx];
			//do a quick check here to reject occluded primitive
			{

				if (possiblyNearClipped && NEAR_CLIP_SURE_VISIBLE_OPTIMIZATION)
				{
					if (primitiveMaxZ == MAX_DEPTH)
					{
						alivePrimitive |= 1 << primitiveIdx;
						continue;
					}
				}


				uint32_t * boundData = ((uint32_t*)bounds) + primitiveIdx;
				uint32_t blockMinX = boundData[0];
				uint32_t blockMaxX = boundData[4];
				const uint32_t blockMinY = boundData[8]; 
				uint32_t blockMaxY = boundData[12];


				uint16_t *pOffsetHiZ = pHiZBuffer + (m_blocksX * blockMinY + blockMinX);

				//reduce 1 and then blockRangeY = BlockMax-BlockMin, then loop  [0, BlockMax - blockMin]
				//blockRangeY--; //bounds[1] = _mm_add_epi32(_mm_sub_epi32(bounds[1], bounds[0]), _mm_set1_epi32(1));
				//blockRangeX--; //bounds[3] = _mm_add_epi32(_mm_sub_epi32(bounds[3], bounds[2]), _mm_set1_epi32(1));
				int blockRangeX = blockMaxX - blockMinX ;
				__m128i primitiveMaxZVi = _mm_set1_epi16(primitiveMaxZ);
				if (DebugOccluderOccludee)
				{
					DebugData[BlockDoWhileIfSave]++;
				}
				uint32_t NextBlockY = blockMinY;
				do
				{
					uint32_t blockY = NextBlockY++;
					uint16_t *pBlockRowHiZ = pOffsetHiZ;
					pOffsetHiZ += m_blocksX;
					

					if (DebugOccluderOccludee)
					{
						DebugData[BlockDoWhileIfSave]++;
					}
					int32_t NextRowRangeX = blockRangeX;
					do
					{
						int32_t rowRangeX = NextRowRangeX;
						NextRowRangeX -= 8;

						// Load HiZ for 8 blocks at once - note we're possibly reading out-of-bounds here; but it doesn't affect correctness if we test more blocks than actually covered
						__m128i hiZblob = _mm_loadu_si128(reinterpret_cast<const __m128i *>(pBlockRowHiZ));
						__m128i cmpResult;
						if(bDrawOccludee)	cmpResult  = _mm_cmple_epu16_soc(hiZblob, primitiveMaxZVi);
						else 				cmpResult = _mm_cmplt_epu16_soc(hiZblob, primitiveMaxZVi);
						uint64_t * r64 = (uint64_t *)& cmpResult;
						uint64_t result = r64[0] & r64[1];

						if (result != -1) 
						{
							int32_t byteBlockCheck = std::min(7, rowRangeX);
							int32_t bit = byteBlockCheck >> 2;
							int32_t offset = byteBlockCheck & 3;
							r64[bit] <<= (3 ^ offset) << 4;

							r64[1] &= (uint64_t)(-bit);


							result = r64[0] | r64[1];

							if (result == 0)
							{
								pBlockRowHiZ += 8;
								continue;
							}
						}
						
						boundData[8] = blockY;

						alivePrimitive |= 1 << primitiveIdx;
						blockMaxY = 0; //reset blockMaxY  to 0 to force do while exit

						break;		
					} while (NextRowRangeX >= 0);
				} while (NextBlockY <= blockMaxY);

				if (DebugOccluderOccludee) 
				{
					uint32_t checkMask = 1 << primitiveIdx;
					if ((alivePrimitive & checkMask) == 0)
					{
						DebugData[PrimitiveEarlyHiZCull]++;
						DebugData[PrimitiveValidNum] --;
					}
				}
				
			}
		} while (pValidIdx != 0);

		if (alivePrimitive == 0)
		{
			if (DebugOccluderOccludee) 
			{
				DebugData[P4EarlyHizCull] ++;
				assert(DebugData[PrimitiveValidNum] == 0);
			}
			return;
		}

		validMask = mAliveIdxMask[alivePrimitive];
	}
	if (DebugOccluderOccludee)
	{
		DebugData[P4EarlyHizCullPass] ++;
	}

	if (bDrawOccludee && possiblyNearClipped == false) {
		minZi = _mm_slli_epi32(minZi, 12);
	}
	else
	{
		__m128 minZ = _mm_fmadd_ps_soc(_mm_min_ps(_mm_min_ps(invW[0], invW[1]), invW[2]), occluderCache->c1, occluderCache->c0);
		minZ = _mm_and_ps(minZ, _mm_cmpgt_ps(minZ, _mm_setzero_ps()));
		minZi = _mm_castps_si128(minZ);
		depthBoundsMin = (uint32_t*)&minZi;
	}

    if (bPixelAABBClipping)
    {
        auto scale8 = _mm_set1_ps(8);
		__m128i mask = _mm_set1_epi32((uint32_t(-1) << 16) | 7);

		__m128i pixel = _mm_cvttps_epi32(_mm_mul_ps(minFx, scale8));
		//make bit 16~31 store block value for each int component
		//make bit 0~15 store block pixel remainder(0~7)
		PrimitivePixelClip->PrimitivePixelBounds[0] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(maxFx, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[1] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(minFy, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[2] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
		pixel = _mm_cvttps_epi32(_mm_mul_ps(maxFy, scale8));
		PrimitivePixelClip->PrimitivePixelBounds[3] = _mm_and_si128(mask, _mm_or_si128(_mm_slli_epi32(pixel, 13), pixel));
    }

	

	if (this->mDebugRenderMode && bDrawOccludee == false)
	{
		HandleDrawMode<3>(x, y, invW, validMask, occluderCache);
		if(bDebugOccluderOnly){
			if (this->DebugData[BlockPacketPrimitiveDebug] == 0) return;
			this->DebugData[BlockPacketPrimitiveDebug] = 0;
		}
	}

	//delay the calculation of edgeNormal X Y, 2,3
	edgeNormalsX[2] = _mm_sub_ps(y[0], y[2]);
	edgeNormalsY[2] = _mm_sub_ps(x[2], x[0]);


	// Do a precise division to reduce error in depth plane. Note that the negativeArea computed here
	// differs from the rasterized region if W < 0, so it can be very small for large covered screen regions.
	//invArea = _mm_div_ps(_mm_set1_ps(1.0f), negativeArea);
	__m128 negativeC1 = _mm_negate_ps_soc(occluderCache->c1);
	__m128 invArea = _mm_div_ps(negativeC1, negativeArea);


	__m128 z0 = _mm_fmadd_ps_soc(invW[0], occluderCache->c1, occluderCache->c0);
	__m128 z20 = _mm_sub_ps(invW[2], invW[0]);
	__m128 z12 = _mm_sub_ps(invW[1], invW[2]);



	// Compute screen space depth plane
	__m128 depthPlane[3];
	depthPlane[1] = _mm_mul_ps(invArea, _mm_fmsub_ps_soc(z20, edgeNormalsX[1], _mm_mul_ps(z12, edgeNormalsX[2])));
	depthPlane[2] = _mm_mul_ps(invArea, _mm_fmsub_ps_soc(z20, edgeNormalsY[1], _mm_mul_ps(z12, edgeNormalsY[2])));


	
	// Depth at center of first pixel
	__m128 refX = _mm_sub_ps(_mm_set1_ps(1.0f / 16.0f), x[0]);
	__m128 refY = _mm_sub_ps(_mm_set1_ps(1.0f / 16.0f), y[0]);
	depthPlane[0] = _mm_fmadd_ps_soc(refX, depthPlane[1], _mm_fmadd_ps_soc(refY, depthPlane[2], z0));

	// Flip edges if W < 0
	__m128 edgeFlipMask[3];
	if (possiblyNearClipped)
	{
		edgeFlipMask[0] = _mm_xor_ps(invW[0], invW[1]);
		edgeFlipMask[1] = _mm_xor_ps(invW[1], invW[2]);
		edgeFlipMask[2] = _mm_xor_ps(invW[0], invW[2]);
        
        __m128 minusZero = _mm_set1_ps(-0.0f);
        edgeFlipMask[0] = _mm_and_ps(edgeFlipMask[0], minusZero);
        edgeFlipMask[1] = _mm_and_ps(edgeFlipMask[1], minusZero);
        edgeFlipMask[2] = _mm_and_ps(edgeFlipMask[2], minusZero);
        
		edgeNormalsX[0] = _mm_xor_ps(edgeNormalsX[0], edgeFlipMask[0]);
		edgeNormalsY[0] = _mm_xor_ps(edgeNormalsY[0], edgeFlipMask[0]);

		edgeNormalsX[1] = _mm_xor_ps(edgeNormalsX[1], edgeFlipMask[1]);
		edgeNormalsY[1] = _mm_xor_ps(edgeNormalsY[1], edgeFlipMask[1]);

		edgeNormalsX[2] = _mm_xor_ps(edgeNormalsX[2], edgeFlipMask[2]);
		edgeNormalsY[2] = _mm_xor_ps(edgeNormalsY[2], edgeFlipMask[2]);
	}

	// Normalize edge equations for lookup

	__m128 invLen[3];
	normalizeEdge(edgeNormalsX[0], edgeNormalsY[0], invLen[0]);
	normalizeEdge(edgeNormalsX[1], edgeNormalsY[1], invLen[1]);
	normalizeEdge(edgeNormalsX[2], edgeNormalsY[2], invLen[2]);
	

	// edgeOffsets is calculated so that (x0, y0) (x1, y1) fall on the line
	//edgeNormalsX * X  + edgeNormalsY * Y + edgeOffsets = 0;
	//substitute (x0, y0)
	//(y1-y0) * invLen * x0 + (x0-x1) * invLen  * y0 + edgeOffsets = 0
	//=> edgeOffsets =  (x1y0 - y1x0) * invLen 

	__m128 edgeOffsets[3];
	// Important not to use FMA here to ensure identical results between neighboring edges
	edgeOffsets[0] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[1], y[0]), _mm_mul_ps(y[1], x[0])), invLen[0]);
	edgeOffsets[1] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[2], y[1]), _mm_mul_ps(y[2], x[1])), invLen[1]);
	edgeOffsets[2] = _mm_mul_ps(_mm_sub_ps(_mm_mul_ps(x[0], y[2]), _mm_mul_ps(y[0], x[2])), invLen[2]);

	// Flip edge offsets as well
	if (possiblyNearClipped)
	{
		edgeOffsets[0] = _mm_xor_ps(edgeOffsets[0], edgeFlipMask[0]);
		edgeOffsets[1] = _mm_xor_ps(edgeOffsets[1], edgeFlipMask[1]);
		edgeOffsets[2] = _mm_xor_ps(edgeOffsets[2], edgeFlipMask[2]);
	}

	// Quantize slopes
	__m128i slopeLookups[4];
	slopeLookups[0] = quantizeSlopeLookup(edgeNormalsX[0], edgeNormalsY[0]);
	slopeLookups[1] = quantizeSlopeLookup(edgeNormalsX[1], edgeNormalsY[1]);
	slopeLookups[2] = quantizeSlopeLookup(edgeNormalsX[2], edgeNormalsY[2]);
	__m128i mergedLookup = _mm_or_si128(_mm_or_si128(slopeLookups[0], slopeLookups[1]), slopeLookups[2]);
	//slopeLookups[3] = _mm_cmpgt_epi32(mergedLookup, _mm_set1_epi32(64 * 64 - 1));
	//faster than _mm_cmpgt_epi32(mergedLookup, _mm_set1_epi32(64 * 64 - 1));
	slopeLookups[3] = _mm_srli_epi32(mergedLookup, 12); //12 means 64*64
	
	if (DebugOccluderOccludee)
	{
		this->DebugData[P4PassCull] ++;
		mergedLookup = _mm_cmplt_epi32(mergedLookup, _mm_set1_epi32(64 * 64));
		int latestMask = _mm_movemask_ps(_mm_castsi128_ps(mergedLookup));
		int rejected = 0;
		uint32_t aliveMask = 0;
		uint32_t alivePrimitiveTemp = validMask;
		do
		{
			int j = (alivePrimitiveTemp & 3);
			aliveMask |= 1 << j;
			alivePrimitiveTemp >>= 2;
		} while (alivePrimitiveTemp > 0);

		rejected += ((aliveMask & 1) > 0) && ((latestMask & 1) == 0);
		rejected += ((aliveMask & 2) > 0) && ((latestMask & 2) == 0);
		rejected += ((aliveMask & 4) > 0) && ((latestMask & 4) == 0);
		rejected += ((aliveMask & 8) > 0) && ((latestMask & 8) == 0);
		
		DebugData[PrimitiveValidNum] -= rejected;
		DebugData[PrimitiveDegenerateCull] += rejected;
		
		DebugData[DebugData[PrimitiveValidNum]] ++;

		uint32_t mergedMask = aliveMask & _mm_movemask_ps(_mm_castsi128_ps(mergedLookup));
		if (mergedMask > 0)
		{
			this->DebugData[P4Rasterized]++;
		}
	}


	__m128 largePrimitive = _mm_cmplt_ps(negativeArea, _mm_set1_ps(-ExtremeSmallPrimitiveAreaTh));
	uint32_t* largePrimitivef = (uint32_t*)&largePrimitive;
	do
	{	
		uint32_t primitiveIdx = validMask & 3;
		validMask >>= 2;

		uint32_t* slopeLookup = ((uint32_t*)&slopeLookups) + primitiveIdx;
		if (slopeLookup[12] != 0) {
			continue;
		}
		
		if (bDebugOccluderOnly) 
		{
			if (bDebugOccluderPixelX != -1 && bDebugOccluderPixelY != -1) 
			{
				//if (this->DebugData[BlockPacketID] != 13) return;
				if (primitiveIdx != this->DebugData[BlockPacketPrimitive]) continue;
			}
		}

		const uint64_t* pRow0 = m_pMaskTable + slopeLookup[0];
		const uint64_t* pRow1 = m_pMaskTable + slopeLookup[4];
		const uint64_t* pRow2 = m_pMaskTable + slopeLookup[8];

		// Extract and prepare per-primitive dataprimitiveMaxZV
		uint32_t primitiveMaxZf = depthBounds[primitiveIdx];
		uint16_t primitiveMaxZ = (uint16_t)primitiveMaxZf;
		uint32_t primitiveMinZf = depthBoundsMin[primitiveIdx];

		primitiveMaxZf <<= 12;
		uint32_t small_primitive_special_mask = -1;
		if (bDrawOccludee || bDebugVerySmallOccluders) {
			small_primitive_special_mask = largePrimitivef[primitiveIdx];
		}


#if defined( SUPPORT_ALL_FEATURE)
		static int DebugPrimitiveId = 0;
		if (bDumpTriangle && bDumpBlockColumnImage)
		{
			DebugPrimitiveId++;
			if (DebugPrimitiveId != 160) continue;
		}
#endif

		if (DebugOccluderOccludee)
		{
			if (slopeLookup[12] == 0) //no degenerate case
			{
				this->DebugData[PrimitiveRasterizedNum]++;
				DebugData[PrimitiveValidNum] --;

				this->DebugData[PrimitiveNearClipeRasterized] += possiblyNearClipped;
			}
		}


		float* depthPlaneData = ((float*)depthPlane) + primitiveIdx;

		float slope = depthPlaneData[8];

		__m128 depthBlockDelta = _mm_set1_ps(slope);

		__m128 depthRowDeltaBtm = _mm_setzero_ps(); 
		depthRowDeltaBtm = _mm_min_ps(_mm_set1_ps(slope* 0.375f), depthRowDeltaBtm);


		
		int xIncrease = (int)(depthPlaneData[4] > 0);
		int yIncrease = (int)(slope > 0);
		//data16: btmLeft 0 1 btmRight 2 3 topleft 4 5 topright 6 7
		int maxBlockIdx = (xIncrease  + yIncrease * 2) << 1;
		int	minBlockIdx = 6 ^ maxBlockIdx;



		__m128 depthDx = _mm_set1_ps(depthPlaneData[4]);
		__m128 depthLeftBase;
		if (VRS_X4Y4_Optimzation)
		{
			depthLeftBase = _mm_fmadd_ps_soc(depthDx, xFactors[xIncrease], _mm_set1_ps(depthPlaneData[0]));
			float halfSlope = slope * 0.5f;
			depthLeftBase = _mm_add_ps(depthLeftBase, _mm_setr_ps(0, 0, halfSlope, halfSlope));
		}
		else
		{
			depthLeftBase = _mm_fmadd_ps_soc(depthDx, _mm_setr_ps(0.0f, 0.125f, 0.25f, 0.375f), _mm_set1_ps(depthPlaneData[0]));
		}


		uint32_t * boundData = ((uint32_t*)bounds) + primitiveIdx;
		uint32_t blockMinX = boundData[0];
		const uint32_t blockMaxX = boundData[4];
		const uint32_t blockMinY = boundData[8];
		const uint32_t blockMaxY = boundData[12];



		float * edgeNormalsXf = (float*)edgeNormalsX + primitiveIdx;
		float * edgeNormalsYf = (float*)edgeNormalsY + primitiveIdx;
		float * edgeOffsetsf = (float*)edgeOffsets + primitiveIdx;

		__m128 edgeNormalsXP = _mm_setr_ps(edgeNormalsXf[0], edgeNormalsXf[4], edgeNormalsXf[8], 0);
		__m128 edgeNormalsYP = _mm_setr_ps(edgeNormalsYf[0], edgeNormalsYf[4], edgeNormalsYf[8], 0);
		__m128 edgeOffsetsP = _mm_setr_ps(edgeOffsetsf[0], edgeOffsetsf[4], edgeOffsetsf[8], 0);

		//delay calculation of edgeNormalsX edgeNormalsY
		//_mm_add_ps(edgeNormalsX[primitiveIdx], edgeNormalsY[primitiveIdx]), _mm_set1_ps(0.5f) is the central point of 8x8 block
		__m128 edgeOffset = _mm_fmadd_ps_soc(_mm_add_ps(edgeNormalsXP, edgeNormalsYP), _mm_set1_ps(0.5f), edgeOffsetsP);
		__m128 edge_mul = _mm_set1_ps(OFFSET_mul);
		edgeOffset = _mm_fmadd_ps_soc(edgeOffset, edge_mul, _mm_set1_ps(OFFSET_add));

		__m128 edgeNormalX = _mm_mul_ps(edgeNormalsXP, edge_mul);
		__m128 edgeNormalY = _mm_mul_ps(edgeNormalsYP, edge_mul);

		
		const uint32_t blocksX = m_blocksX;
		const uint32_t blocksXRows = m_blocksXFullDataRows;

		uint32_t StartYBlocks = blocksX * blockMinY;

		uint16_t *pOffsetHiZ = m_pHiz + StartYBlocks;
		uint64_t *outblockRowData = m_pDepthBuffer + StartYBlocks* PairBlockNum;

		__m128 rowDepthLeftBtmOffset = _mm_add_ps(depthLeftBase, depthRowDeltaBtm);

		if (bPixelAABBClipping)
		{
			PrimitivePixelClip->UpdatePixelAABBData(primitiveIdx, mPixelBound);
		}
		if (DebugOccluderOccludee) {
			DebugData[BlockTotalPrimitives]++;
		}



		__m128 blockMinYf = _mm_set1_ps(float(blockMinY));
		__m128 rowDepthLeftOffsetY = _mm_fmadd_ps_soc(depthBlockDelta, blockMinYf, rowDepthLeftBtmOffset);
		__m128 edgeOffsetY = _mm_fmadd_ps_soc(edgeNormalY, blockMinYf, edgeOffset);


		__m128 offsetX = _mm_fmadd_ps_soc(edgeNormalX, _mm_set1_ps((float)blockMinX), edgeOffsetY);
		int32_t PreviousSkip = 65536;

		uint32_t blockY = blockMinY;
		uint32_t NextBlockX = blockMinX;
		int32_t CurrentSkip = 0;


		if (DebugOccluderOccludee)
		{
			DebugData[BlockTotal]+= (blockMaxX - blockMinX + 1) * (blockMaxY - blockMinY + 1);
		}
		while (true)
		{
			if (DebugOccluderOccludee)
			{
				DebugData[BlockDoWhileIfSave]++;
			}

			uint32_t ConvexOffset = 0; 
			do
			{

				__m128i lookup = _mm_cvttps_epi32(offsetX);
				offsetX = _mm_add_ps(edgeNormalX, offsetX);
				lookup = _mm_max_epi32(lookup, _mm_setzero_si128());
				
				uint32_t* lookIdx = (uint32_t*)&lookup;
				uint32_t idxOr = lookIdx[0] | lookIdx[1] | lookIdx[2];
				if (bDrawOccludee || bDebugVerySmallOccluders) {
					idxOr &= small_primitive_special_mask;
				}

				if (idxOr > 63)
				{
					NextBlockX++;

					if (bConvexOptimization) {						
						if (DebugOccluderOccludee) 
						{
							if (ConvexOffset > 0) {
								uint32_t blockX = NextBlockX-1;
								DebugData[BlockConvexRow10Cull] += blockMaxX - blockX;
							}
							else {
								DebugData[BlockConvexRow10CullOverhead] ++;
							}
						}
						//Convex Optimization 0: YesNo optimization. Stop if Block state from see to not see
						NextBlockX |= ConvexOffset;
						CurrentSkip++;						
					}
					if (DebugOccluderOccludee)
					{
						DebugData[BlockOneSureZeroCull] ++;
					}
					continue;
				}

				uint32_t blockX = NextBlockX;
				NextBlockX++;

				//put the convex optimization here
				//because there are lots of single block update
				if (bConvexOptimization) {
					ConvexOffset = 65536;
					if (DebugOccluderOccludee)
					{
						DebugData[BlockConvexRow10CullOverhead] ++;
					}
				}

				uint16_t *pBlockRowHiZ = pOffsetHiZ + blockX;
				if (pBlockRowHiZ[0] >= primitiveMaxZ)
				{
					if (DebugOccluderOccludee) {
						DebugData[BlockPrimitiveMaxLessThanMinCull]++;
					}
					continue;
				}
				
				uint64_t blockMask = -1; // in case of all 0, the whole block is active
				if (idxOr != 0) 
				{
					blockMask  = pRow0[lookIdx[0]];
					blockMask &= pRow1[lookIdx[1]];
					blockMask &= pRow2[lookIdx[2]];
					// No pixels covered => skip block
					if (blockMask == 0)
					{
						if (DebugOccluderOccludee) {
							DebugData[BlockMaskJointZeroCull]++;
						}
						continue;
					}
				}
				

				if (bDumpBlockColumnImage)
				{
					if (blockX == DebugDumpBlockX && blockY == DebugDumpBlockY) {}
					else continue;

					}

				//draw triangle
				if (VRS_X4Y4_Optimzation)
				{
					__m128i rowDepthLeft = _mm_castps_si128(_mm_fmadd_ps_soc(depthDx, _mm_set1_ps((float)blockX), rowDepthLeftOffsetY));

					rowDepthLeft = _mm_max_epi32(rowDepthLeft, _mm_set1_epi32(primitiveMinZf));
					rowDepthLeft = _mm_min_epi32(rowDepthLeft, _mm_set1_epi32(primitiveMaxZf));

					__m128i depthData = packDepthPremultipliedVRS12Fast(rowDepthLeft);

					uint16_t * depth16 = (uint16_t*)&depthData;

					uint16_t maxBlockDepth = depth16[maxBlockIdx];
					if (bDebug_ValidateMinMaxBlockIdx) {
						uint16_t minDepth = depth16[minBlockIdx];
						for (int xx = 0; xx < 8; xx++) {
							assert(depth16[xx] <= maxBlockDepth);
							assert(depth16[xx] >= minDepth);
						}
					}
					if (maxBlockDepth > pBlockRowHiZ[0])
					{
						uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;

						uint32_t * depth32 = (uint32_t*)depth16;
						if (PairBlockNum <= CheckerBoardVizMaskApproach)
						{
							bool small_primitive_check = false;
							if (bDrawOccludee || bDebugVerySmallOccluders) {
								small_primitive_check = small_primitive_special_mask == 0;
							}
							if (blockMask != -1 || small_primitive_check)
							{
								if (DebugOccluderOccludee)
								{
									DebugData[BlockRenderPartial]++;
									DebugData[BlockRenderTotal]++;
								}

								if (bPixelAABBClipping)
								{
									blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
									if (blockMask == 0)
									{
										if (DebugOccluderOccludee)
										{
											DebugData[BlockAABBClipToZero]++;
										}
										continue;
									}
								}

								if (PairBlockNum == PureCheckerBoardApproach) 
								{
									__m128i* out = (__m128i*)outBlockData;
									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										updateBlockMSCBPartial_Occludee(depth32, blockMask, out, occluderCache);
										if (occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
											return;
									}
									else {
										updateBlockMSCBPartial(depth32, blockMask, out, nullptr, pBlockRowHiZ, maxBlockDepth);
									}
								}
								else {
									int bit = blockX & 1;
									__m128i* out = GetDepthData(outBlockData, bit);
									uint64_t* maskData = GetMaskData(outBlockData, bit);

									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										updateBlockMSCBPartial_Occludee(depth32, blockMask, out, occluderCache);
										if (occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
											return;
									}
									else {
										updateBlockMSCBPartial(depth32, blockMask, out, maskData, pBlockRowHiZ, maxBlockDepth);
									}
								}
							}
							else //full block update
							{
								this->mUpdateAnyBlock = true;
								if (DebugOccluderOccludee)
								{
									DebugData[BlockRenderFull]++;
									DebugData[BlockRenderTotal]++;
									DebugData[BlockMinUseOne]++;
								}



								__m128i* out = nullptr;
								if (PairBlockNum == CheckerBoardVizMaskApproach) 
								{
									int bit = blockX & 1;
									out = GetDepthData(outBlockData, bit);
									uint64_t* maskData = GetMaskData(outBlockData, bit);
									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										occluderCache->mRasterizedOccludeeVisible = maskData[0] != -1;
										if(occluderCache->mRasterizedOccludeeVisible) return;
									}
									else 
									{
										maskData[0] = -1;
									}
								}
								else {
									out = (__m128i*) outBlockData;
								}

	

								uint16_t minBlockDepth = depth16[minBlockIdx];
								// All pixels covered => skip edge tests

								uint16_t * pBlockRowHiZMax = pBlockRowHiZ + m_HizBufferSize;
								if (minBlockDepth >= pBlockRowHiZMax[0]) //full block update, min is larger than exist max
								{
									if (DebugOccluderOccludee) {
										if (pBlockRowHiZ[0] == 0) 
										{
											DebugData[BlockRenderInitial]++;
											DebugData[BlockRenderInitialFull]++;
										}
									}

									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										occluderCache->mRasterizedOccludeeVisible = true;
										return;
									}
									else {
										__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
										__m128i depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
										out[0] = depthBottom;
										out[1] = depthBottom;
										out[2] = depthTop;
										out[3] = depthTop;
										pBlockRowHiZ[0] = minBlockDepth;

										pBlockRowHiZ[m_HizBufferSize] = maxBlockDepth;
									}
								}
								else
								{
									__m128i	depthBottom = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
									__m128i depthTop = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);

									if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
										__m128i visible = _mm_cmple_epu16_soc(out[0], depthBottom);
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[1], depthBottom));
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[2], depthTop));
										visible = _mm_or_si128(visible, _mm_cmple_epu16_soc(out[3], depthTop));
										if (_mm_anybit_one_soc(visible)) {
											occluderCache->mRasterizedOccludeeVisible = true;
											return;
										}
									}
									else {
										out[0] = _mm_max_epu16(out[0], depthBottom);
										out[1] = _mm_max_epu16(out[1], depthBottom);
										out[2] = _mm_max_epu16(out[2], depthTop);
										out[3] = _mm_max_epu16(out[3], depthTop);


										pBlockRowHiZ[0] = std::max<uint16_t>(minBlockDepth, pBlockRowHiZ[0]);
										pBlockRowHiZMax[0] = std::max<uint16_t>(maxBlockDepth, pBlockRowHiZMax[0]);
									}
								}
							}
						}
						else if (PairBlockNum == FullBlockApproach) 
						{
#if defined( SUPPORT_ALL_FEATURE)
							if (blockMask != -1 && bPixelAABBClipping)
							{
								blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
							}
							__m128i depthRows[2];
							depthRows[0] = _mm_setr_epi32(depth32[0], depth32[0], depth32[1], depth32[1]);
							depthRows[1] = _mm_setr_epi32(depth32[2], depth32[2], depth32[3], depth32[3]);
							if (bDrawOccludee && Rasterize_ClippedOccludee_AS_OCCLUDER && !bDrawOccludeeToDepthMap) {
								occluderCache->mRasterizedOccludeeVisible = updateBlock_Occludee(depthRows, blockMask, (__m128i*)(outBlockData), pBlockRowHiZ);
								if (occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
								{
									return;
								}
							}
							else {
								updateBlock(depthRows, blockMask, (__m128i*)(outBlockData), pBlockRowHiZ);
							}
#endif
						}
					}
					else
					{
						if (DebugOccluderOccludee) {
							DebugData[BlockMaxLessThanMinCull]++;
						}
					}

#if defined( SUPPORT_ALL_FEATURE)
					if (bDumpBlockColumnImage)
					{
						uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;
						uint64_t* maskData = GetMaskData(outBlockData, blockX & 1);

						uint64_t t0 = pRow0[lookIdx[0]];
						uint64_t t1 = pRow1[lookIdx[1]];
						uint64_t t2 = pRow2[lookIdx[2]];
						DumpColumnBlock(t0, t1, t2, primitiveMaxZ, maskData[0]);
					}
#endif
				}
				else
				{

#if defined( SUPPORT_ALL_FEATURE)
					__m128 depthDxHalf = _mm_set1_ps(depthPlaneData[4] * 0.5f);
					__m128 lineDepthLeft = _mm_fmadd_ps_soc(depthBlockDelta, _mm_set1_ps(float(blockY)), depthLeftBase);
					__m128 rowDepthLeft = _mm_fmadd_ps_soc(depthDx, _mm_set1_ps((float)blockX), lineDepthLeft);
					__m128 rowDepthRight = _mm_add_ps(depthDxHalf, rowDepthLeft);

					if (blockMask != -1 && bPixelAABBClipping)
					{
						blockMask &= PrimitivePixelClip->GetPixelAABBMask(blockX, blockY);
					}

					__m128 depthRowDelta = _mm_set1_ps(slope * 0.125f);

					uint64_t *outBlockData = outblockRowData + blockX * PairBlockNum;
					updateBlockWithMaxZ(rowDepthLeft, rowDepthRight, blockMask, depthRowDelta, _mm_set1_epi32(primitiveMaxZf), (__m128i*) outBlockData, pBlockRowHiZ);
#endif
				}
			}while (NextBlockX <= blockMaxX);
			if (blockY >= blockMaxY)
			{
				break;
			}
			blockY++;

			//CurrentSkip means the number of empty blocks before block covered by triangle
			if (bConvexOptimization) 
			{
				CurrentSkip -= NextBlockX >> 16;
				if (CurrentSkip <= PreviousSkip) //>90% chance enter this...
				{
					if (DebugOccluderOccludee){
						DebugData[BlockConvexEdge31CullRowCheckPass]++;
					}
                    if (ConvexOffset != 0)
					{
						pOffsetHiZ += blocksX;
						outblockRowData += blocksXRows;
						edgeOffsetY = _mm_add_ps(edgeOffsetY, edgeNormalY);
						rowDepthLeftOffsetY = _mm_add_ps(rowDepthLeftOffsetY, depthBlockDelta);


						//Convex Optimization 3: Edge24 Block Backward Optimization
						static constexpr bool enableEdge24Skip = true;

						if (enableEdge24Skip == false)
						{
							PreviousSkip = CurrentSkip;
							NextBlockX = blockMinX;
							CurrentSkip = 0;
						}
						else
						{
							//safe to skip: 2 * current - previous - 1
							int nextSkip = (CurrentSkip << 1) - PreviousSkip - 1;
							PreviousSkip = CurrentSkip;
							//http://www-mdp.eng.cam.ac.uk/web/library/enginfo/mdp_micro/lecture4/lecture4-3-3.html
							CurrentSkip = nextSkip & ~(nextSkip >> 31); //ARM right shift sign extension

							if (DebugOccluderOccludee) {
								DebugData[BlockConvexEdge24Cull] += CurrentSkip;
								DebugData[BlockConvexEdge24Check]++;
							}

							NextBlockX = blockMinX + CurrentSkip;
						}
                        offsetX = _mm_fmadd_ps_soc(edgeNormalX, _mm_set1_ps((float)NextBlockX), edgeOffsetY);
                    }
					else
					{
						if (DebugOccluderOccludee) {
							DebugData[BlockConvexAllZeroRow]++;
						}
						//Convex Optimization 1: NoNoFastRowScan Empty Row Fast Check Optimization
						//This would happen when triangle clip with image region boundary
						uint32_t currentY = blockY;
						
						__m128 edgeNormalXBlockMinX = _mm_mul_ps(edgeNormalX, _mm_set1_ps((float)blockMinX));

						__m128 offsetMin = _mm_add_ps(edgeNormalXBlockMinX, edgeOffsetY);
						__m128 offsetMax = _mm_sub_ps(offsetX, edgeNormalX);//roll back one step

						 offsetMin = _mm_min_ps(offsetMin, offsetMax);
						
						
						//if head & tail both >= 63, the whole row could be skipped
						//any offset >= 63, the whole row could be skipped.
						offsetMin = _mm_sub_ps(offsetMin, _mm_set1_ps(63)); 
						
						uint32_t controlY = 0;
						do {
							if (DebugOccluderOccludee) {
								DebugData[BlockConvexRow00CullNextScanRows]++;
							}

							offsetMin = _mm_add_ps(offsetMin, edgeNormalY);
							__m128i cmp = _mm_srai_epi32(_mm_castps_si128(offsetMin), 31); //take sign only

							uint32_t * mask = (uint32_t*)&cmp;
							uint32_t skipMask = mask[0] & mask[1] & mask[2];

							currentY += (skipMask + 1);  // skipMask + 1 is equivalanet to skipMask == 0  for the value 0, -1
							controlY = currentY | skipMask;
							if (DebugOccluderOccludee) {
								if (skipMask == 0) {
									DebugData[BlockConvexRow00Cull] += blockMaxX - blockMinX + 1;
								}
							}
						} while (controlY <= blockMaxY);
						if (currentY <= blockMaxY)
                        {
							int dy = currentY - blockY + 1;
							blockY = currentY;
							pOffsetHiZ += blocksX * dy;
							outblockRowData += blocksXRows * dy;
							__m128 dyFv = _mm_set1_ps((float)dy);
							edgeOffsetY = _mm_fmadd_ps_soc(edgeNormalY, dyFv, edgeOffsetY);
							offsetX = _mm_add_ps(edgeNormalXBlockMinX, edgeOffsetY);
							rowDepthLeftOffsetY = _mm_fmadd_ps_soc(depthBlockDelta, dyFv, rowDepthLeftOffsetY);

							NextBlockX = blockMinX;
							CurrentSkip = 0;
						}
						else 
						{
							break;
						}						
					}
				}
				else
				{
					//Convex Optimization 2: Edge31 Block Forward Optimization
					//safe to skip: 2 * current - previous - 1
					//BlockMin     -> BlockMin + 2 * current - previous - 1
					//CurrentSkip  -> 0
					//PreviousSkip -> PreviousSkip + 1 - CurrentSkip;
					PreviousSkip = PreviousSkip + 1 - CurrentSkip; // - (CurrentSkip - PreviousSkip - 1)

					blockMinX += (CurrentSkip - PreviousSkip);

					if (DebugOccluderOccludee)
					{
						int skipCount = (CurrentSkip - PreviousSkip);// 2 * CurrentSkip - PreviousSkip - 1;
						if (skipCount > 0)
						{
							if (blockMinX > blockMaxX)
							{
								skipCount = blockMaxX - (blockMinX - skipCount) + 1;
							}
							DebugData[BlockConvexEdge31Cull] += skipCount * (1 + blockMaxY - blockY );
						}
						DebugData[BlockConvexEdge31CullRowCheck]++;
					}

					if (blockMinX <= blockMaxX) //99% here
					{
						pOffsetHiZ += blocksX;
						outblockRowData += blocksXRows;
						edgeOffsetY = _mm_add_ps(edgeOffsetY, edgeNormalY);
						offsetX = _mm_fmadd_ps_soc(edgeNormalX, _mm_set1_ps((float)blockMinX), edgeOffsetY);
						rowDepthLeftOffsetY = _mm_add_ps(rowDepthLeftOffsetY, depthBlockDelta);
						CurrentSkip = 0;
						NextBlockX = blockMinX;
					}
					else
					{
						if (DebugOccluderOccludee) 
						{
							DebugData[BlockConvexEdge31CullRowCheckPassExit]++;
						}
						break;
					}
				}
			}
			else 
			{
				pOffsetHiZ += blocksX;
				outblockRowData += blocksXRows;
				edgeOffsetY = _mm_fmadd_ps_soc(edgeNormalY, _mm_set1_ps((float)blockY) , edgeOffset);
				offsetX = _mm_fmadd_ps_soc(edgeNormalX, _mm_set1_ps((float)blockMinX), edgeOffsetY);
				rowDepthLeftOffsetY = _mm_add_ps(rowDepthLeftOffsetY, depthBlockDelta);
				NextBlockX = blockMinX;
			}
		}
	}
	while (validMask > 0);

	if (DebugOccluderOccludee)
	{
		if (bDebugOccluderOnly == false) {
			assert(DebugData[PrimitiveValidNum] == 0);
		}
	}
}

template <bool possiblyNearClipped, bool bBackFaceCulling, bool bDrawOccludee>
void Rasterizer::SplitToTwoTriangles(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache)
{
	if (DebugOccluderOccludee)
	{
		this->DebugData[P4QuadSplit]++;
		DebugData[PrimitiveValidNumQuad] = DebugData[PrimitiveValidNum];
	}
	if (bQuadToTriangleMergeOp) 
	{
		uint32_t pValidIdx = mAliveIdxMask[validMask];

		if (pValidIdx < 16) //at most two
		{
			
			uint32_t pInvalidIdx = mAliveIdxMask[15 ^ validMask];  // get invalid idx... mNonAliveIdxMask[validMask] =  mAliveIdxMask[15 ^ validMask];

			//One triangle patch would take care the quads patch with active number <= 2.
			float * Xf = (float*)X;
			float * Yf = (float*)Y;
			float * Wf = (float*)W;
			float * invWf = (float*)invW;
			float * validf = (float*)&primitiveValid;

			do
			{
				// Move index and mask to next set bit
				uint32_t fromIdx = pValidIdx & 3;
				uint32_t fromIdx2 = fromIdx | 8;
				uint32_t fromIdx3 = fromIdx | 12;
				uint32_t updateIdx = pInvalidIdx & 3;
				uint32_t updateIdx2 = updateIdx | 4;
				uint32_t updateIdx3 = updateIdx | 8;

				Xf[updateIdx] = Xf[fromIdx];
				Xf[updateIdx2] = Xf[fromIdx2];
				Xf[updateIdx3] = Xf[fromIdx3];

				Yf[updateIdx] = Yf[fromIdx];
				Yf[updateIdx2] = Yf[fromIdx2];
				Yf[updateIdx3] = Yf[fromIdx3];

				Wf[updateIdx] = Wf[fromIdx];
				Wf[updateIdx2] = Wf[fromIdx2];
				Wf[updateIdx3] = Wf[fromIdx3];

				invWf[updateIdx] = invWf[fromIdx];
				invWf[updateIdx2] = invWf[fromIdx2];
				invWf[updateIdx3] = invWf[fromIdx3];

				validf[updateIdx] = -0.0f;

				pValidIdx >>= 2;
				pInvalidIdx >>= 2;
			} while (pValidIdx > 0);

			if (possiblyNearClipped == true)
			{
				primitiveValid = _mm_and_ps(primitiveValid, _mm_xor_ps(_mm_and_ps(_mm_and_ps(W[2], W[0]), W[1]), _mm_set1_ps(-0.0f)));
			}
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);

			if (DebugOccluderOccludee) {
				DebugData[QuadToTriangleMerge] ++;
			}
			return;
		}
	}


	if (DebugOccluderOccludee) {
		DebugData[QuadToTriangleSplit] ++;
	}

	if (possiblyNearClipped == false)
	{
		if (bBackFaceCulling == false)
		{	
			//once back face culling off, data might be changed. so deep copy here
			__m128 X2[3], Y2[3], W2[3], invW2[3];
			X2[0] = X[0];
			X2[1] = X[2];
			X2[2] = X[3];
			Y2[0] = Y[0];
			Y2[1] = Y[2];
			Y2[2] = Y[3];
			W2[0] = W[0];
			W2[1] = W[2];
			W2[2] = W[3];
			invW2[0] = invW[0];
			invW2[1] = invW[2];
			invW2[2] = invW[3];
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X2, Y2, invW2, W2, primitiveValid, occluderCache);
			if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
				return;
			if (DebugOccluderOccludee)
			{
				DebugData[PrimitiveValidNum] = DebugData[PrimitiveValidNumQuad];
			}
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);
			if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
				return;
		}
		else
		{
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid, occluderCache);
			if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
				return;
			X[1] = X[0];
			Y[1] = Y[0];
			invW[1] = invW[0];
			W[1] = W[0];

			if (DebugOccluderOccludee)
			{
				DebugData[PrimitiveValidNum] = DebugData[PrimitiveValidNumQuad];
			}
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X + 1, Y + 1, invW + 1, W + 1, primitiveValid, occluderCache);
			if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
				return;
		}
	}
	else 
	{
		//direct back to triangle approach
		auto W02 = _mm_and_ps(W[2], W[0]);
		__m128 minusZero = _mm_set1_ps(-0.0f);
		if (bBackFaceCulling == false)
		{
			// All W < 0 means fully culled by camera plane
			__m128 primitiveValid0 = _mm_and_ps(primitiveValid, _mm_xor_ps(_mm_and_ps(W02, W[3]), minusZero));
			if (_mm_same_sign0(primitiveValid0) == false)
			{
				//once back face culling off, data might be changed. so deep copy here
				__m128 X2[3], Y2[3], W2[3], invW2[3];
				X2[0] = X[0];
				X2[1] = X[2];
				X2[2] = X[3];
				Y2[0] = Y[0];
				Y2[1] = Y[2];
				Y2[2] = Y[3];
				W2[0] = W[0];
				W2[1] = W[2];
				W2[2] = W[3];
				invW2[0] = invW[0];
				invW2[1] = invW[2];
				invW2[2] = invW[3];
				drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X2, Y2, invW2, W2, primitiveValid0, occluderCache);
				if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
					return;
			}
		}


		// All W < 0 means fully culled by camera plane
		__m128 primitiveValid0 = _mm_and_ps(primitiveValid, _mm_xor_ps(_mm_and_ps(W02, W[1]), minusZero));
		if (_mm_same_sign0(primitiveValid0) == false)
		{
			if (DebugOccluderOccludee)
			{
				DebugData[PrimitiveValidNum] = DebugData[PrimitiveValidNumQuad];
			}
			drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X, Y, invW, W, primitiveValid0, occluderCache);
			if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
				return;
		}

		if (bBackFaceCulling == true)
		{
			// All W < 0 means fully culled by camera plane
			primitiveValid0 = _mm_and_ps(primitiveValid, _mm_xor_ps(_mm_and_ps(W02, W[3]), minusZero));
			if (_mm_same_sign0(primitiveValid0) == false)
			{
				X[1] = X[0];
				Y[1] = Y[0];
				invW[1] = invW[0];
				W[1] = W[0];
				if (DebugOccluderOccludee)
				{
					DebugData[PrimitiveValidNum] = DebugData[PrimitiveValidNumQuad];
				}
				drawTriangle< possiblyNearClipped, bBackFaceCulling, bDrawOccludee>(X + 1, Y + 1, invW + 1, W + 1, primitiveValid0, occluderCache);
				if (bDrawOccludee && occluderCache->mRasterizedOccludeeVisible && !bDrawOccludeeToDepthMap)
					return;
			}
		}
	}
}

//draw point p, q here
void Rasterizer::drawLine(float* p, float*q) 
{
	float ratio = MeshLineScale;
	if(ratio != 1.0f)
	{
		p[0] = (p[0] - m_width * 0.5f) * ratio + m_width * 0.5f;
		p[1] = (p[1] - m_height * 0.5f) * ratio + m_height * 0.5f;
		q[0] = (q[0] - m_width * 0.5f) * ratio + m_width * 0.5f;
		q[1] = (q[1] - m_height * 0.5f) * ratio + m_height * 0.5f;
	}
	if (p[0] > q[0])
	{
		std::swap(p, q);
	}

	if ((p[1] < 0 && q[1] < 0) || (p[1] >= m_height && q[1] >= m_height))
	{
		return;
	}

	if ((p[0] < 0 && q[0] < 0) || (p[0] >= m_width && q[0] >= m_width))
	{
		return;
	}

	{
		if (p[0] < 0)
		{
			float r = (0.5f - p[0]) / (q[0] - p[0]);
			p[2] = (q[2] - p[2])  * r + p[2];
			p[1] = (q[1] - p[1])  * r + p[1];
			p[0] = 0.5f;
		}
		if ( (p[1] < 0 && q[1] < 0) || (p[1] >= m_height && q[1] >= m_height))
		{
			return;
		}
		if (q[0] >= m_width)
		{
			float r = (m_width - 0.5f - p[0]) / (q[0] - p[0]);
			q[2] = (q[2] - p[2])  * r + p[2];
			q[1] = (q[1] - p[1])  * r + p[1];
			q[0] = (float)m_width - 0.5f;
		}
		if ((p[1] < 0 && q[1] < 0) || (p[1] >= m_height && q[1] >= m_height))
		{
			return;
		}

		if (p[1] > q[1])
		{
			std::swap(p, q);
		}

		if (p[1] < 0)
		{
			float r = (0.5f - p[1]) / (q[1] - p[1]);
			p[2] = (q[2] - p[2]) * r + p[2];
			p[0] = (q[0] - p[0]) * r + p[0];
			p[1] = 0.5f;
		}

		if ( (p[0] < 0 && q[0] < 0) || (p[0] >= m_width && q[0] >= m_width))
		{
			return;
		}

		if (q[1] >= m_height)
		{
			float r = ((float)m_height - 1 - p[1]) / (q[1] - p[1]);
			q[2] = (q[2] - p[2]) * r + p[2];
			q[0] = (q[0] - p[0]) * r + p[0];
			q[1] = (float)m_height - 0.5f;
		}


		if ((p[0] < 0 && q[0] < 0) || (p[0] >= m_width && q[0] >= m_width))
		{
			return;
		}
	}

	p[2] = std::max<float>(p[2], MIN_PIXEL_DEPTH_FLOAT);
	q[2] = std::max<float>(q[2], MIN_PIXEL_DEPTH_FLOAT);

	if (mDebugRenderType == RenderType::Point || mDebugRenderType == RenderType::MeshPoint) //3 draw points only
	{
		drawPixel((int)p[0], (int)p[1], p[2]);
		drawPixel((int)q[0], (int)q[1], q[2]);
		return;
	}

	// line algorithm reference:
	// https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf
	int x0 = (int)p[0];
	int y0 = (int)p[1];

	int x1 = (int)q[0];
	int y1 = (int)q[1];

	int dx = x1 - x0;
	int dy = y1 - y0;
	int stepx, stepy;

	if (dy < 0) { dy = -dy; stepy = -1; }
	else { stepy = 1; }
	if (dx < 0) { dx = -dx; stepx = -1; }
	else { stepx = 1; }
	dy <<= 1; /* dy is now 2*dy */
	dx <<= 1; /* dx is now 2*dx */

	drawPixel(x0, y0, p[2]);
	if (dx == 0 && dy == 0) 
	{
		return;
	}

	if (x0 < 0 || x0 >= (int)m_width || y0 < 0 || y0 >= (int)m_height ||
		x1 < 0 || x1 >= (int)m_width || y1 < 0 || y1 >= (int)m_height)
	{		
		return;
	}


	if (dx > dy) {
		float zSlope = (q[2] - p[2]) / (q[0] - p[0]);

		float p2p0zSlope = p[2] - p[0] * zSlope;
		int fraction = dy - (dx >> 1);
		while (x0 != x1) {
			x0 += stepx;
			if (fraction >= 0) {
				y0 += stepy;
				fraction -= dx;
			}
			fraction += dy;
			//drawPixelSafe(x0, y0, p[2] + (x0 - p[0]) * zSlope);
			drawPixelSafe(x0, y0, x0 * zSlope + p2p0zSlope);
		}
	}
	else {
		float zSlope = (q[2] - p[2]) / (q[1] - p[1]);

		float p2p1zSlope = p[2] - p[1] * zSlope;
		int fraction = dx - (dy >> 1);
		while (y0 != y1) {
			if (fraction >= 0) {
				x0 += stepx;
				fraction -= dy;
			}
			y0 += stepy;
			fraction += dx;
			drawPixelSafe(x0, y0, y0 * zSlope + p2p1zSlope);
			//drawPixelSafe(x0, y0, p[2] + (y0 - p[1]) * zSlope);
		}
	}
}

void Rasterizer::drawPixelSafe(int x, int y, float zf)
{
	int z = *(int *)&zf;
	z = z >> 12;  

	z = std::min<uint16_t>(65535, z + 256);//move forward the value

	if (mDebugRenderType == MeshLine) z = 65535;
	int pixelIdx = y * this->m_width + x;
	m_depthBufferPointLines[pixelIdx] = std::max<uint16_t>(m_depthBufferPointLines[pixelIdx], z);
	if(bDebugOccluderOnly){
		if (x == bDebugOccluderPixelX && y == bDebugOccluderPixelY) 
		{
			this->DebugData[BlockPacketPrimitiveDebug] = 1;
		}
	}
}
void Rasterizer::drawPixel(int x, int y, float zf)
{
	//float maxf = decompressFloat(65535);
	//if (zf >= maxf) return;
	if (x < 0 || x >=(int) m_width || y < 0 || y >= (int)m_height)
	{
		return;
	}
	drawPixelSafe(x, y, zf);


	//mapping pixel (x, y) to 64 bit
	//maskData[0] |= (uint64_t)1 << (8 * idxX + 7 - idxY);
	//maskData[0] |= (uint64_t)1 << ( (idxX<<3) | (7 ^ idxY));
}


template <int PrimitveEdgeNum>
void Rasterizer::HandleDrawMode(__m128* x, __m128* y, __m128* z, uint32_t alivePrimitive, OccluderRenderCache* OccluderCache)
{	
	__m128 z2[3];
	if (PrimitveEdgeNum == 3)
	{
		//reset the true data, here the z is actually invW. Set this way to make the main code gap shorter
		__m128* invW = z;
		z = z2;

		__m128 c0 = OccluderCache->c0;
		__m128 c1 = OccluderCache->c1;
		z[0] = _mm_fmadd_ps_soc(invW[0], c1, c0);
		z[1] = _mm_fmadd_ps_soc(invW[1], c1, c0);
		z[2] = _mm_fmadd_ps_soc(invW[2], c1, c0);
	}

	if (bDebugOccluderOnly) {
		if (-1 == bDebugOccluderPixelX && -1 == bDebugOccluderPixelY)
		{
			this->DebugData[BlockPacketPrimitiveDebug] = 1;
		}
	}

	__m128 px[PrimitveEdgeNum];
	__m128 py[PrimitveEdgeNum];

	for (int i = 0; i < PrimitveEdgeNum; ++i)
	{
		px[i] = (_mm_mul_ps(x[i], _mm_set1_ps(8)));
		py[i] = (_mm_mul_ps(y[i], _mm_set1_ps(8)));

	}

	float * fpx = (float*)px;
	float * fpy = (float*)py;
	float * fpz = (float*)z;
	do
	{
		int j = alivePrimitive & 3;
		if (DebugOccluderOccludee){
			this->DebugData[BlockPacketPrimitive] = j;
		}
		alivePrimitive >>= 2;
		float p[3];
		float q[3];
		for (int i = 0; i < PrimitveEdgeNum; i++)
		{
			int idx = j | (i << 2);
			p[0] = fpx[idx];
			p[1] = fpy[idx];
			p[2] = fpz[idx];


			int k = (i + 1) % PrimitveEdgeNum;
			idx = j | (k << 2);
			q[0] = fpx[idx];
			q[1] = fpy[idx];
			q[2] = fpz[idx];

			if (q[2] < 0 && p[2] < 0)  continue;
			float flipRatio = 10.0f;  //this value could be infinite
			if (p[2] < 0) {
				p[0] = q[0] + (q[0] - p[0]) * flipRatio;
				p[1] = q[1] + (q[1] - p[1]) * flipRatio;
			}
			if (q[2] < 0) {
				q[0] = p[0] + (p[0] - q[0]) * flipRatio;
				q[1] = p[1] + (p[1] - q[1]) * flipRatio;
			}
			drawLine(p, q);

		}
	} while (alivePrimitive != 0);
}

template void Rasterizer::rasterize<0>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<1>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<2>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<3>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<4>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<5>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<6>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<7>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<8>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<9>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<10>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<11>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<12>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<13>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<14>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<15>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);


template void Rasterizer::rasterize<16>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<17>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<18>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<19>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<4>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<21>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<6>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<23>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<24>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<25>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<26>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
template void Rasterizer::rasterize<27>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<12>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<29>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);
//template void Rasterizer::rasterize<14>(common::OccluderMesh& raw);
template void Rasterizer::rasterize<31>(common::OccluderMesh& raw, OccluderRenderCache* occluderCache);




template bool Rasterizer::queryVisibility<true, false, false>(const float* minmaxf, OccluderRenderCache* OccluderCache);
template bool Rasterizer::queryVisibility<false, true, false>(const float* minmaxf, OccluderRenderCache* OccluderCache);
template bool Rasterizer::queryVisibility<false, false, false>(const float* minmaxf, OccluderRenderCache* OccluderCache);
template bool Rasterizer::queryVisibility<false, true, true>(const float* minmaxf, OccluderRenderCache* OccluderCache);



template bool Rasterizer::queryVisibility_OBB<false, true, false>(const float* minmaxf);
template bool Rasterizer::queryVisibility_OBB<false, false, false>(const float* minmaxf);
template bool Rasterizer::queryVisibility_OBB<false, true, true>(const float* minmaxf);


template bool Rasterizer::queryVisibility_QUAD<true, false>(const float* minmaxf);
template bool Rasterizer::queryVisibility_QUAD<false, false>(const float* minmaxf);
template bool Rasterizer::queryVisibility_QUAD<true, true>(const float* minmaxf);


template bool Rasterizer::query2D<true, false>(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY, uint16_t maxZ);
template bool Rasterizer::query2D<false, true>(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY, uint16_t maxZ);
template bool Rasterizer::query2D<false, false>(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY, uint16_t maxZ);



template void Rasterizer::batchQueryWithTree<true, true>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);
template void Rasterizer::batchQueryWithTree<true, false>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);
template void Rasterizer::batchQueryWithTree<false, true>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);
template void Rasterizer::batchQueryWithTree<false, false>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);


template  void Rasterizer::batchQuery<true>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);
template  void Rasterizer::batchQuery<false>(const float * bbox, unsigned int nMesh, bool * results, bool obbMode);

template void Rasterizer::HandleDrawMode<3>(__m128* x, __m128* y, __m128* z, uint32_t alivePrimitive, OccluderRenderCache* OccluderCache);
template void Rasterizer::HandleDrawMode<4>(__m128* x, __m128* y, __m128* z, uint32_t alivePrimitive, OccluderRenderCache* OccluderCache);

template void Rasterizer::drawTriangle<true, true, false>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<false, true, false>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<true, false, false>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<false, false, false>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<true, true, true>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<false, true, true>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);
template void Rasterizer::drawTriangle<true, false, true>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, OccluderRenderCache* occluderCache);

template void Rasterizer::drawQuad<true, false>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, __m128* edgeNormalsX, __m128* edgeNormalsY, __m128* areas, OccluderRenderCache* occluderCache);
template void Rasterizer::drawQuad<true, true>(__m128* x, __m128* y, __m128* invW, __m128* W, __m128 primitiveValid, __m128* edgeNormalsX, __m128* edgeNormalsY, __m128* areas, OccluderRenderCache* occluderCache);
////template void Rasterizer::drawQuad<false>(__m128 * x, __m128 * y, __m128 * invW, __m128 * W, __m128 primitiveValid, __m128* edgeNormalsX, __m128* edgeNormalsY, __m128* areas);


template void Rasterizer::SplitToTwoTriangles<true, true, true>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<true, false, true>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<false, true, true>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<false, false, true>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<true, true, false>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<true, false, false>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<false, true, false>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);
template void Rasterizer::SplitToTwoTriangles<false, false, false>(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask, OccluderRenderCache* occluderCache);

void PixelMaskBound::calculateMask()
{

	//rowMask |= 1 << (8 * x + (7 - y));
	uint64_t yMask = 255;
	PixelMinXMask[7] = yMask << (7 << 3);
	for (int x = 6; x >= 0; --x)
	{
		uint64_t current = yMask << (x << 3);
		PixelMinXMask[x] = current | PixelMinXMask[x + 1];
	}

	PixelMaxXMask[0] = yMask;
	for (int x = 1; x <= 7; ++x)
	{
		uint64_t current = yMask << (x << 3);
		PixelMaxXMask[x] = current | PixelMaxXMask[x - 1];
	}

	//		rowMask |= 1 << (8 * x + (7 - y));
	uint64_t xMask = 0;
	for (int x = 0; x <= 7; x++)
	{
		uint64_t rowMaskBit = 1;
		xMask |= rowMaskBit << (8 * x);
	}
	PixelMinYMask[7] = xMask;
	for (int y = 6; y >= 0; y--)
	{
		uint64_t current = xMask << (7 - y);
		PixelMinYMask[y] = current | PixelMinYMask[y + 1];
	}


	PixelMaxYMask[0] = xMask << (7 - 0);
	for (int y = 1; y <= 7; y++)
	{
		uint64_t current = xMask << (7 - y);
		PixelMaxYMask[y] = current | PixelMaxYMask[y - 1];
	}

	if (PairBlockNum <= PureCheckerBoardApproach+1) 
	{
		for (int idx = 0; idx <= 7; idx++)
		{
			PixelMinXMask[idx] = CheckerBoardTransform(PixelMinXMask[idx]);
			PixelMaxXMask[idx] = CheckerBoardTransform(PixelMaxXMask[idx]);
			PixelMinYMask[idx] = CheckerBoardTransform(PixelMinYMask[idx]);
			PixelMaxYMask[idx] = CheckerBoardTransform(PixelMaxYMask[idx]);
		}
	}
}

} // namespace util
#endif
