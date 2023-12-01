//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
/*
** Code from https://github.com/rawrunprotected/rasterizer/blob/master/SoftwareRasterizer/Rasterizer.h under CC0 1.0 Universal (CC0 1.0) Public Domain Dedication.
*/

#pragma once

#include <memory>
#include <vector>
#include "Common/MathUtil.h"
#include "Common/CompilerSpecificSIMD.h"
#include <cassert>
#if defined(SDOC_NATIVE)
#include <iostream>
#endif
#include "SOCUtil.h"

enum SDOCDebug
{
	//P4 stands for Primitive 4. Here Primitive just means Triangle
	P4Valid0 = 0,
	P4Valid1 = 1,
	P4Valid2 = 2,
	P4Valid3 = 3,
	P4Valid4 = 4,
	P4DrawTriangle = 5,

	FastRowBitCull = 6,
	FastBlockHizCull = 7,
	FastBlockEmptyPass = 8,
	BlockMaskPass = 9,
	BlockPixelPass = 10,

	BlockCorrectMinPass = 11,
	BlockEmptyPass = 12,
	BlockPixelCull = 13,
	FastHalfPlaneCull = 14,
	OccludeeNearClipPass = 15,
	OccludeeFrustumCull = 16,
	OccludeeQuery2d = 17,
	OccludeeCull = 18,
	OccludeeOnePixelExpandCheck = 19,
	QueryBlockDoWhileIfSave = 20,
	InterleaveQuerySkipPass = 21,

	MaxOccludeeZ = 22,
	BlockRowCheck = 23,

	FastBlockDepthCompare = 24,
	FastPlaneBlockDepthCompareSave = 25, 

	P4Total = 26,
	P4Rasterized = 27,
	PrimitiveTotalInput = 28,
	PrimitiveRasterizedNum = 29,
	PrimitiveEarlyHiZCull = 30,
	BlockRenderTotal = 31,

	P4NearClipInput = 32,
	PrimitiveNearClipeRasterized = 33,

	OccluderCulled = 34,
	OccluderRasterized = 35,
	P4CameraNearPlaneCull = 36,
	P4BackFaceCull = 37,
	P4FrustumCull = 38,
	P4EarlyHizCull = 39,
	P4EarlyHizCullPass = 40,
	P4PassCull = 41,
	P4PassFrustumCull = 42,

	PrimitiveCameraNearPlaneCull = 43,
	PrimitiveBackFaceCull = 44,
	PrimitiveFrustumCull = 45,
	PrimitiveValidNum = 46,
	PrimitiveValidNumQuad = 47,
	PrimitiveDegenerateCull = 48,

	OccluderFrustumCulled = 49,

	BlockHizQuickCompare = 53,
	BlockDoWhileIfSave = 54,
	BlockTotal = 55,
	BlockConvexRow10Cull = 56,
	BlockConvexRow10CullOverhead = 57,
	BlockOneSureZeroCull = 58,

	BlockConvexEdge31Cull = 59,
	BlockConvexEdge31CullRowCheck = 60,
	BlockConvexEdge31CullRowCheckPass = 80,
	BlockConvexEdge31CullRowCheckPassExit = 81,
	BlockConvexAllZeroRow = 82,
	BlockConvexRow00Cull = 83,
	BlockConvexRow00CullNextScanRows = 84,
	BlockConvexEdge24Cull = 85,
	BlockConvexEdge24Check = 86,

	BlockPrimitiveMaxLessThanMinCull = 61,
	BlockMaskJointZeroCull = 62,
	BlockMaxLessThanMinCull = 63,
	BlockTotalPrimitives = 64,

	BlockAABBClipToZero = 65,
	BlockRenderPartial = 66,
	BlockRenderFull = 67,
	BlockRenderInitial = 68,
	BlockRenderInitialPartial = 69,
	BlockRenderInitialFull = 70,


	BlockMinCompute4 = 90,
	BlockMinUseOne = 91,


	BlockPacketID = 92,
	BlockPacketPrimitive = 93,
	BlockPacketPrimitiveDebug = 94,






	BlockEmptyBlock = 110,
	BlockMinValidDepth = 111,


	RasterizedOccluderTotalTriangles = 115,
	RasterizedOccluderTotalVertices = 116,



		OccludeeQueryMaxPass = 120,
		OccluderQueryMaxPass = 121,
		CurrentOccludeeIdx = 122,


		PrimitiveRasterizedQuadNum = 130,
		BatchQuad4Rasterized = 131,
		BatchQuadConvex = 132,


		P4QuadFrustumCull = 140,
		P4QuadSplit = 141,
		P4QuadFrustumPass = 142,



		QuadToTriangleMerge = 150,
	QuadToTriangleSplit = 151,
	QuadProcessed = 152,

};
namespace util
{



struct OccluderRenderCache
{
	bool NeedsClipping = false;
	bool FlipOccluderFace = false;

	__m128 mat[4]; 

	__m128 c0;
	__m128 c1;
	__m128 NegativeC1;

	void prepareCache(__m128& edge0, __m128& edge1, __m128&edge2)
	{
		mat[0] = edge0;
		mat[1] = edge1;
		mat[2] = edge2;
	}
	__m128 FullMeshInvExtents;
	__m128 FullMeshMinusRefMinInvExtents;
};

class PrimitiveBoundaryClipCache
{
public:
	PrimitiveBoundaryClipCache() 
	{
		this->calculateMask(); //initialize the mask
	}

	uint32_t MinBlockX;
	uint32_t MaxBlockX;
	uint32_t MinBlockY;
	uint32_t MaxBlockY;

	__m128i PrimitivePixelBounds[4];
	inline uint64_t GetPixelAABBMask(uint32_t blockX, uint32_t blockY)
	{
        return  MinXBoundary[blockX == MinBlockX]
                & MaxXBoundary[blockX == MaxBlockX]
                & MinYBoundary[blockY == MinBlockY]
                & MaxYBoundary[blockY == MaxBlockY];
	}

	void UpdatePixelAABBData(int primitiveIdx)
	{
		uint16_t *  mPixelData = (uint16_t*)PrimitivePixelBounds + (primitiveIdx<<1); //mask content, 0~7

		MinXBoundary[1] = this->PixelMinXMask[mPixelData[0]];
		MaxXBoundary[1] = this->PixelMaxXMask[mPixelData[8]];
		MinYBoundary[1] = this->PixelMinYMask[mPixelData[16]];
		MaxYBoundary[1] = this->PixelMaxYMask[mPixelData[24]];

		MinBlockX = mPixelData[1];
		MaxBlockX = mPixelData[9];
		MinBlockY = mPixelData[17];
		MaxBlockY = mPixelData[25];
	}

private:

	uint64_t MinYBoundary[2];
	uint64_t MaxYBoundary[2];
	uint64_t MinXBoundary[2];
	uint64_t MaxXBoundary[2];

	void calculateMask();
public:
	uint64_t PixelMinXMask[8];
	uint64_t PixelMinYMask[8];
	uint64_t PixelMaxXMask[8];
	uint64_t PixelMaxYMask[8];
	
};
struct InterleaveConfig 
{
	bool IsRotating = false;
	bool mRenderRight = true;
	bool CurrentFrameInterleaveDrawing = false;

	void configRotating(bool value) 
	{
		IsRotating = value;
		MaskIndex = (int)value;
		mBlock_XRightStart = mBlock_XRightStartAll[MaskIndex];
		mBlock_XLeftEnd = mBlock_XLeftEndAll[MaskIndex];
		mPixel_XRightStart = mPixel_XRightStartAll[MaskIndex];
		mPixel_XLeftEnd = mPixel_XLeftEndAll[MaskIndex];
	}
	unsigned int MaskIndex = 0; //set to 1 when rotating for high accuracy
	unsigned int mOctuple = 0;

	//current frame data frame update when config rotation
	uint32_t mBlock_XRightStart = 0;
	uint32_t mBlock_XLeftEnd = 0;
	uint32_t mPixel_XRightStart = 0;
	uint32_t mPixel_XLeftEnd = 0;
	uint64_t InterleaveFrame = 2;

	

private:


	uint32_t mBlock_XRightStartAll[2];
	uint32_t mBlock_XLeftEndAll[2];
	uint32_t mPixel_XRightStartAll[2];
	uint32_t mPixel_XLeftEndAll[2];


	void updateConfig(int keyIndex, int octuple, uint32_t totalBlockX)
	{

		int SkipRegion = (totalBlockX * (8 - octuple)) >> 3;
		//this->mBlock_XRightStartAll[keyIndex] = SkipRegion - 1; //no overlap region
		this->mBlock_XRightStartAll[keyIndex] = SkipRegion ; //at least one block is overlapped
		this->mBlock_XLeftEndAll[keyIndex] = (totalBlockX - SkipRegion);
		

		this->mPixel_XRightStartAll[keyIndex] = (this->mBlock_XRightStartAll[keyIndex] << 3);
		this->mPixel_XLeftEndAll[keyIndex] = (this->mBlock_XLeftEndAll[keyIndex] << 3) | 7;


		
	}

public:

	void config(uint32_t m_blocksX, uint32_t m_blocksY)
	{
		
		updateConfig(0, this->mOctuple, m_blocksX);
		updateConfig(1, 6, m_blocksX);
		configRotating(MaskIndex);

	}


	

};

class Rasterizer
{
public:
    Rasterizer();
    ~Rasterizer();


    void setResolution(unsigned int width, unsigned int height);

	void UpdateFrustumCullPlane();

	void setModelViewProjectionT(const common::Matrix4x4 &localToClip);

    void configBeforeRasterization();



	template <bool bQueryOccluder, bool OccludeeWidth1024, bool multiThreadQuery>
    bool queryVisibility( const float* minmaxf);

	void doRasterize(common::OccluderMesh& raw);


	template <bool bQueryOccluder, bool bOccludeeWidth1024>
    bool query2D(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY, uint16_t maxZ);

	// Currently unused
    bool readBackDepth(unsigned char* target, common::DumpImageMode mode);


	template <bool bHasTreeData, bool OccludeeWidth1024>
	void batchQueryWithTree(const float * bbox, unsigned int nMesh, bool * results);

	template <bool OccludeeWidth1024>
	void batchQuery(const float * bbox, unsigned int nMesh, bool * results);



	size_t getMemoryUsage() const;

	bool mWidthIn1024 = true;
	bool InterleaveRendering = false;

	bool ShowOccludeeInDepthMap = true;
	bool ShowOccludeeInDepthMapNext = false; //introduce this to enforce that ShowOccludeeInDepthMap could be changed only at beginning of the frame
	bool mDebugRenderMode = false;    


	//config vertices of model is stored by clockwise or counter-clockwise
	bool mClockWise = false;


	uint8_t mDebugRenderType = 0;     //0 means default(No point, line), 1 means point, 2 means lines
	


	uint32_t mCurrValidOccluderNum = 0;
	uint32_t mLastOccluderNum = 0;
	uint64_t mGlobalFrameNum = 0;

	void configGlobalFrameNum(uint64_t frameNum);

		

	float mNearPlane = 1.0f;

private:

	template <bool bPixelAABBClippingQuad>
	void drawQuad(__m128 * x, __m128 * y, __m128 * invW, __m128 * W,  __m128 primitiveValid, __m128* edgeNormalsX, __m128* edgeNormalsY, __m128* areas);

	template <int RASTERIZE_CONFIG>
	void rasterize(common::OccluderMesh& raw);

    inline void precomputeRasterizationTable();

	uint64_t mCheckerBoardQueryMask[36];
	uint8_t mCheckerBoardQueryOffset[8];
    __m128 xFactors[2];
    

	__m128 m_localToClip[4];
	__m128 m_FrustumPlane[6];


    std::vector<uint64_t> m_precomputedRasterTables;
	uint64_t* m_pMaskTable = nullptr;
	//std::vector<uint16_t> m_hiZ; //m_hiz is not reuse second half of m_precomputedRasterTables

	std::vector<__m128i> m_depthBuffer;
	uint64_t *m_pDepthBuffer = nullptr;


	uint16_t *m_pHiz = nullptr;
	uint16_t *m_pHizMax = nullptr;
	
	//for max data when value == 0, means all cells are 0, for quick initial update
	//for interleave mode, used by coherent mode to storePixel previous cache


    uint32_t m_width = 0;
	uint32_t m_height = 0;
	uint32_t m_blocksX = 0;
	uint32_t m_blocksY = 0;
	int mBlockWidthMin;
	uint32_t mBlockWidthMax;

	uint32_t m_blocksYMinusOne = 0;
	uint32_t m_blocksXFullDataRows=0;
	
	//temp cache to avoid convert between int & float
	__m128i m_MaxCoord_WHWH;
	__m128i m_MaxCoordOccludee_WHWH;

	__m128i m_MaxCoord_WHWHOccluder;
	__m128i m_MinCoord_WHWHOccluder;


	
	PrimitiveBoundaryClipCache* mPrimitiveBoundaryClip = nullptr;

#if defined( SUPPORT_ALL_FEATURE)
	//updateBlockWithMaxZ
	inline void updateBlockWithMaxZ(__m128 rowDepthLeft, __m128 rowDepthRight, uint64_t blockMask, __m128 depthRowDelta, __m128i primitiveMaxZV, __m128i * out, uint16_t * pBlockRowHiZ);

	inline void updateBlock(__m128i* depthRows, uint64_t blockMask, __m128i * out, uint16_t * pBlockRowHiZ);
#endif

	


	inline void updateBlockMSCBPartial(uint32_t * depth32, uint64_t blockMask, __m128i* out, uint64_t* maskData, uint16_t * pBlockRowHiZ, uint16_t maxBlockDepth);
	

	//drawTriangle
	template <bool possiblyNearClipped, bool bBackFaceCull>
	void drawTriangle(__m128* x, __m128* y, __m128* invW, __m128* W,  __m128 primitiveValid);

	template <bool possiblyNearClipped, bool bBackFaceCulling>
	void SplitToTwoTriangles(__m128* X, __m128* Y, __m128* W, __m128* invW, __m128 primitiveValid, int validMask);
public:
	void onOccluderRenderFinish();


	//temp variables for occluder after query before rasterizer
	OccluderRenderCache mOccluderCache;


	inline bool inFrustum(__m128& boundsMin, __m128& boundsMax, __m128& extents);
	InterleaveConfig mInterleave;
	uint32_t m_blockSize = 0;
	uint32_t m_HizBufferSize = 0;

	uint32_t m_totalPixels = 0;



	void configCoherent();



	uint16_t mCurrentOccludeeDepth = 0;






	uint64_t mCurrentOccludee = 0;
	std::vector<uint64_t> mOccludeeResults;


	uint32_t * DebugData = nullptr; //Debug counters


	//__m128 mBatchMinZ;
	std::vector<uint16_t> m_depthBufferPointLines;

	uint64_t mIndexBuffer[4];

	//use 1~15 only, used to map 1 bit alive mask to 2 bit alive mask
	//where the 2 bit value stand for the index
	uint8_t mAliveIdxMask[16]; 

	std::vector<uint64_t> mAnyDataBlockMask;

private:
	template <int PrimitveEdgeNum>
	void HandleDrawMode(__m128* x, __m128* y, __m128* z, uint32_t alivePrimitive);

	void drawPixel(int x, int y, float zf);
	void drawPixelSafe(int x, int y, float zf);
	void drawLine(float* p, float*q);
#if defined( SUPPORT_ALL_FEATURE)
	void DumpColumnBlock(uint64_t t0, uint64_t t1, uint64_t t2, uint16_t maxDepth, uint64_t blockMask);
#endif

public:
	bool mUpdateAnyBlock = false;
	bool mOccludeeTrueAsCulled = false;
	uint16_t* mOccludeeTreeData = nullptr;

	const __m128 * pLocalToClipRow = nullptr;
};

} // namespace util

